#include "bdpt.h"
#include "stats.h"
#include "paramset.h"
#include "scoped.h"

STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIB("Integrator/Path length", pathLength);

int BDPTIntegrator::generateCameraSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                          const Camera &camera, const Point2f &pFilm, Vertex *path)
{
    if (maxDepth == 0) return 0;
    ProfilePhase _(Stage::BDPTGenerateSubpath);

    // Sample initial ray for camera subpath
    CameraSample camSample;
    camSample.pFilm = pFilm;
    camSample.pLens = sampler.get2D();
    camSample.time = sampler.get1D();
    RayDifferential ray;
    Spectrum beta = camera.generateRayDifferential(camSample, &ray);
    ray.scaleDifferentials(1 / sqrtf(sampler.samplesPerPixel));

    // Generate first vertex on camera subpath and start random walk
    float pdfPos, pdfDir;
    path[0] = Vertex::createCamera(&camera, ray, beta);
    camera.pdf_We(ray, &pdfPos, &pdfDir);
    return randomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1, TransportMode::Radiance,
                      path + 1) + 1;
}

int BDPTIntegrator::generateLightSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                         float time, const Distribution1D &lightDistrib,
                                         const LightIndexMap &lightToIndex, Vertex *path)
{
    if (maxDepth == 0) return 0;
    ProfilePhase _(Stage::BDPTGenerateSubpath);

    // Sample initial ray for light subpath
    float pdfChoice;
    int lightNum = lightDistrib.sampleDiscrete(sampler.get1D(), &pdfChoice);
    const auto &light = scene.lights[lightNum];
    RayDifferential ray;
    Normal3f nLight;
    float pdfPos, pdfDir;
    Spectrum Le = light->sample_Le(sampler.get2D(), sampler.get2D(), time, &ray, &nLight, &pdfPos, &pdfDir);
    if (Le.isBlack() || pdfPos == 0 || pdfDir == 0) return 0;

    // Generate first vertex on light subpath and start random walk
    path[0] = Vertex::createLight(light.get(), ray, nLight, Le, pdfPos * pdfChoice);
    Spectrum beta = Le * absDot(nLight, ray.d) / (pdfChoice * pdfPos * pdfDir);
    int nVertices = randomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1, TransportMode::Importance,
                               path + 1);

    // Correct subpath sampling densities for infinite area lights
    if (path[0].isInfiniteLight()) {
        if (nVertices > 0) {
            path[1].pdfFwd = pdfPos;
            if (path[1].isOnSurface())
                path[1].pdfFwd *= absDot(ray.d, path[1].ns());
        }
        path[0].pdfFwd = infiniteLightDensity(scene.lights, lightDistrib, lightToIndex, ray.d);
    }

    return nVertices + 1;
}

int BDPTIntegrator::randomWalk(const Scene &scene, RayDifferential &ray, Sampler &sampler, MemoryArena &arena,
                               Spectrum beta, float pdf, int maxDepth, TransportMode mode, Vertex *path)
{
    if (maxDepth == 0) return 0;
    float pdfFwd = pdf, pdfRev = 0; // unit solid angle
    int bounces = 0;
    while (true) {
        MediumInteraction mi;
        // Trace a ray and sample te medium, if any
        SurfaceInteraction isect;
        bool foundIsect = scene.intersect(ray, &isect);
        if (ray.medium)
            beta *= ray.medium->sample(ray, sampler, arena, &mi);
        if (beta.isBlack()) break;
        Vertex &vertex = path[bounces], &prev = path[bounces - 1];

        if (mi.isValid()) {
            // Record medium interaction in path and compute forward density
            vertex = Vertex::createMedium(mi, beta, pdfFwd, prev);
            if (++bounces >= maxDepth) break;

            // Sample direction and compute reverse density at preceding vertex
            Vector3f wi;
            pdfFwd = pdfRev = mi.phase->sample_p(-ray.d, &wi, sampler.get2D());
            ray = mi.spawnRay(wi);
        }
        else {
            // Handle surface interaction for path generation
            if (!foundIsect) {
                // Capture escaped ray when tracing from the camera
                if (mode == TransportMode::Radiance) {
                    vertex = Vertex::createLight(EndpointInteraction(ray), beta, pdfFwd);
                    ++bounces;
                }
                break;
            }

            // Compute scattering functions for mode and skip over medium boundaries
            isect.computeScatteringFunctions(ray, arena, true, mode);
            if (!isect.bsdf) {
                ray = isect.spawnRay(ray.d);
                continue;
            }

            // Initiaize vertex with surface intersection information
            vertex = Vertex::createSurface(isect, beta, pdfFwd, prev);
            if (++bounces >= maxDepth) break;

            // Sample BSDF at current vertex and compute reverse probability
            Vector3f wi, wo = isect.wo;
            BxDFType flags;
            Spectrum f = isect.bsdf->sample_f(wo, &wi, sampler.get2D(), &pdfFwd, BSDF_ALL, &flags);
            if (f.isBlack() || pdfFwd == 0) break;
            beta *= f * absDot(wi, isect.shading.n) / pdfFwd;
            pdfRev = isect.bsdf->pdf(wi, wo, BSDF_ALL);
            if (flags & BSDF_SPECULAR) {
                vertex.delta = true;
                pdfRev = pdfFwd = 0;
            }
            beta *= correctShadingNormal(isect, wo, wi, mode);
            ray = isect.spawnRay(wi);
        }

        // Compute reverse area density at preceding vertex
        prev.pdfRev = vertex.convertDensity(pdfRev, prev);
    }
    return bounces;
}

Spectrum BDPTIntegrator::connectVertices(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices,
                                         int s, int t, const Distribution1D &lightDistrib,
                                         const LightIndexMap &lightToIndex, const Camera &camera,
                                         Sampler &sampler, Point2f *pRaster, float *misWeight)
{
    ProfilePhase _(Stage::BDPTConnectSubpaths);
    Spectrum L;
    // Ignore invalid connections related to infinite area lights
    if (t > 1 && s != 0 && cameraVertices[t - 1].type == VertexType::Light)
        return 0;

    // Perform connection and write contribution to L
    Vertex sampled;
    if (s == 0) // no light vertex
    {
        // Interpret the camera subpath as a complete path
        const Vertex &pt = cameraVertices[t - 1]; // last camera vertice
        if (pt.isLight())
            L = pt.compute_Le(scene, cameraVertices[t - 2]) * pt.beta;
    }
    else if (t == 1) // only one camera vertex
    {
        // Sample a point on the camera and connect it to the light subpath
        const Vertex &qs = lightVertices[s - 1];
        if (!qs.isConnectible()) return 0;
        VisibilityTester visib;
        Vector3f wi;
        float pdf;
        Spectrum Wi = camera.sample_Wi(qs.getInteraction(), sampler.get2D(), &wi, &pdf, pRaster, &visib);
        if (pdf == 0 || Wi.isBlack()) return 0;
        sampled = Vertex::createCamera(&camera, visib.getP1(), Wi / pdf);
        L = qs.beta * qs.compute_f(sampled, TransportMode::Importance) * sampled.beta;
        if (!L.isBlack()) L *= visib.compute_Tr(scene, sampler);
        if (qs.isOnSurface()) L *= absDot(wi, qs.ns());
    }
    else if (s == 1) // only one light vertex
    {
        // Sample a point on the light and connect it to the camera subpath
        const Vertex &pt = cameraVertices[t - 1];
        if (pt.isConnectible()) {
            float lightPdf;
            VisibilityTester visib;
            Vector3f wi;
            float pdf;
            int lightNum = lightDistrib.sampleDiscrete(sampler.get1D(), &lightPdf);
            const auto &light = scene.lights[lightNum];
            Spectrum lightWeight = light->sample_Li(pt.getInteraction(), sampler.get2D(), &wi, &pdf, &visib);
            if (pdf == 0 || lightWeight.isBlack()) return 0;

            EndpointInteraction ei(visib.getP1(), light.get());
            sampled = Vertex::createLight(ei, lightWeight / (pdf * lightPdf), 0);
            sampled.pdfFwd = sampled.pdfLightOrigin(scene, pt, lightDistrib, lightToIndex);
            L = pt.beta * pt.compute_f(sampled, TransportMode::Radiance) * sampled.beta;
            if (pt.isOnSurface())
                L *= absDot(wi, pt.ns());
            if (!L.isBlack())
                L *= visib.compute_Tr(scene, sampler);
        }
    }
    else
    {
        // Handle all other bidirectional connection cases
        const Vertex &qs = lightVertices[s - 1], &pt = cameraVertices[t - 1];
        if (qs.isConnectible() && pt.isConnectible()) {
            L = qs.beta * qs.compute_f(pt, TransportMode::Radiance) *
                pt.compute_f(qs, TransportMode::Importance) * pt.beta;
            if (!L.isBlack())
                L *= compute_G(scene, sampler, qs, pt);
        }
    }

    totalPaths++;
    if (L.isBlack()) zeroRadiancePaths++;
    REPORT_VALUE(pathLength, s + t - 2);

    // Compute MIS weight for connection strategy
    *misWeight = L.isBlack() ? 0 : MISweight(scene, lightVertices, cameraVertices, sampled, s, t, lightDistrib,
                                             lightToIndex);

    return L;
}

#define REMAP0(f) ((f) == 0 ? 1 : (f))

float BDPTIntegrator::MISweight(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices,
                                Vertex &sampled, int s, int t, const Distribution1D &lightDistrib,
                                const LightIndexMap &lightToIndex)
{
    if (s + t == 2) return 1;
    float sumRi = 0;

    // Temporarily update vertex properties for current strategy
    // Look up connection vertices and their predecessors
    Vertex *qs = s > 0 ? &lightVertices[s - 1] : nullptr,
            *pt = t > 0 ? &cameraVertices[t - 1] : nullptr,
            *qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr,
            *ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

    // Update sample vertex for s = 1 or t = 1 strategy
    ScopedAssignment<Vertex> a1;
    if (s == 1) a1 = { qs, sampled };
    else if (t == 1) a1 = { pt, sampled };

    // Mark connection vertices as non-generate
    ScopedAssignment<bool> a2, a3;
    if (pt) a2 = { &pt->delta, false };
    if (qs) a3 = { &qs->delta, false };

    // Update reverse density of p[t-1], p[t-2], q[s-1], q[s-2]
    ScopedAssignment<float> a4, a5, a6, a7;
    if (pt) a4 = { &pt->pdfRev, s > 0 ? qs->pdf(scene, qsMinus, *pt)
                                      : pt->pdfLightOrigin(scene, *ptMinus, lightDistrib, lightToIndex)};
    if (ptMinus) a5 = { &ptMinus->pdfRev, s > 0 ? pt->pdf(scene, qs, *ptMinus)
                                                : pt->pdfLight(scene, *ptMinus) };
    if (qs) a6 = { &qs->pdfRev, pt->pdf(scene, ptMinus, *qs) }; // no t = 0 strategy
    if (qsMinus) a7 = { &qsMinus->pdfRev, qs->pdf(scene, pt, *qsMinus) };

    // Consider hypothetical connection strategies on the camera subpath
    float ri = 1;
    for (int i = t - 1; i > 0; i--) {
        ri *= REMAP0(cameraVertices[i].pdfRev) / REMAP0(cameraVertices[i].pdfFwd);
        if (!cameraVertices[i].delta && !cameraVertices[i - 1].delta)
            sumRi += ri;
    }

    // And on the light subpath
    ri = 1;
    for (int i = s - 1; i >= 0; i--) {
        ri *= REMAP0(lightVertices[i].pdfRev) / REMAP0(lightVertices[i].pdfFwd);
        bool deltaLightVertex = i > 0 ? lightVertices[i - 1].delta : lightVertices[0].isDeltaLight();
        if (!lightVertices[i].delta && !deltaLightVertex)
            sumRi += ri;
    }

    return 1 / (1.0f + sumRi);
}

void BDPTIntegrator::render(const Scene &scene) {
    // Partition the image into tiles
    Film *film = camera->film;
    const Bounds2i sampleBounds = film->getSampleBounds();
    const Vector2i sampleExtent = sampleBounds.diagonal();
    constexpr int tileSize = 16;
    const int nXTiles = (sampleExtent.x + tileSize - 1) / tileSize;
    const int nYTiles = (sampleExtent.y + tileSize - 1) / tileSize;
    ProgressReporter reporter(nXTiles * nYTiles, "Rendering");

    // Render and write the output image to disk
    if (scene.lights.empty())
        WARNING("No lights in the scene. Rendering black image...");
    auto lightDistrib = computeLightPowerDistribution(scene.lights);
    LightIndexMap lightToIndex;
    for (size_t i = 0; i < scene.lights.size(); i++)
        lightToIndex[scene.lights[i].get()] = i;

    Parallel::forLoop2D([&] (const Point2i tile) {
        // Get film tile
        MemoryArena arena;
        int seed = tile.y * nXTiles + tile.x;
        unique_ptr<Sampler> tileSampler = sampler->clone(seed);
        int x0 = sampleBounds.pMin.x + tile.x * tileSize;
        int x1 = min(x0 + tileSize, sampleBounds.pMax.x);
        int y0 = sampleBounds.pMin.y + tile.y * tileSize;
        int y1 = min(y0 + tileSize, sampleBounds.pMax.y);
        Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
        auto filmTile = camera->film->getFilmTile(tileBounds);

        // Main render process
        for (Point2i pPixel : tileBounds) {
            tileSampler->startPixel(pPixel);
            if (!insideExclusive(pPixel, pixelBounds)) continue;
            do {
                Point2f pFilm = Point2f(pPixel) + tileSampler->get2D();

                // Trace camera and light subpath
                Vertex *cameraVertices = arena.alloc<Vertex>(maxDepth + 2);
                Vertex *lightVertices = arena.alloc<Vertex>(maxDepth + 1); // t = 0 is not useful
                int nCamera = generateCameraSubpath(scene, *tileSampler, arena, maxDepth + 2, *camera, pFilm,
                                                    cameraVertices);
                int nLight = generateLightSubpath(scene, *tileSampler, arena, maxDepth + 1,
                                                  cameraVertices[0].time(), *lightDistrib, lightToIndex,
                                                  lightVertices);

                // Execute all connection strategies
                Spectrum L;
                for (int t = 1; t <= nCamera; t++) {
                    for (int s = 0; s <= nLight; s++) {
                        int depth = t + s - 2;
                        if ((s == 1 && t == 1) || depth < 0 || depth > maxDepth) continue;
                        Point2f pFilmNew = pFilm;
                        float misWeight = 0;
                        Spectrum Lpath = connectVertices(scene, lightVertices, cameraVertices, s, t, *lightDistrib,
                                                         lightToIndex, *camera, *tileSampler, &pFilmNew, &misWeight);
                        if (t == 1) // connect directly to camera
                            film->addSplat(pFilmNew, Lpath * misWeight);
                        else
                            L += Lpath * misWeight;
                    }
                }
                filmTile->addSample(pFilm, L);
                arena.reset();

            } while (tileSampler->startNextSample());
        } // end tile

        film->mergeFilmTile(move(filmTile));
        reporter.update();

    }, Point2i(nXTiles, nYTiles));

    reporter.done();
    film->writeImage(1.0f / sampler->samplesPerPixel);
}

BDPTIntegrator * BDPTIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                        shared_ptr<const Camera> camera)
{
    int maxDepth = params.findOneInt("maxdepth", 5);
    int np;
    const int *pb = params.findInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->getSampleBounds();
    if (pb) {
        if (np != 4)
            ERROR("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = intersect(pixelBounds, Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.area() == 0)
                ERROR("Degenerate \"pixelbounds\" specified.");
        }
    }

    return new BDPTIntegrator(sampler, camera, maxDepth, pixelBounds);
}
