#include "vcm.h"
#include "stats.h"
#include "scoped.h"
#include "samplers/sobol.h"

STAT_RATIO("VCM/Light vertices per iteration", storedLightVertices, nIterations)

class VCMIntegrator::LightVerticesGrid {
public:
    struct VertexNode {
        Vertex *vertex;
        VertexNode *next;
    };

    LightVerticesGrid(Vertex **vertices, vector<int> &pathLengths, float searchRadius, int maxDepth,
                      const Bounds3f &worldBound, vector<MemoryArena> &arenas)
        : hashSize(pathLengths.size()), grid(pathLengths.size()), searchRadius(searchRadius),
          bounds(worldBound)
    {
        // Compute resolution of grid in each dimension
        Vector3f diag = bounds.diagonal();
        float maxDiag = maxComp(diag);
        int baseGridRes = int(maxDiag / searchRadius);
        for (int i = 0; i < 3; i++)
            resolution[i] = max(int(baseGridRes * diag[i] / maxDiag), 1);

         // Add light vertices to grid
        Parallel::forLoop([&] (int pixelIndex) {
            auto &arena = arenas[Parallel::getThreadIndex()];
            Vertex *path = vertices[pixelIndex];
            int pathLength = pathLengths[pixelIndex];

            for (int i = 1; i < pathLength; i++) {
                Vertex *vertex = path + i;
                if (vertex->beta.isBlack()) continue;
                if (vertex->isOnSurface() && vertex->si.bsdf) {
                    bool isDiffuse = vertex->si.bsdf->numComponents(BxDFType(BSDF_DIFFUSE | BSDF_REFLECTION
                                                                        | BSDF_TRANSMISSION)) > 0;
                    bool isGlossy = vertex->si.bsdf->numComponents(BxDFType(BSDF_GLOSSY | BSDF_REFLECTION
                                                                       | BSDF_TRANSMISSION)) > 0;
                    if (!isDiffuse && !(isGlossy && i == (maxDepth - 1) )) continue;
                }

                Point3i pGrid;
                toGridIndex(vertex->p(), &pGrid);
                unsigned h = hash(pGrid);
                auto node = arena.alloc<VertexNode>();
                node->vertex = vertex;
                node->next = grid[h];
                while (!grid[h].compare_exchange_weak(node->next, node));

                storedLightVertices++;
            }

        }, hashSize, 4096);
    }

    void query(const Vertex *pt, vector<Vertex *> &lightVertices) const {
        Point3i pMin, pMax;
        toGridIndex(pt->p() - Vector3f(searchRadius), &pMin);
        toGridIndex(pt->p() + Vector3f(searchRadius), &pMax);

        Spectrum L;
        for (int px = pMin.x; px <= pMax.x; px++)
            for (int py = pMin.y; py <= pMax.y; py++)
                for (int pz = pMin.z; pz <= pMax.z; pz++) {
                    Point3i index(px, py, pz);
                    for (auto *node = grid[hash(index)].load(memory_order_relaxed); node; node = node->next) {
                        auto *vertex = node->vertex;
                        if (distanceSq(pt->p(), vertex->p()) > SQ(searchRadius)) continue;
                        lightVertices.push_back(vertex);
                    }
                }
    }

    bool toGridIndex(const Point3f &p, Point3i *pi) const {
        bool inBounds = true;
        Vector3f pg = bounds.offset(p);
        for (int i = 0; i < 3; i++) {
            (*pi)[i] = int(resolution[i] * pg[i]);
            inBounds &= ((*pi)[i] >= 0 && (*pi)[i] < resolution[i]);
            (*pi)[i] = clamp((*pi)[i], 0, resolution[i] - 1);
        }
        return inBounds;
    }

    const atomic<VertexNode *> & operator [] (const Point3i &index) const { return grid[hash(index)]; }

private:
    inline unsigned hash(const Point3i &p) const {
        unsigned hash = (p.x * 73856093) ^ (p.y * 19349663) ^ (p.z * 83492791);
        return hash % unsigned(hashSize);
    }

    const int hashSize;
    vector<atomic<VertexNode *>> grid;
    float searchRadius;
    Bounds3f bounds;
    int resolution[3];
};

int VCMIntegrator::generateCameraSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena,
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
    return randomWalk(PathType::Camera, scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1, 
                      TransportMode::Radiance, path + 1) + 1;
}

int VCMIntegrator::generateLightSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena,
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
    int nVertices = randomWalk(PathType::Light, scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1, 
                               TransportMode::Importance, path + 1);

    // Correct subpath sampling densities for infinite area lights
    if (path[0].isInfiniteLight()) {
        if (nVertices > 0) {
            path[1].pdfFwd = pdfPos;
            if (path[1].isOnSurface())
                path[1].pdfFwd *= absDot(ray.d, path[1].ns());
        }
        path[0].pdfFwd = BDPTIntegrator::infiniteLightDensity(scene.lights, lightDistrib, lightToIndex, ray.d);
    }

    return nVertices + 1;
}

#define REMAP0(f) ((f) == 0 ? 1 : (f))

int VCMIntegrator::randomWalk(const PathType type, const Scene &scene, RayDifferential &ray, Sampler &sampler,
                              MemoryArena &arena, Spectrum beta, float pdf, int maxDepth, TransportMode mode,
                              Vertex *path)
{
    if (maxDepth == 0) return 0;
    float pdfFwd = pdf, pdfRev = 0; // w.r.t. solid angle
    int bounces = 0;
    while (true) {
        MediumInteraction mi;
        // Trace a ray and sample the medium, if any
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
            beta *= BDPTIntegrator::correctShadingNormal(isect, wo, wi, mode);
            ray = isect.spawnRay(wi);
        }

        // Compute quantities for MIS
        prev.pdfRev = vertex.convertDensity(pdfRev, prev); // reverse area density
        float gRev = vertex.solidAngleToArea(prev);
        if (bounces == 1) {
            if (type == PathType::Light) {
                vertex.dVCM = REMAP0(prev.pdfRev) / (REMAP0(prev.pdfFwd) * REMAP0(vertex.pdfFwd));
                vertex.dVC = gRev / (REMAP0(prev.pdfFwd) * REMAP0(vertex.pdfFwd));
                vertex.dVM = vertex.dVC / etaVCM;
            }
            else
                vertex.dVCM = REMAP0(prev.pdfRev) / (REMAP0(prev.pdfFwd) * REMAP0(vertex.pdfFwd));
        } else {
            float gRp = gRev / REMAP0(vertex.pdfFwd);
            float revPdf = REMAP0(prev.pdfSolidAngle(scene, &vertex, *(&prev - 1)));
            if (vertex.delta) {
                vertex.dVCM = 0;
                vertex.dVC = gRp * revPdf * prev.dVC;
                vertex.dVM = gRp * revPdf * prev.dVM;
            } else {
                vertex.dVCM = 1.0f / vertex.pdfFwd;
                vertex.dVC = gRp * (etaVCM + prev.dVCM + revPdf * prev.dVC);
                vertex.dVM = gRp * (1.0f + prev.dVCM / etaVCM + revPdf * prev.dVM);
            }
        }
    }

    return bounces;
}

Spectrum VCMIntegrator::connectVertices(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, 
                                        int s, int t, const Distribution1D &lightDistrib, 
                                        const LightIndexMap &lightToIndex, const Camera &camera, Sampler &sampler, Point2f *pRaster)
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
                L *= BDPTIntegrator::compute_G(scene, sampler, qs, pt);
        }
    }

    // Compute MIS weight for connection strategy
    float weight = L.isBlack() ? 0 : MISweight(scene, Technique::Connection, lightVertices[s - 1], 
                                               cameraVertices[t - 1], s, t, lightDistrib, lightToIndex, &sampled);

    return weight * L;
}

#define SPHERE_VOLUME(r) ((4.0f / 3.0f) * PI * CUB(r))

Spectrum VCMIntegrator::mergeVertices(const Scene &scene, const LightVerticesGrid &grid, Vertex **lightPaths,
                                      Vertex *cameraVertices, int t, const Distribution1D &lightDistrib,
                                      const LightIndexMap &lightToIndex)
{
    Vertex *pt = &cameraVertices[t - 1];
    if (pt->type == VertexType::Light) return 0;
    if (pt->isOnSurface()) {
        bool isDiffuse = pt->si.bsdf->numComponents(BxDFType(BSDF_DIFFUSE | BSDF_REFLECTION
                                                            | BSDF_TRANSMISSION)) > 0;
        bool isGlossy = pt->si.bsdf->numComponents(BxDFType(BSDF_GLOSSY | BSDF_REFLECTION
                                                           | BSDF_TRANSMISSION)) > 0;
        if (!isDiffuse && !(isGlossy && t == (maxDepth - 1) )) return 0;
    }

    // Precompute camera weight
    float camPdf = 1;
    for (int i = 1; i < t; i++)
        camPdf *= REMAP0(cameraVertices[i].pdfFwd);

    Spectrum L;
    vector<Vertex *> lightVertices;
    grid.query(pt, lightVertices);

    for (auto vertex : lightVertices) {
        Vertex *path = lightPaths[vertex->pathIndex];
        Spectrum Lpath;
        if (vertex->isLight()) continue; // ignore light vertex

        if (pt->type == VertexType::Surface) // Surface estimate
        {
            Spectrum phi = pt->beta * pt->si.bsdf->compute_f(pt->wo(), vertex->wo()) * vertex->beta;
            Lpath = phi / (nPixels * PI * SQ(searchRadius));
        }
        else if (pt->type == VertexType::Medium) // Volume estimate
        {
            Spectrum phi = pt->beta * pt->mi.phase->compute_p(pt->wo(), vertex->wo()) * vertex->beta;
            Spectrum sigma_s = vertex->mi.getMedium(Vector3f(vertex->ns()))->get_sigma_s(&vertex->mi);
            Lpath = phi / (nPixels * sigma_s * SPHERE_VOLUME(searchRadius));
        }
        else continue;

        int s = vertex - lightPaths[vertex->pathIndex] + 1;
        float weight = Lpath.isBlack() ? 0 : MISweight(scene, Technique::Merging, *vertex, *pt, s, t,
                                                       lightDistrib, lightToIndex, nullptr);
        L += weight * Lpath;
    }

    return L;
}

float VCMIntegrator::MISweight(const Scene &scene, const Technique v, Vertex &lightVertex, Vertex &cameraVertex,
                               int s, int t, const Distribution1D &lightDistrib,
                               const LightIndexMap &lightToIndex, const Vertex *sampled)
{
    if (s + t == 2) return 1;

    // Temporarily update vertex properties for current strategy
    // Look up connection vertices and their predecessors
    Vertex *qs, *qsMinus, *pt, *ptMinus;
    qs = s > 0 ? &lightVertex : nullptr;
    qsMinus = s > 1 ? (&lightVertex - 1) : nullptr;
    pt = t > 0 ? &cameraVertex : nullptr;
    ptMinus = t > 1 ? (&cameraVertex - 1) : nullptr;

    // Mark connection vertices as non-generate
    ScopedAssignment<bool> a2, a3;
    if (pt) a2 = { &pt->delta, false };
    if (qs) a3 = { &qs->delta, false };

    // Compute vertex merging weight
    if (v == Technique::Merging) {
        float wy = qs->dVCM / etaVCM + REMAP0(qs->pdfSolidAngle(scene, ptMinus, *qsMinus)) * qs->dVM;
        float wz = pt->dVCM / etaVCM + REMAP0(pt->pdfSolidAngle(scene, qsMinus, *ptMinus)) * pt->dVM;
        return 1.0f / (wy + 1.0f + wz);
    }

    // Update sample vertex for s = 1 or t = 1 strategy
    ScopedAssignment<Vertex> a1;
    if (s == 1) a1 = { qs, *sampled };
    else if (t == 1) a1 = { pt, *sampled };

    // Update reverse density of p[t-1], p[t-2], q[s-1], q[s-2]
    ScopedAssignment<float> a4, a5, a6, a7;
    if (pt) a4 = { &pt->pdfRev, s > 0 ? qs->pdf(scene, qsMinus, *pt)
                                      : pt->pdfLightOrigin(scene, *ptMinus, lightDistrib, lightToIndex)};
    if (ptMinus) a5 = { &ptMinus->pdfRev, s > 0 ? pt->pdf(scene, qs, *ptMinus)
                                                : pt->pdfLight(scene, *ptMinus) };
    if (qs) a6 = { &qs->pdfRev, pt->pdf(scene, ptMinus, *qs) }; // no t = 0 strategy
    if (qsMinus) a7 = { &qsMinus->pdfRev, qs->pdf(scene, pt, *qsMinus) };

    // Compute vertex connection weight
    if (s == 0) { // eye vertex directly sampled on light source
        float pdfRev = REMAP0(pt->pdfLightSolidAngle(scene, *ptMinus));
        float wz = REMAP0(pt->pdfRev) * pt->dVCM + REMAP0(pt->pdfFwd) * pdfRev * pt->dVC;
        return 1.0f / (1.0f + wz);
    } 
    else if (s == 1) {
        float wy = REMAP0(qs->pdfFwd) / REMAP0(qs->pdfRev);
        float wz = wy * REMAP0(pt->pdfRev) * (etaVCM + pt->dVCM + pt->pdfSolidAngle(scene, qs, *ptMinus) * pt->dVC);
        return 1.0f / (wy + 1.0f + wz);
    }
    else if (t == 1) {
        float wy = REMAP0(pt->pdfFwd) / REMAP0(pt->pdfRev) * REMAP0(qs->pdfRev) * 
                   (etaVCM + qs->dVCM + qs->pdfSolidAngle(scene, pt, *qsMinus) * qs->dVC);
        return 1.0f / (wy + 1.0f);
    }

    return 0;
}

#define REMAP0(f) ((f) == 0 ? 1 : (f))

void VCMIntegrator::render(const Scene &scene) {
    ProfilePhase __(Stage::IntegratorRender);
    Stats::nIterations = nIterations;

    // Initialize pixel bounds
    Film *film = camera->film;
    const Bounds2i sampleBounds = film->getSampleBounds();
    const Vector2i sampleExtent = sampleBounds.diagonal();
    nPixels = sampleBounds.area();
    constexpr int tileSize = 16;
    const int nXTiles = (sampleExtent.x + tileSize - 1) / tileSize;
    const int nYTiles = (sampleExtent.y + tileSize - 1) / tileSize;

    // Initialize sampler
    SobolSampler globalSampler(nIterations, sampleBounds);

    // Initialize light distribution
    if (scene.lights.empty())
        WARNING("No lights in the scene. Rendering black image...");
    auto lightDistrib = computeLightPowerDistribution(scene.lights);
    LightIndexMap lightToIndex;
    for (size_t i = 0; i < scene.lights.size(); i++)
        lightToIndex[scene.lights[i].get()] = i;

    // Main render loop
    ProgressReporter reporter(nIterations * 2, "Rendering ");
    for (int ite = 0; ite < nIterations; ite++) {
        vector<MemoryArena> lightArenas(Parallel::maxThreadIndex());

        // Sample light path
        unique_ptr<Vertex *[]> lightPaths(new Vertex *[nPixels]);
        vector<int> lightPathLengths(nPixels);
        constexpr int chunkSize = 256;
        int nChunk = nPixels / chunkSize + 1;

        Parallel::forLoop([&] (int chunkIndex) {
            auto &arena = lightArenas[Parallel::getThreadIndex()];
            auto sampler = globalSampler.clone(chunkIndex);
            for (int i = 0; i < chunkSize; i++) {
                int pathIndex = chunkIndex * chunkSize + i;
                if (pathIndex >= nPixels) return;
                sampler->startPixel(Point2i(chunkIndex, i));
                sampler->setSampleNumber(ite);
                lightPaths[pathIndex] = arena.alloc<Vertex>(maxDepth + 1);
                lightPathLengths[pathIndex] =
                        generateLightSubpath(scene, *sampler, arena, sampler->get1D(), *lightDistrib,
                                             lightToIndex, lightPaths[pathIndex]);
                for (int k = 0; k < lightPathLengths[pathIndex]; k++)
                    lightPaths[pathIndex][k].pathIndex = pathIndex; // append pathIndex to vertices for VM
            }
        }, nChunk);

        // Build range search struct
        searchRadius = initialSearchRadius * pow(float(ite + 1), 0.5f * (alpha - 1));
        etaVCM = nPixels * PI * SQ(searchRadius);
        LightVerticesGrid grid(lightPaths.get(), lightPathLengths, searchRadius, maxDepth, scene.getWorldBound(),
                               lightArenas);
        reporter.update();

        Parallel::forLoop2D([&] (const Point2i tile) {
            MemoryArena arena;
            int seed = tile.y * nXTiles + tile.x;
            auto tileSampler = globalSampler.clone(seed);

            // Get film tile
            int x0 = sampleBounds.pMin.x + tile.x * tileSize;
            int x1 = min(x0 + tileSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + tile.y * tileSize;
            int y1 = min(y0 + tileSize, sampleBounds.pMax.y);
            Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
            auto filmTile = camera->film->getFilmTile(tileBounds);

            for (Point2i pPixel : tileBounds) {
                tileSampler->startPixel(pPixel);
                tileSampler->setSampleNumber(ite);
                // Compute pixel offset
                Point2i pPixelO = Point2i(pPixel - sampleBounds.pMin);
                int pixelOffset = pPixelO.x + pPixelO.y * (sampleBounds.pMax.x - sampleBounds.pMin.x);

                // Sample camera path
                Point2f pFilm = Point2f(pPixel) + tileSampler->get2D();
                Vertex *cameraVertices = arena.alloc<Vertex>(maxDepth + 2);
                int nCamera = generateCameraSubpath(scene, *tileSampler, arena, *camera, pFilm, cameraVertices);
                Vertex *lightVertices = lightPaths[pixelOffset];
                int nLight = lightPathLengths[pixelOffset];

                Spectrum L;
                for (int t = 1; t <= nCamera; t++) {
                    // Vertex connection
                    for (int s = 0; s <= nLight; s++) {
                        int depth = t + s - 2;
                        if ((s == 1 && t == 1) || depth < 0 || depth > maxDepth) continue;

                        // Vertex connection
                        Point2f pFilmNew = pFilm;
                        Spectrum Lvc =
                                connectVertices(scene, lightVertices, cameraVertices, s, t, *lightDistrib,
                                                lightToIndex, *camera, *tileSampler, &pFilmNew);
                        if (t == 1) // connect directly to camera
                            film->addSplat(pFilmNew, Lvc);
                        else
                            L += Lvc;
                    }

                    // Vertex merging (for t >= 2)
                    if (t >= 2) {
                        Spectrum Lvm = mergeVertices(scene, grid, lightPaths.get(), cameraVertices, t,
                                                     *lightDistrib, lightToIndex);
                        L += Lvm;
                    }
                }

                filmTile->addSample(pFilm, L);
            } // end each pixel in tile bounds

            film->mergeFilmTile(move(filmTile));
        }, Point2i(nXTiles, nYTiles));

        reporter.update();
    } // end iteration

    film->writeImage(1.0f / nIterations);
    reporter.done();
}

VCMIntegrator *VCMIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                     shared_ptr<const Camera> camera)
{
    int maxDepth = params.findOneInt("maxdepth", 7);
    int nIterations = params.findOneInt("iterations", params.findOneInt("numiterations", 10));
    float radius = params.findOneFloat("radius", 0.01f);
    float alpha = params.findOneFloat("alpha", 0.75f);
    return new VCMIntegrator(camera, nIterations, maxDepth, radius, alpha);
}
