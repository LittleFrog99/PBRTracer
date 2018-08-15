#include "bdpt.h"
#include "stats.h"
#include "core/bsdf.h"
#include "core/primitive.h"
#include "paramset.h"

inline float BDPTIntegrator::correctShadingNormal(const SurfaceInteraction &isect, const Vector3f &wo,
                                                  const Vector3f &wi, TransportMode mode)
{
    if (mode == TransportMode::Importance) {
        float num = absDot(wo, isect.shading.n) * absDot(wi, isect.n);
        float denom = absDot(wo, isect.n) * absDot(wi, isect.shading.n);
        if (denom == 0) return 0;
        return num / denom;
    } else
        return 1;
}

float BDPTIntegrator::infiniteLightDensity(const vector<shared_ptr<Light>> lights,
                                                  const Distribution1D &distrib, const Vector3f &w)
{
    float pdf = 0;
    for (unsigned i = 0; i < lights.size(); i++)
        if (lights[i]->flags & int(LightFlags::Infinite))
            pdf += lights[i]->pdf_Li(Interaction(), -w) * distrib.func[i];
    return pdf / (distrib.funcInt * lights.size());
}


struct BDPTIntegrator::EndpointInteraction : Interaction {
    // Light endpoint
    EndpointInteraction() : Interaction(), light(nullptr) {}
    EndpointInteraction(const Light *light, const Ray &ray, const Normal3f &nl)
        : Interaction(ray.o, ray.time, ray.medium), light(light) { n = nl; }
    EndpointInteraction(const Interaction &it, const Light *light) : Interaction(it), light(light) {}
    EndpointInteraction(const Ray &ray) : Interaction(ray(1), ray.time, ray.medium), light(nullptr)
    { n = Normal3f(-ray.d); }

    // Camera endpoint
    EndpointInteraction(const Interaction &it, const Camera *camera) : Interaction(it), camera(camera) {}
    EndpointInteraction(const Camera *camera, const Ray &ray)
        : Interaction(ray.o, ray.time, ray.medium), camera(camera) {}

    union {
        const Camera *camera;
        const Light *light;
    };
};

struct BDPTIntegrator::Vertex {
    Vertex() : ei() {}
    Vertex(VertexType type, const EndpointInteraction &ei, const Spectrum &beta)
        : type(type), beta(beta), ei(ei) {}

    Vertex(const SurfaceInteraction &si, const Spectrum &beta)
        : type(VertexType::Surface), beta(beta), si(si) {}

    Vertex(const MediumInteraction &mi, const Spectrum &beta) : type(VertexType::Medium), beta(beta), mi(mi) {}

    Vertex(const Vertex &v) { memcpy(this, &v, sizeof(Vertex)); }

    Vertex & operator = (const Vertex &v) {
        memcpy(this, &v, sizeof(Vertex));
        return *this;
    }

    static Vertex createCamera(const Camera *camera, const Ray &ray, const Spectrum &beta) {
        return Vertex(VertexType::Camera, EndpointInteraction(camera, ray), beta);
    }

    static Vertex createCamera(const Camera *camera, const Interaction &it, const Spectrum &beta) {
        return Vertex(VertexType::Camera, EndpointInteraction(it, camera), beta);
    }

    static Vertex createLight(const Light *light, const Ray &ray, const Normal3f &nLight, const Spectrum &Le,
                              float pdf)
    {
        Vertex v(VertexType::Light, EndpointInteraction(light, ray, nLight), Le);
        v.pdfFwd = pdf;
        return v;
    }

    static Vertex createLight(const EndpointInteraction &ei, const Spectrum &beta, float pdf) {
        Vertex v(VertexType::Light, ei, beta);
        v.pdfFwd = pdf;
        return v;
    }

    static Vertex createMedium(const MediumInteraction &mi, const Spectrum &beta, float pdf, const Vertex &prev)
    {
        Vertex v(mi, beta);
        v.pdfFwd = prev.convertDensity(pdf, v);
        return v;
    }

    static Vertex createSurface(const SurfaceInteraction &si, const Spectrum &beta, float pdf, const Vertex &prev)
    {
        Vertex v(si, beta);
        v.pdfFwd = prev.convertDensity(pdf, v);
        return v;
    }

    const Interaction & getInteraction() const {
        switch (type) {
        case VertexType::Medium: return mi;
        case VertexType::Surface: return si;
        default: return ei;
        }
    }

    const Point3f & p() const { return getInteraction().p; }
    float time() const { return getInteraction().time; }
    const Normal3f & ng() const { return getInteraction().n; }
    const Normal3f & ns() const { return (type == VertexType::Surface) ? si.shading.n : getInteraction().n; }

    Spectrum compute_f(const Vertex &next) const {
        Vector3f wi = normalize(next.p() - p());
        switch (type) {
        case VertexType::Surface: return si.bsdf->compute_f(si.wo, wi);
        case VertexType::Medium: return mi.phase->compute_p(mi.wo, wi);
        default: return 0;
        }
    }

    bool isConnectible() const {
        switch (type) {
        case VertexType::Camera: return true;
        case VertexType::Medium: return true;
        case VertexType::Light: return (ei.light->flags & int(LightFlags::DeltaDirection)) == 0;
        case VertexType::Surface: return si.bsdf->numComponents( // has BSDF other than specular
                        BxDFType(BSDF_DIFFUSE | BSDF_GLOSSY |
                                 BSDF_REFLECTION | BSDF_TRANSMISSION)) > 0;
        }
    }

    bool isOnSurface() const { return ng() != Normal3f(); }

    bool isLight() const {
        return type == VertexType::Light || (type == VertexType::Surface && si.primitive->getAreaLight());
    }

    bool isDeltaLight() const {
        return type == VertexType::Light && ei.light && ei.light->isDeltaLight();
    }

    bool isInfiniteLight() const {
        return type == VertexType::Light && (!ei.light || ei.light->flags & int(LightFlags::Infinite) ||
                                             ei.light->flags & int(LightFlags::DeltaDirection));
    }

    Spectrum compute_Le(const Scene &scene, const Vertex &v) const {
        if (!isLight()) return 0;
        Vector3f w = normalize(v.p() - p());
        if (isInfiniteLight()) {
            Spectrum Le;
            for (const auto &light : scene.lights)
                Le += light->compute_Le(Ray(p(), w));
            return Le;
        }
        return 0;
    }

    float convertDensity(float pdf, const Vertex &next) const {
        if (next.isInfiniteLight()) return pdf; // return solid angle density
        Vector3f w = next.p() - p();
        if (w.lengthSq() == 0) return 0;
        float invDistSq = 1 / w.lengthSq();
        if (next.isOnSurface())
            pdf *= absDot(next.ng(), w * sqrt(invDistSq));
        return pdf * invDistSq;
    }

    float pdf(const Scene &scene, const Vertex *prev, const Vertex &next) const {
        if (type == VertexType::Light)
            return pdfLight(scene, next);
        // Compute directions to preceeding and next vertex
        Vector3f wp, wn = normalize(next.p() - p());
        if (prev) wp = normalize(prev->p() - p());

        // Compute directional density depending on the vertex type
        float pdf = 0, unused;
        if (type == VertexType::Camera)
            ei.camera->pdf_We(ei.spawnRay(wn), &unused, &pdf);
        else if (type == VertexType::Surface)
            pdf = si.bsdf->pdf(wp, wn);
        else if (type == VertexType::Medium)
            pdf = mi.phase->compute_p(wp, wn);

        return convertDensity(pdf, next);
    }

    float pdfLight(const Scene &scene, const Vertex &v) const {
        Vector3f w = v.p() - p();
        float invDistSq = 1 / w.lengthSq();
        w *= sqrt(invDistSq);
        float pdf = 0;
        if (isInfiniteLight()) {
            // Compute planar sampling density for infinite light
            Point3f worldCenter;
            float worldRadius;
            scene.getWorldBound().boundingSphere(&worldCenter, &worldRadius);
            pdf = 1 / (PI * SQ(worldRadius));
        } else {
            const auto light = type == VertexType::Light ? ei.light : si.primitive->getAreaLight();
            float pdfPos, pdfDir;
            light->pdf_Le(Ray(p(), w, time()), ng(), &pdfPos, &pdfDir);
            pdf = pdfDir * invDistSq;
        }
        if (v.isOnSurface())
            pdf *= absDot(v.ng(), w);
        return pdf;
    }

    float pdfLightOrigin(const Scene &scene, const Vertex &v, const Distribution1D &lightDistrib) const {
        Vector3f w = normalize(v.p() - p());
        if (isInfiniteLight()) {
            return infiniteLightDensity(scene.lights, lightDistrib, w); // solid angle density
        } else {
            float pdfPos = 0, pdfDir = 0, pdfChoice = 0;
            const auto light = type == VertexType::Light ? ei.light : si.primitive->getAreaLight();
            for (unsigned i = 0; i < scene.lights.size(); i++)
                if (scene.lights[i].get() == light) {
                    pdfChoice = lightDistrib.discretePDF(i);
                    break;
                }
            light->pdf_Le(Ray(p(), w, time()), ng(), &pdfPos, &pdfDir);
            return pdfPos * pdfChoice;
        }
        return 0;
    }

    VertexType type;
    Spectrum beta;
    union {
        EndpointInteraction ei;
        MediumInteraction mi;
        SurfaceInteraction si;
    };
    bool delta = false;
    float pdfFwd = 0, pdfRev = 0; // unit area

};

inline Spectrum BDPTIntegrator::compute_G(const Scene &scene, Sampler &sampler, const Vertex &v0, const Vertex &v1)
{
    Vector3f d = v0.p() - v1.p();
    float g = 1 / d.lengthSq();
    d *= sqrt(g);
    if (v0.isOnSurface()) g *= absDot(v0.ns(), d);
    if (v1.isOnSurface()) g *= absDot(v1.ns(), d);
    VisibilityTester visib(v0.getInteraction(), v1.getInteraction());
    return g * visib.compute_Tr(scene, sampler);
}

int BDPTIntegrator::generateCameraSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                          const Camera &camera, const Point2f &pFilm, Vertex *path)
{
    if (maxDepth == 0) return 0;

    // Sample initial ray for camera subpath
    CameraSample camSample;
    camSample.pFilm = pFilm;
    camSample.pLens = sampler.get2D();
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
                                         float time, const Distribution1D &lightDistrib, Vertex *path)
{
    if (maxDepth == 0) return 0;

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
    int nVertices = randomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth, TransportMode::Importance,
                               path + 1);

    // Correct subpath sampling densities for infinite area lights
    if (path[0].isInfiniteLight()) {
        path[1].pdfFwd = pdfPos;
        if (path[1].isOnSurface())
            path[1].pdfFwd *= absDot(ray.d, path[1].ns());
        path[0].pdfFwd = infiniteLightDensity(scene.lights, lightDistrib, ray.d);
    }

    return nVertices + 1;
}

int BDPTIntegrator::randomWalk(const Scene &scene, RayDifferential &ray, Sampler &sampler, MemoryArena &arena,
                               Spectrum beta, float pdf, int maxDepth, TransportMode mode, Vertex *path)
{
    if (maxDepth == 0) return 0;
    float pdfFwd = pdf, pdfRev; // unit solid angle
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

        if (mi.isValid())
        {
            // Record medium interaction in path and compute forward density
            vertex = Vertex::createMedium(mi, beta, pdfFwd, prev);
            if (++bounces >= maxDepth) break;

            // Sample direction and compute reverse density at preceding vertex
            Vector3f wi;
            pdfFwd = pdfRev = mi.phase->sample_p(-ray.d, &wi, sampler.get2D());
            ray = mi.spawnRay(wi);
        } else {
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
                                         int s, int t, const Distribution1D &lightDistrib, const Camera &camera,
                                         Sampler &sampler, Point2f *pRaster, float *misWeight)
{
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
        L = qs.beta * qs.compute_f(sampled) * visib.compute_Tr(scene, sampler) * sampled.beta;
        if (qs.isOnSurface())
            L *= absDot(wi, qs.ns());
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
            sampled.pdfFwd = sampled.pdfLightOrigin(scene, pt, lightDistrib);
            L = pt.beta * pt.compute_f(sampled) * sampled.beta;
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
            L = qs.beta * qs.compute_f(pt) * pt.compute_f(qs) * pt.beta;
            if (!L.isBlack())
                L *= compute_G(scene, sampler, qs, pt);
        }
    }

    // Compute MIS weight for connection strategy
    *misWeight = L.isBlack() ? 0 : MISweight(scene, lightVertices, cameraVertices, sampled, s, t, lightDistrib);
    L *= *misWeight;

    return L;
}

template <class T>
class BDPTIntegrator::ScopedAssignment {
public:
    ScopedAssignment(T *target = nullptr, T value = T()) : target(target) {
        if (target) {
            backup = *target;
            *target = value;
        }
    }

    ~ScopedAssignment() { if (target) *target = backup; }

private:
    T *target;
    T backup;
};

#define REMAP0(f) ((f) == 0 ? 1 : (f))

float BDPTIntegrator::MISweight(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices,
                                Vertex &sampled, int s, int t, const Distribution1D &lightDistrib)
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

    // Upate reverse density of p[t-1], p[t-2], q[s-1], q[s-2]
    ScopedAssignment<float> a4, a5, a6, a7;
    if (pt) a4 = { &pt->pdfRev, s > 0 ? qs->pdf(scene, qsMinus, *pt)
                                      : pt->pdfLightOrigin(scene, *ptMinus, lightDistrib)};
    if (ptMinus) a5 = { &ptMinus->pdfRev, s > 0 ? pt->pdf(scene, qs, *ptMinus)
                                                : pt->pdfLight(scene, *ptMinus)};
    if (qs) a6 = {&qs->pdfRev, pt->pdf(scene, ptMinus, *qs)}; // no t = 0 strategy
    if (qsMinus) a7 = {&qsMinus->pdfRev, qs->pdf(scene, pt, *qsMinus)};

    // Consider hypothetical connection strategies on the camera subpath
    float ri = 0;
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
                                                  cameraVertices[0].time(), *lightDistrib, lightVertices);

                // Execute all connection strategies
                Spectrum L;
                for (int t = 1; t <= nCamera; t++) {
                    for (int s = 0; s <= nLight; s++) {
                        int depth = t + s - 2;
                        if ((s == 1 && t == 1) || depth < 0 || depth > maxDepth) continue;
                        Point2f pFilmNew = pFilm;
                        float misWeight = 0;
                        Spectrum Lpath = connectVertices(scene, lightVertices, cameraVertices, s, t,
                                                         *lightDistrib, *camera, *sampler, &pFilmNew,
                                                         &misWeight);
                        if (t == 1) // connect directly to camera
                            film->addSplat(pFilmNew, Lpath);
                        else
                            L += Lpath;
                    }
                }
                filmTile->addSample(pFilm, L);

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
