#ifndef INTEGRATOR_BDPT
#define INTEGRATOR_BDPT

#include "core/integrator.h"
#include "core/bsdf.h"
#include "core/primitive.h"
#include <unordered_map>

class BDPTIntegrator : public Integrator {
public:
    BDPTIntegrator(shared_ptr<Sampler> &sampler, shared_ptr<const Camera> camera, int maxDepth,
                   const Bounds2i &pixelBounds)
        : sampler(sampler), camera(camera), maxDepth(maxDepth), pixelBounds(pixelBounds) {}

    static BDPTIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                   shared_ptr<const Camera> camera);

    void render(const Scene &scene);

    /* Shared types and methods for other bidirectional integrator */
    enum class VertexType { Camera, Light, Surface, Medium };
    struct Vertex;
    struct EndpointInteraction;
    template <class T> class ScopedAssignment;
    typedef unordered_map<const Light *, size_t> LightIndexMap;

    static int generateCameraSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                     const Camera &camera, const Point2f &pFilm, Vertex *path);
    static int generateLightSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                    float time, const Distribution1D &lightDistrib,
                                    const LightIndexMap &lightToIndex, Vertex *path);
    static int randomWalk(const Scene &scene, RayDifferential &ray, Sampler &sampler, MemoryArena &arena,
                          Spectrum beta, float pdf, int maxDepth, TransportMode mode, Vertex *path);
    static Spectrum connectVertices(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, int s,
                                    int t, const Distribution1D &lightDistrib, const LightIndexMap &lightToIndex,
                                    const Camera &camera, Sampler &sampler, Point2f *pRaster,
                                    float *misWeight = nullptr);
    static float MISweight(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, Vertex &sampled,
                           int s, int t, const Distribution1D &lightDistrib, const LightIndexMap &lightToIndex);

    static Spectrum compute_G(const Scene &scene, Sampler &sampler, const Vertex &v0, const Vertex &v1);
    static float correctShadingNormal(const SurfaceInteraction &isect, const Vector3f &wo, const Vector3f &wi,
                                      TransportMode mode);
    static float infiniteLightDensity(const vector<shared_ptr<Light>> lights, const Distribution1D &distrib,
                                      const LightIndexMap &lightToIndex,
                                      const Vector3f &w);

private:
    shared_ptr<Sampler> sampler;
    shared_ptr<const Camera> camera;
    const int maxDepth;
    const Bounds2i pixelBounds;
};

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

inline float BDPTIntegrator::infiniteLightDensity(const vector<shared_ptr<Light>> lights,
                                                  const Distribution1D &distrib,
                                                  const LightIndexMap &lightToIndex, const Vector3f &w)
{
    float pdf = 0;
    for (const auto &light : lights) {
        size_t index = lightToIndex.find(light.get())->second;
        pdf += light->pdf_Li(Interaction(), -w) * distrib.func[index];
    }
    return pdf / (distrib.funcInt * distrib.count());
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
    const Vector3f & wo() const { return getInteraction().wo; }
    float time() const { return getInteraction().time; }
    const Normal3f & ng() const { return getInteraction().n; }
    const Normal3f & ns() const { return (type == VertexType::Surface) ? si.shading.n : getInteraction().n; }

    Spectrum compute_f(const Vertex &next, TransportMode mode) const {
        Vector3f wi = normalize(next.p() - p());
        switch (type) {
        case VertexType::Surface:
            return si.bsdf->compute_f(si.wo, wi) * correctShadingNormal(si, si.wo, wi, mode);
        case VertexType::Medium:
            return mi.phase->compute_p(mi.wo, wi);
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

    float pdfLightOrigin(const Scene &scene, const Vertex &v, const Distribution1D &lightDistrib,
                         const LightIndexMap &lightToIndex) const
    {
        Vector3f w = normalize(v.p() - p());
        if (isInfiniteLight()) {
            return infiniteLightDensity(scene.lights, lightDistrib, lightToIndex, w); // solid angle density
        } else {
            float pdfPos = 0, pdfDir = 0, pdfChoice = 0;
            const auto light = type == VertexType::Light ? ei.light : si.primitive->getAreaLight();
            size_t index = lightToIndex.find(light)->second;
            pdfChoice = lightDistrib.discretePDF(index);
            light->pdf_Le(Ray(p(), w, INFINITY, time()), ng(), &pdfPos, &pdfDir);
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
    int pathIndex; // for VCM
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


#endif // INTEGRATOR_BDPT
