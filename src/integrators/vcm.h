#ifndef INTEGRATOR_VCM
#define INTEGRATOR_VCM

#include "integrators/bdpt.h"

class VCMIntegrator : public Integrator {
public:
    enum class AlgorithmType { LightTrace, PPM, BPM, BDPT, VCM };
    enum class VertexType { Surface, Medium };

    VCMIntegrator(shared_ptr<const Camera> &camera, AlgorithmType algorithm, int nIterations, int maxDepth,
                  float initialRadius, float alpha);

    static VCMIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                  shared_ptr<const Camera> camera);

    void render(const Scene &scene);

    int getMaxDepth() const { return maxDepth; }
    int getLightSubpathNumber() const { return nLightSubpaths; }
    float getVCWeightFactor() const { return wVC; }
    float getVMWeightFactor() const { return wVM; }
    bool doPPM() const { return usePPM; }

private:
    struct Vertex;
    struct SubpathState;
    class RangeQuery;
    class LightVerticesGrid;
    enum class Technique { Connection, Merging };

    void lightPDFs(const Light *light, float pdfChoice, float pdfPos, float pdfDir, float *pdfTrace,
                   float *pdfConnect) const;
    SubpathState generateLightSample(const Scene &scene, Sampler &sampler) const;
    int lightRandomWalk(const Scene &scene, Sampler &sampler, MemoryArena &arena, SubpathState &lightState,
                        Vertex *path, Film *film) const;
    void connectToCamera(const Scene &scene, Sampler &sampler, const Vertex &lightVertex, Film *film) const;
    bool sampleScattering(Sampler &sampler, const Vertex &vertex, SubpathState &state, TransportMode mode) const;
    SubpathState generateCameraSample(Sampler &sampler, const Point2f &pFilm) const;
    Spectrum getLightRadiance(const Light *light, SubpathState &camState, SurfaceInteraction *isect = nullptr) const;
    Spectrum directIllumination(const Scene &scene, Sampler &sampler, const Vertex &camVertex) const;
    Spectrum connectVertices(const Scene &scene, Sampler &sampler, const Vertex &lightVertex,
                             const Vertex &camVertex) const;

    shared_ptr<const Camera> camera;
    unique_ptr<Distribution1D> lightDistrib;
    LightIndexMap lightToIndex;

    const int nIterations, maxDepth;
    bool useVM = false, useVC = false, lightTraceOnly = false, usePPM = false;
    const float initialRadius, alpha;
    float wVC, wVM;
    int nPixels, nLightSubpaths;
};

struct VCMIntegrator::SubpathState {
    RayDifferential ray;
    Spectrum beta = 0;
    int pathLength = 1; // number of path segments, including this
    bool isInfiniteLight = true;
    BxDFType flags = BSDF_NONE;
    bool isSpecularPath = true; // all scattering events so far were specular
    float dVCM = 0, dVC = 0, dVM = 0;

    bool isDelta() const { return flags & BSDF_SPECULAR; }
};

struct VCMIntegrator::Vertex {
    VertexType type;
    union {
        MediumInteraction mi;
        SurfaceInteraction si;
    };
    Spectrum beta; // path throughput
    int pathLength = 0; // number of segments between source and vertex
    float dVC = 0, dVM = 0, dVCM = 0; // MIS quantities

    Vertex() {}
    Vertex(const Vertex &v) { memcpy(this, &v, sizeof(Vertex)); }

    Vertex(const SubpathState &state, SurfaceInteraction &si)
        : type(VertexType::Surface), si(si), beta(state.beta), pathLength(state.pathLength),
          dVC(state.dVC), dVM(state.dVM), dVCM(state.dVCM) {}

    Vertex(const SubpathState &state, MediumInteraction &mi)
        : type(VertexType::Medium), mi(mi), beta(state.beta), pathLength(state.pathLength),
          dVC(state.dVC), dVM(state.dVM), dVCM(state.dVCM) {}

    Vertex & operator = (const Vertex &v) {
        memcpy(this, &v, sizeof(Vertex));
        return *this;
    }

    const Interaction & getInteraction() const {
        switch (type) {
        case VertexType::Medium: return mi;
        case VertexType::Surface: return si;
        }
    }

    const Point3f & p() const { return getInteraction().p; }
    const Vector3f & wo() const { return getInteraction().wo; }
    float time() const { return getInteraction().time; }
    const Normal3f & ng() const { return getInteraction().n; }
    const Normal3f & ns() const { return (type == VertexType::Surface) ? si.shading.n : getInteraction().n; }

    bool isOnSurface() const { return type == VertexType::Surface; }

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi, TransportMode mode) const {
        if (type == VertexType::Surface)
            return si.bsdf->compute_f(wo, wi) * BDPTIntegrator::correctShadingNormal(si, wo, wi, mode);
        else
            return mi.phase->compute_p(wo, wi);
    }

    Spectrum compute_f(const Vector3f &wi, TransportMode mode) const {
        return compute_f(wo(), wi, mode);
    }

    Spectrum compute_f(const Vertex &next, TransportMode mode) const {
        Vector3f wi = normalize(next.p() - p());
        return compute_f(wi, mode);
    }

    float pdfW(const Vector3f &wo, const Vector3f &wi) const {
        float pdf = 0;
        if (type == VertexType::Surface)
            pdf = si.bsdf->pdf(wo, wi);
        else
            pdf = mi.phase->compute_p(wo, wi);

        return pdf;
    }

    float pdfW(const Vector3f &wi) const {
        return pdfW(wo(), wi);
    }

    float pdfW(const Vertex *prev, const Vertex &next) const {
        // Compute directions to preceeding and next vertex
        Vector3f wp, wn = normalize(next.p() - p());
        if (prev) wp = normalize(prev->p() - p());
        return pdfW(wp, wn);
    }

};


#endif // INTEGRATOR_VCM
