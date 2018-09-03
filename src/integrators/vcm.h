#ifndef INTEGRATOR_VCM
#define INTEGRATOR_VCM

#include "bdpt.h"

class VCMIntegrator : public Integrator {
public:
    VCMIntegrator(shared_ptr<const Camera> &camera, int nIterations, int maxDepth, float initialSearchRadius,
                  float alpha)
        : camera(camera), nIterations(nIterations), maxDepth(maxDepth), initialSearchRadius(initialSearchRadius),
          alpha(alpha) {}

    static VCMIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                  shared_ptr<const Camera> camera);

    void render(const Scene &scene);

private:
    enum class Technique { Connection, Merging };
    enum class PathType { Camera, Light };
    class LightVerticesGrid;
    struct Vertex;
    using VertexType = BDPTIntegrator::VertexType;
    using LightIndexMap = BDPTIntegrator::LightIndexMap;

    int generateCameraSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena,
                              const Camera &camera, const Point2f &pFilm, Vertex *path);
    int generateLightSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena,
                             float time, const Distribution1D &lightDistrib,
                             const LightIndexMap &lightToIndex, Vertex *path);
    int randomWalk(const PathType type, const Scene &scene, RayDifferential &ray, Sampler &sampler, MemoryArena &arena,
                   Spectrum beta, float pdf, int maxDepth, TransportMode mode, Vertex *path);
    Spectrum connectVertices(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, int s,
                             int t, const Distribution1D &lightDistrib, const LightIndexMap &lightToIndex,
                             const Camera &camera, Sampler &sampler, Point2f *pRaster);
    Spectrum mergeVertices(const Scene &scene, const LightVerticesGrid &grid, Vertex **lightPaths,
                           Vertex *cameraVertices, int t, const Distribution1D &lightDistrib,
                           const LightIndexMap &lightToIndex);
    float MISweight(const Scene &scene, const Technique v, Vertex &lightVertex, Vertex &cameraVertex, int s, int t,
                    const Distribution1D &lightDistrib, const LightIndexMap &lightToIndex, const Vertex *sampled);

    shared_ptr<const Camera> camera;
    const int nIterations;
    const int maxDepth;
    const float initialSearchRadius;
    const float alpha; // variance-bias trade-off parameter
    int nPixels;
    float searchRadius;
    float etaVCM;
};

using EndpointInteraction = BDPTIntegrator::EndpointInteraction;

struct VCMIntegrator::Vertex : public BDPTIntegrator::Vertex {
    Vertex() : BDPTIntegrator::Vertex() {}
    Vertex(BDPTIntegrator::Vertex &vertex) : BDPTIntegrator::Vertex(vertex) {}
    
    Vertex & operator = (const BDPTIntegrator::Vertex &vertex) {
        memcpy(this, &vertex, sizeof(BDPTIntegrator::Vertex));
        dVC = dVM = dVCM = 0;
        return *this;
    }

    Vertex & operator = (const Vertex &vertex) {
        memcpy(this, &vertex, sizeof(Vertex));
        return *this;
    }

    float pdfSolidAngle(const Scene &scene, const Vertex *prev, const Vertex &next) const {
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

        return pdf;
    }

    float pdfArea(const Scene &scene, const Vertex *prev, const Vertex &next) const {
        return convertDensity(pdfSolidAngle(scene, prev, next), next);
    }

    float solidAngleToArea(const Vertex &next) const {
        float g = 1;
        if (next.isInfiniteLight()) return g; // return solid angle density
        Vector3f w = next.p() - p();
        if (w.lengthSq() == 0) return 0;
        float invDistSq = 1 / w.lengthSq();
        if (next.isOnSurface())
            g *= absDot(next.ng(), w * sqrt(invDistSq));
        return g * invDistSq;
    }

    float pdfLightSolidAngle(const Scene &scene, const Vertex &v) const {
        Vector3f w = normalize(v.p() - p());
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
            pdf = pdfDir;
        }
        return pdf;
    }

    int pathIndex; 
    float dVC = 0, dVM = 0, dVCM = 0;
};

#endif // INTEGRATOR_VCM
