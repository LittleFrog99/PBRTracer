#ifndef INTEGRATOR_BDPT
#define INTEGRATOR_BDPT

#include "core/integrator.h"

class BDPTIntegrator : public Integrator {
public:
    BDPTIntegrator(shared_ptr<Sampler> &sampler, shared_ptr<const Camera> camera, int maxDepth,
                   const Bounds2i &pixelBounds)
        : sampler(sampler), camera(camera), maxDepth(maxDepth), pixelBounds(pixelBounds) {}

    static BDPTIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                   shared_ptr<const Camera> camera);

    void render(const Scene &scene);

protected:
    enum class VertexType { Camera, Light, Surface, Medium };
    struct Vertex;
    struct EndpointInteraction;
    template <class T> class ScopedAssignment;

    static int generateCameraSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                     const Camera &camera, const Point2f &pFilm, Vertex *path);
    static int generateLightSubpath(const Scene &scene, Sampler &sampler, MemoryArena &arena, int maxDepth,
                                    float time, const Distribution1D &lightDistrib, Vertex *path);
    static int randomWalk(const Scene &scene, RayDifferential &ray, Sampler &sampler, MemoryArena &arena,
                          Spectrum beta, float pdf, int maxDepth, TransportMode mode, Vertex *path);
    static Spectrum connectVertices(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, int s,
                                    int t, const Distribution1D &lightDistrib, const Camera &camera,
                                    Sampler &sampler, Point2f *pRaster, float *misWeight = nullptr);
    static float MISweight(const Scene &scene, Vertex *lightVertices, Vertex *cameraVertices, Vertex &sampled,
                           int s, int t, const Distribution1D &lightDistrib);

    static Spectrum compute_G(const Scene &scene, Sampler &sampler, const Vertex &v0, const Vertex &v1);
    static float correctShadingNormal(const SurfaceInteraction &isect, const Vector3f &wo, const Vector3f &wi,
                                      TransportMode mode);
    static float infiniteLightDensity(const vector<shared_ptr<Light>> lights, const Distribution1D &distrib,
                                      const Vector3f &w);

    shared_ptr<Sampler> sampler;
    shared_ptr<const Camera> camera;
    const int maxDepth;
    const Bounds2i pixelBounds;
};

#endif // INTEGRATOR_BDPT
