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
    class LightVerticesGrid;
    template <class T> class ScopedAssignment;
    using Vertex = BDPTIntegrator::Vertex;
    using VertexType = BDPTIntegrator::VertexType;

    Spectrum mergeVertices(const LightVerticesGrid &grid, Vertex **lightPaths, Vertex *cameraVertices, int t,
                           int nLightPaths, float searchRadius, float *weight) const;

    shared_ptr<const Camera> camera;
    const int nIterations;
    const int maxDepth;
    const float initialSearchRadius;
    float alpha; // variance-bias trade-off parameter
};

#endif // INTEGRATOR_VCM
