#ifndef INTEGRATOR_SPPM
#define INTEGRATOR_SPPM

#include "core/integrator.h"

class SPPMIntegrator : public Integrator {
public:
    SPPMIntegrator(shared_ptr<const Camera> &camera, int nIterations, int photonsPerIteration, int maxDepth,
                   float initialSearchRadius, int writeFrequency)
        : camera(camera), initialSearchRadius(initialSearchRadius), nIterations(nIterations), maxDepth(maxDepth),
          photonsPerIteration(photonsPerIteration), writeFrequency(writeFrequency) {}

    static Integrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                               shared_ptr<const Camera> camera);

    void render(const Scene &scene);

private:
    struct SPPMPixel;
    struct SPPMPixelListNode;
    class VisiblePointGrid;

    shared_ptr<const Camera> camera;
    const float initialSearchRadius;
    const int nIterations;
    const int maxDepth;
    const int photonsPerIteration;
    const int writeFrequency;
};

#endif // INTEGRATOR_SPPM
