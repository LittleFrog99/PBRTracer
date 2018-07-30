#ifndef INTEGRATOR_WHITTED
#define INTEGRATOR_WHITTED

#include "core/integrator.h"

class WhittedIntegrator : public SamplerIntegrator {
public:
    WhittedIntegrator(int maxDepth, shared_ptr<const Camera> camera, shared_ptr<Sampler> sampler,
            const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}

    static WhittedIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                      shared_ptr<const Camera> camera);

    Spectrum compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                           MemoryArena &arena, int depth = 0) const;

private:
    const int maxDepth;
};

#endif // INTEGRATOR_WHITTED
