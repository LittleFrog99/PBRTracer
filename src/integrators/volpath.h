#ifndef INTEGRATOR_VOLPATH
#define INTEGRATOR_VOLPATH

#include "core/integrator.h"

class VolPathIntegrator : public SamplerIntegrator {
public:
    VolPathIntegrator(int maxDepth, shared_ptr<const Camera> camera, shared_ptr<Sampler> sampler,
                      const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}

    static VolPathIntegrator *create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                      shared_ptr<const Camera> camera);

    Spectrum compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                        MemoryArena &arena, int depth = 0) const;

private:
    int maxDepth;
};

#endif // VOLPATH_H
