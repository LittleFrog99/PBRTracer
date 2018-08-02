#ifndef INTGERATOR_PATH
#define INTGERATOR_PATH

#include "core/integrator.h"

class PathIntegrator : public SamplerIntegrator {
public:
    PathIntegrator(int maxDepth, shared_ptr<const Camera> camera, shared_ptr<Sampler> sampler,
                   const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}

    static PathIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                   shared_ptr<const Camera> camera);

    Spectrum compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                        MemoryArena &arena, int depth = 0) const;

private:
    const int maxDepth;
};

#endif // INTGERATOR_PATH
