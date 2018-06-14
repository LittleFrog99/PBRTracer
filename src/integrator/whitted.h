#ifndef WHITTED_H
#define WHITTED_H

#include "core/integrator.h"

class Whitted : public SamplerIntegrator {
public:
    Whitted(int maxDepth, shared_ptr<const Camera> camera, shared_ptr<Sampler> sampler,
            const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}

    RGBSpectrum compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                           MemoryArena &arena, int depth = 0) const;

private:
    const int maxDepth;
};

#endif // WHITTED_H
