#ifndef INTEGRATOR_DIRECTLIGHTING
#define INTEGRATOR_DIRECTLIGHTING

#include "core/integrator.h"

enum class LightStrategy { UniformSampleAll, UniformSampleOne };

class DirectLightingIntegrator : public SamplerIntegrator {
public:
    DirectLightingIntegrator(LightStrategy strategy, int maxDepth, shared_ptr<const Camera> camera,
                             shared_ptr<Sampler> sampler, const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), strategy(strategy), maxDepth(maxDepth)
    {}

    static DirectLightingIntegrator * create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                             shared_ptr<const Camera> camera);

    void preprocess(const Scene &scene, Sampler &sampler);
    Spectrum compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                        MemoryArena &arena, int depth = 0) const;

private:
    const LightStrategy strategy;
    const int maxDepth;
    vector<int> nLightSamples;
};

#endif // INTEGRATOR_DIRECTLIGHTING
