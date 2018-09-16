#ifndef INTEGRATOR_MLT
#define INTEGRATOR_MLT

#include "integrators/bdpt.h"
#include "samplers/pss.h"

class MLTIntegrator : public Integrator {
public:
    MLTIntegrator(shared_ptr<const Camera> camera, uint maxDepth, uint nBootstrap, uint nChains, uint mutPerPixel,
                  float sigma, float largeStepProb)
        : camera(camera), maxDepth(maxDepth), nBootstrap(nBootstrap), nChains(nChains), mutPerPixel(mutPerPixel),
          sigma(sigma), largeStepProb(largeStepProb) {}

    static MLTIntegrator * create(const ParamSet &params, shared_ptr<Sampler>, shared_ptr<const Camera> camera);

    void render(const Scene &scene);

private:
    enum SampleStream {
        CAMERA_STREAM_INDEX,
        LIGHT_STREAM_INDEX,
        CONNECTION_STREAM_INDEX,
        NUM_SAMPLE_STREAMS
    };

    Spectrum compute_L(const Scene &scene, MemoryArena &arena, const unique_ptr<Distribution1D> &lightDistrib,
                       const LightIndexMap &lightToIndex, PSSSampler &sampler, int depth, Point2f *pRaster);

    shared_ptr<const Camera> camera;
    const uint maxDepth, nBootstrap, nChains, mutPerPixel;
    const float sigma, largeStepProb;
};

#endif // INTEGRATOR_MLT
