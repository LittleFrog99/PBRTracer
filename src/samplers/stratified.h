#ifndef SAMPLER_STRATIFIED
#define SAMPLER_STRATIFIED

#include "core/sampling.h"

class StratifiedSampler : public PixelSampler {
public:
    StratifiedSampler(int xPixelSamples, int yPixelSamples, bool jitterSamples, int nSampledDimensions)
        : PixelSampler(xPixelSamples * yPixelSamples, nSampledDimensions),
          xPixelSamples(xPixelSamples), yPixelSamples(yPixelSamples), jitterSamples(jitterSamples) {}

    static StratifiedSampler * create(const ParamSet &params);

    void startPixel(const Point2i &p);
    unique_ptr<Sampler> clone(int seed);

private:
    static void sample1D(Float *samples, int nsamples, Random &rng, bool jitter = true);
    static void sample2D(Point2f *samples, int nx, int ny, Random &rng, bool jitter = true);
    static void latinHypercube(Float *samples, int nSamples, int nDim, Random &rng);

    const int xPixelSamples, yPixelSamples;
    const bool jitterSamples;
};

#endif // SAMPLER_STRATIFIED
