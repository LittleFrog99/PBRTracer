#ifndef SAMPLER_STRATIFIED
#define SAMPLER_STRATIFIED

#include "core/sampling.h"

template <class T>
class Bounds2;

class StratifiedSampler : public PixelSampler {
public:
    StratifiedSampler(int xPixelSamples, int yPixelSamples, bool jitterSamples, int nSampledDimensions)
        : PixelSampler(xPixelSamples * yPixelSamples, nSampledDimensions),
          xPixelSamples(xPixelSamples), yPixelSamples(yPixelSamples), jitterSamples(jitterSamples) {}

    static StratifiedSampler * create(const ParamSet &params, const Bounds2<int> &sampleBounds);

    void startPixel(const Point2i &p);
    unique_ptr<Sampler> clone(int seed);

private:
    static void sample1D(float *samples, int nsamples, Random &rng, bool jitter = true);
    static void sample2D(Point2f *samples, int nx, int ny, Random &rng, bool jitter = true);
    static void latinHypercube(float *samples, int nSamples, int nDim, Random &rng);

    const int xPixelSamples, yPixelSamples;
    const bool jitterSamples;
};

#endif // SAMPLER_STRATIFIED
