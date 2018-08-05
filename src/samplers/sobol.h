#ifndef SAMPLER_SOBOL
#define SAMPLER_SOBOL

#include "core/sampling.h"
#include "core/bounds.h"

class SobolSampler : public GlobalSampler {
public:
    SobolSampler(int64_t samplesPerPixel, const Bounds2i &sampleBounds);
    static SobolSampler * create(const ParamSet &params, const Bounds2i &sampleBounds);

    unique_ptr<Sampler> clone(int seed);
    int64_t getIndexForSample(int64_t sampleNum) const;
    float sampleDimension(int64_t index, int dimension) const;

private:
    static uint64_t intervalToIndex(const uint32_t m, uint64_t frame,const Point2i &p);
    static float sampleFloat(int64_t a, int dimension, uint32_t scramble = 0);

    const Bounds2i &sampleBounds;
    int resolution, log2Res;

    static constexpr int NUM_DIMENSION = 1024;
    static constexpr int MATRIX_SIZE = 52;
    static const uint32_t MATRICES_32[NUM_DIMENSION * MATRIX_SIZE];
    static const uint64_t VdC_MATRICES[][MATRIX_SIZE];
    static const uint64_t VdC_MATRICES_INV[][MATRIX_SIZE];
};

#endif // SAMPLER_SOBOL
