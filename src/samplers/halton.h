#ifndef SAMPLER_HALTON
#define SAMPLER_HALTON

#include "core/sampling.h"
#include "core/bounds.h"
#include "log.h"

class HaltonSampler : public GlobalSampler {
public:
    HaltonSampler(int samplePixels, const Bounds2i &sampleBounds);
    static HaltonSampler * create(const ParamSet &params, const Bounds2i &sampleBounds);

    int64_t getIndexForSample(int64_t sampleNum) const;
    float sampleDimension(int64_t index, int dim) const;

    unique_ptr<Sampler> clone(int) { return unique_ptr<Sampler>(new HaltonSampler(*this)); }

private:
    static vector<uint16_t> computeRadicalInversePermutations(Random &rng);
    static void extendedGCD(uint64_t a, uint64_t b, int64_t *x, int64_t *y);

    inline static uint16_t * permutationForDimension(int dim);
    inline static uint64_t multiplicativeInverse(int64_t a, int64_t n);
    template <int base>
    inline static uint64_t inverseRadicalInverse(uint64_t inverse, int nDigits);

    Point2i baseScales, baseExponents;
    int sampleStride;
    uint64_t multInv[2];
    mutable Point2i pixelForOffset = Point2i(numeric_limits<int>::max(), numeric_limits<int>::max());
    mutable int64_t offsetForCurrentPixel;

    static constexpr int PrimeTableSize = 1024;
    static constexpr int MaxResolution = 128;
    static int primeSums[PrimeTableSize]; // sums of preceeding primes
    static vector<uint16_t> radicalInvPerms;
};

inline uint16_t *HaltonSampler::permutationForDimension(int dim) {
    if (dim >= PrimeTableSize)
        SEVERE("HaltonSampler can only sample %d dimensions.", PrimeTableSize);
    return &radicalInvPerms[primeSums[dim]];
}

inline uint64_t HaltonSampler::multiplicativeInverse(int64_t a, int64_t n) {
    int64_t x, y;
    extendedGCD(a, n, &x, &y);
    return mod(x, n);
}

template<int base>
uint64_t HaltonSampler::inverseRadicalInverse(uint64_t inverse, int nDigits) {
    uint64_t index = 0;
    for (int i = 0; i < nDigits; ++i) {
        uint64_t digit = inverse % base;
        inverse /= base;
        index = index * base + digit;
    }
    return index;
}


#endif // SAMPLER_HALTON
