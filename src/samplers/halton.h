#ifndef SAMPLER_HALTON
#define SAMPLER_HALTON

#include "core/sampling.h"
#include "core/bounds.h"
#include "random.h"
#include "log.h"

typedef unsigned int uint128_t __attribute__((mode(TI)));

class HaltonSampler : public GlobalSampler {
public:
    HaltonSampler(int samplePixels, const Bounds2i &sampleBounds);
    static HaltonSampler * create(const ParamSet &params, const Bounds2i &sampleBounds);

    int64_t getIndexForSample(int64_t sampleNum) const;
    float sampleDimension(int64_t index, int dim) const;

    unique_ptr<Sampler> clone(int) {
        return unique_ptr<Sampler>(new HaltonSampler(*this));
    }

private:
    static float radicalInverse(int baseIndex, uint64_t a);
    static uint64_t inverseRadicalInverse(uint64_t inverse, int nDigits, int baseIndex);
    static vector<uint16_t> computeRadicalInversePermutations(Random &rng);
    static float scrambledRadicalInverse(int baseIndex, uint64_t a, const uint16_t *perm);
    static void extendedGCD(uint64_t a, uint64_t b, int64_t *x, int64_t *y);

    inline static uint32_t reverseBits32(uint32_t n) {
        n = (n << 16) | (n >> 16);
        n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
        n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
        n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
        n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
        return n;
    }

    inline static uint64_t reverseBits64(uint64_t n) {
        uint64_t n0 = reverseBits32(uint32_t(n));
        uint64_t n1 = reverseBits32(uint32_t(n >> 32));
        return (n0 << 32) | n1;
    }

    inline static uint64_t integerDivide(uint64_t dividend, int baseIndex) {
        uint128_t interm = divMagicConsts[baseIndex];
        interm *= (dividend + 1);
        interm >>= divBitShifts[baseIndex];
        return uint64_t(interm);
    }

    inline static uint16_t * permutationForDimension(int dim) {
        if (dim >= PRIME_TABLE_SIZE)
            SEVERE("HaltonSampler can only sample %d dimensions.", PRIME_TABLE_SIZE);
        return &radicalInvPerms[primeSums[dim]];
    }

    inline static uint64_t multiplicativeInverse(int64_t a, int64_t n) {
        int64_t x, y;
        extendedGCD(a, n, &x, &y);
        return mod(x, n);
    }

    Point2i baseScales, baseExponents;
    int sampleStride;
    uint64_t multInv[2];
    mutable Point2i pixelForOffset = Point2i(numeric_limits<int>::max(), numeric_limits<int>::max());
    mutable int64_t offsetForCurrentPixel;

    static constexpr int PRIME_TABLE_SIZE = 1024;
    static constexpr int MAX_RESOLUTION = 128;
    static const int PRIMES[PRIME_TABLE_SIZE];
    static int primeSums[PRIME_TABLE_SIZE]; // sums of preceeding primes
    static float invPrimes[PRIME_TABLE_SIZE];
    static uint64_t divMagicConsts[PRIME_TABLE_SIZE];
    static int divBitShifts[PRIME_TABLE_SIZE];
    static vector<uint16_t> radicalInvPerms;
};

#endif // SAMPLER_HALTON
