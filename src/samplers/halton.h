#ifndef SAMPLER_HALTON
#define SAMPLER_HALTON

#include "core/sampling.h"
#include "core/bounds.h"
#include "random.h"

typedef unsigned int uint128_t __attribute__((mode(TI)));

class HaltonSampler : public GlobalSampler {
public:
    HaltonSampler(int samplePixels, const Bounds2i &sampleBounds);

private:
    static Float radicalInverse(int baseIndex, uint64_t a);

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
        uint128_t interm = DIV_MAGIC_CONSTS[baseIndex];
        interm *= (dividend + 1);
        interm >>= DIV_BITSHIFTS[baseIndex];
        return uint64_t(interm);
    }

    static constexpr int PRIME_TABLE_SIZE = 1024;
    static const int PRIMES[PRIME_TABLE_SIZE];
    static int PRIME_SUMS[PRIME_TABLE_SIZE]; // sums of preceeding primes
    static Float INV_PRIMES[PRIME_TABLE_SIZE];
    static uint64_t DIV_MAGIC_CONSTS[PRIME_TABLE_SIZE];
    static int DIV_BITSHIFTS[PRIME_TABLE_SIZE];
};

#endif // SAMPLER_HALTON
