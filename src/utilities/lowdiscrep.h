#ifndef UTILITY_LOW_DISCREPANCY
#define UTILITY_LOW_DISCREPANCY

#include "random.h"

namespace LowDiscrepancy {

static constexpr int PrimeTableSize = 1024;
extern int Primes[PrimeTableSize];

float radicalInverse(int baseIndex, uint64_t a);
float scrambledRadicalInverse(int baseIndex, uint64_t a, const uint16_t *perm);

inline uint32_t reverseBits32(uint32_t n) {
    n = (n << 16) | (n >> 16);
    n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
    n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
    n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
    n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
    return n;
}

inline uint64_t reverseBits64(uint64_t n) {
    uint64_t n0 = reverseBits32(uint32_t(n));
    uint64_t n1 = reverseBits32(uint32_t(n >> 32));
    return (n0 << 32) | n1;
}


}

#endif // UTILITY_LOW_DISCREPANCY
