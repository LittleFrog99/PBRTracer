#ifndef UTILITY_RANDOM
#define UTILITY_RANDOM

#include "utilities.h"

class Random { // implements PCG pseudo-number generator
public:
#ifndef DOUBLE_AS_FLOAT
    static constexpr Float ONE_MINUS_EPSILON = 0x1.fffffep-1;
#else
    static constexpr Float ONE_MINUS_EPSILON = 0x1.fffffffffffffp-1;
#endif
    static constexpr uint64_t PCG32_DEFAULT_STATE = 0x853c49e6748fea9bULL;
    static constexpr uint64_t PCG32_DEFAULT_STREAM = 0xda3e39cb94b95bdbULL;
    static constexpr uint64_t PCG32_MULT = 0x5851f42d4c957f2dULL;

    Random() : state(PCG32_DEFAULT_STATE), inc(PCG32_DEFAULT_STREAM) {}
    Random(uint32_t sequence) { setSequence(sequence); }

    void setSequence(uint32_t index);
    uint32_t uniformUInt32();
    uint32_t uniformUInt32(uint32_t b);
    void advance(int64_t idelta);

    Float uniformFloat() {
        return min(ONE_MINUS_EPSILON, Float(uniformUInt32() * 0x1p-32f));
    }

    template <class Iterator>
    void shuffle(Iterator begin, Iterator end) {
        for (Iterator it = end - 1; it > begin; --it)
            iter_swap(it, begin + uniformUInt32(uint32_t(it - begin + 1)));
    }

private:
    uint64_t state, inc;
};

#endif // UTILITY_RANDOM
