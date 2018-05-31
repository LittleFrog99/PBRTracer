#include "random.h"

void Random::setSequence(uint32_t index) {
    state = 0u;
    inc = (index << 1u) | 1u;
    uniformUInt32();
    state += PCG32_DEFAULT_STATE;
    uniformUInt32();
}

uint32_t Random::uniformUInt32() {
    uint64_t oldstate = state;
    state = oldstate * PCG32_MULT + inc;
    uint32_t xorshifted = uint32_t(((oldstate >> 18u) ^ oldstate) >> 27u);
    uint32_t rot = uint32_t(oldstate >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
}

uint32_t Random::uniformUInt32(uint32_t b) {
    uint32_t threshold = (~b + 1u) % b;
    while (true) {
        uint32_t r = uniformUInt32();
        if (r >= threshold) return r % b;
    }
}

void Random::advance(int64_t idelta) {
    uint64_t cur_mult = PCG32_MULT, cur_plus = inc, acc_mult = 1u,
             acc_plus = 0u, delta = uint64_t(idelta);
    while (delta > 0) {
        if (delta & 1) {
            acc_mult *= cur_mult;
            acc_plus = acc_plus * cur_mult + cur_plus;
        }
        cur_plus = (cur_mult + 1) * cur_plus;
        cur_mult *= cur_mult;
        delta /= 2;
    }
    state = acc_mult * state + acc_plus;
}
