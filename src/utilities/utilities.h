#ifndef COMMON_UTILITIES
#define COMMON_UTILITIES

/* Common Included Headers */
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

/* Macros */
#ifndef FLOAT_AS_DOUBLE
typedef float Float;
#else
typedef double Float;
#endif

#define SQ(x) (x) * (x)

/* Global Constants */
static const Float PI = 3.14159265358979323846;
static const Float INV_PI = 0.31830988618379067154;
static const Float INV_TWO_PI = 0.15915494309189533577;
static const Float INV_FOUR_PI = 0.07957747154594766788;
static const Float PI_OVER_TWO = 1.57079632679489661923;
static const Float PI_OVER_FOUR = 0.78539816339744830961;
static const Float SQRT_TWO = 1.41421356237309504880;

/* Utility Funtions and Classes */
using namespace std;

namespace Math {
    template <class T, class U, class V>
    inline T clamp(T value, U low, V high) {
        if (value < low) return low;
        else if (value > high) return high;
        else return value;
    }

    template <class T>
    inline T mod(T a, T b) {
        T result = a - (a/b) * b;
        return T(result < 0 ? (result + b) : result);
    }

    inline Float mod(Float a, Float b) {
        return fmod(a, b);
    }

    inline Float radians(Float degrees) {
        return (PI / 180) * degrees;
    }

    inline Float degrees(Float radians) {
        return (180 / PI) * radians;
    }

    inline Float log2(Float x) {
        static const Float invLog2 = 1.442695040888963387004650940071;
        return std::log(x) * invLog2;
    }

    inline int log2Int(uint32_t v) {
        return 31 - __builtin_clz(v);
    }

    template <typename T>
    inline bool isPowerOf2(T v) {
        return v && !(v & (v - 1));
    }

    inline int32_t roundUpPow2(int32_t v) {
        v--;
        v |= v >> 1; v |= v >> 2;
        v |= v >> 4; v |= v >> 8;
        v |= v >> 16;
        return v + 1;
    }

    inline int64_t roundUpPow2(int64_t v) {
        v--;
        v |= v >> 1; v |= v >> 2;
        v |= v >> 4; v |= v >> 8;
        v |= v >> 16; v |= v >> 32;
        return v + 1;
    }

    inline int countTrailingZeros(uint32_t v) {
        return __builtin_ctz(v);
    }

    inline int findInterval(int size, function<bool(int)> compare) {
        int first = 0, length = size;
        while (length > 0) {
            int half = length >> 1, middle = first + half;
            if (compare(middle)) {
                first = middle + 1;
                length -= half + 1;
            } else
                length = half;
        }
        return clamp(first - 1, 0, size - 2);
    }

    inline Float lerp(Float t, Float v1, Float v2) {
        return (1 - t) * v1 + t * v2;
    }

    bool solveQuadratic(Float a, Float b, Float c, Float *t0, Float *t1);

    bool solveLinear2x2(const Float A[2][2], const Float B[2], Float *x0, Float *x1);
}

using namespace Math;

#endif // COMMON_UTILITIES
