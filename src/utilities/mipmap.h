#ifndef UTILITY_MIPMAP
#define UTILITY_MIPMAP

#include "vector.h"
#include "memory.h"

enum class ImageWrap { Repeat, Black, Clamp };

struct ResampleWeight {
    int firstTexel;
    float weight[4];
    float sumWeight() const { return weight[0] + weight[1] + weight[2] + weight[3]; }
};

template <class T>
class Mipmap {
public:
    Mipmap(const Point2i &res, const T *img, bool doTrilinear = false, float maxAnisotropy = 8.0f,
           ImageWrap wrapMode = ImageWrap::Repeat);
    T lookup(const Point2f &st, float width = 0.0f) const; // triangle filter
    T lookup(const Point2f &st, Vector2f dst0, Vector2f dst1) const; // EWA filter

    unsigned width() const { return resolution[0]; }
    unsigned height() const { return resolution[1]; }
    unsigned levels() const { return pyramid.size(); }

private:
    unique_ptr<ResampleWeight[]> resampleWeights(int oldRes, int newRes);
    const T & texel(unsigned level, int s, int t) const;
    T triangle(unsigned level, const Point2f &st) const;
    T EWA(unsigned level, Point2f st, Vector2f dst0, Vector2f dst1) const;

    inline static float lanczos(float x, float tau = 2) {
        x = abs(x);
        if (x < 1e-5f) return 1;
        if (x > 1.f) return 0;
        x *= PI;
        float s = sin(x * tau) / (x * tau);
        float lanczos = sin(x) / x;
        return s * lanczos;
    }

    static constexpr int LOG_BLOCK_SIZE = 2;
    static constexpr int WEIGHT_LUT_SIZE = 128;
    static float weightLut[WEIGHT_LUT_SIZE];

    const bool doTrilinear;
    const float maxAnisotropy;
    const ImageWrap wrapMode;
    Point2i resolution;
    vector<unique_ptr<BlockedArray<T, LOG_BLOCK_SIZE>>> pyramid;
};

extern template class Mipmap<float>;
extern template class Mipmap<RGBSpectrum>;

#endif // UTILITY_MIPMAP
