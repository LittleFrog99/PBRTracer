#ifndef UTILITY_MIPMAP
#define UTILITY_MIPMAP

#include "vector.h"
#include "memory.h"

enum class ImageWrap { Repeat, Black, Clamp };

template <class T>
class Mipmap {
public:
    Mipmap(const Point2i &res, const T *img, bool doTrilinear = false, float maxAnisotropy = 8.0f,
           ImageWrap wrapMode = ImageWrap::Repeat);
    T lookup(const Point2f &st, Vector2f dst0, Vector2f dst1) const;

private:
    struct ResampleWeight {
        int firstTexel;
        float weight[4];
    };

    const bool doTrilinear;
    const float maxAnisotropy;
    const ImageWrap wrapMode;
    Point2i resolution;
};

#endif // UTILITY_MIPMAP
