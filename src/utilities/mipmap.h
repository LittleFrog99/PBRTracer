#ifndef UTILITY_MIPMAP
#define UTILITY_MIPMAP

#include "vector.h"
#include "memory.h"

enum class ImageWrap { Repeat, Black, Clamp };

template <class T>
class MIPMap {
public:
    MIPMap(const Point2i &res, const T *img, bool doTrilinear, float maxAnisotropy, ImageWrap wrapMode);

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
