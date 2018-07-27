#ifndef UTILITY_IMAGEIO
#define UTILITY_IMAGEIO

#include "core/spectrum.h"
#include "core/bounds.h"

namespace ImageIO {

inline float gammaCorrect(float value) {
    if (value <= 0.0031308f)
        return 12.92f * value;
    return 1.055f * pow(value, (float)(1.f / 2.4f)) - 0.055f;
}

inline float inverseGammaCorrect(float value) {
    if (value <= 0.04045f)
        return value * 1.f / 12.92f;
    return pow((value + 0.055f) * 1.f / 1.055f, (float)2.4f);
}

unique_ptr<RGBSpectrum[]> readImage(const string &name, Point2i *resolution);

RGBSpectrum *readImageEXR(const string &name, int *width, int *height,
                          Bounds2i *dataWindow = nullptr, Bounds2i *displayWindow = nullptr);

void writeImage(const string &name, const float *rgb, const Bounds2i &outputBounds,
                const Point2i &totalResolution);

};

#endif // IMAGEIO_H
