#ifndef UTILITY_IMAGEIO
#define UTILITY_IMAGEIO

#include "core/spectrum.h"
#include "core/bounds.h"

namespace ImageIO {

unique_ptr<RGBSpectrum[]> readImage(const string &name, Point2i *resolution);

RGBSpectrum *readImageEXR(const string &name, int *width, int *height,
                          Bounds2i *dataWindow = nullptr, Bounds2i *displayWindow = nullptr);

void writeImage(const string &name, const Float *rgb, const Bounds2i &outputBounds,
                const Point2i &totalResolution);

};

#endif // IMAGEIO_H
