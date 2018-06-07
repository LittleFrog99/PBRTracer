#ifndef SPECTRUM_H
#define SPECTRUM_H


#include "stringprint.h"

enum class SpectrumType {
    Reflectance,
    Illuminant
};

class Spectrum {
public:
    inline static void XYZToRGB(const Float xyz[3], Float rgb[3]) {
        rgb[0] = 3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
        rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
        rgb[2] = 0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];
    }

    inline static void RGBToXYZ(const Float rgb[3], Float xyz[3]) {
        xyz[0] = 0.412453f * rgb[0] + 0.357580f * rgb[1] + 0.180423f * rgb[2];
        xyz[1] = 0.212671f * rgb[0] + 0.715160f * rgb[1] + 0.072169f * rgb[2];
        xyz[2] = 0.019334f * rgb[0] + 0.119193f * rgb[1] + 0.950227f * rgb[2];
    }
};

template <int nSpectrumSamples>
class CoefficientSpectrum {
public:

protected:
    Float channels[nSpectrumSamples];
};

class RGBSpectrum : public CoefficientSpectrum<3> {
public:
    RGBSpectrum();

    inline static RGBSpectrum fromRGB(const Float rgb[3]) {
        RGBSpectrum s;
        s.channels[0] = rgb[0];
        s.channels[1] = rgb[1];
        s.channels[2] = rgb[2];
        return s;
    }

    inline static RGBSpectrum fromXYZ(const Float xyz[3]) {
        RGBSpectrum r;
        Spectrum::XYZToRGB(xyz, r.channels);
        return r;
    }
};


#endif // SPECTRUM_H
