#ifndef SPECTRUM_H
#define SPECTRUM_H

#include "stringprint.h"

enum class SpectrumType {
    Reflectance,
    Illuminant
};

class Spectrum {
public:
    static constexpr int SAMPLED_LAMBDA_START = 400;
    static constexpr int SAMPLED_LAMBDA_END = 700;
    static constexpr int NUM_SPECTRAL_SAMPLES = 60;

    static constexpr int NUM_CIE_SAMPLES = 471;
    static const Float CIE_LAMBDA[NUM_CIE_SAMPLES];
    static const Float CIE_X[NUM_CIE_SAMPLES];
    static const Float CIE_Y[NUM_CIE_SAMPLES];
    static const Float CIE_Z[NUM_CIE_SAMPLES];
    static constexpr Float CIE_Y_INTEGRAL = 106.856895;

    static bool samplesAreSorted(const Float *lambda, const Float *vals, int n);
    static void sortSamples(Float *lambda, Float *vals, int n);
    static Float interpolateSamples(const Float *lambda, const Float *vals, int n, Float l);

    static void blackbody(const Float *lambda, int n, Float T, Float *Le);
    static void blackbodyNormalized(const Float *lambda, int n, Float T, Float *Le);
};

template <int nSpectrumSamples>
class CoefficientSpectrum : public Spectrum {
public:
    CoefficientSpectrum(Float v = 0.f) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] = v;
    }

    CoefficientSpectrum(const CoefficientSpectrum &s) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] = s.channels[i];
    }

    void print(FILE *f) const {
        fprintf(f, "[ ");
        for (int i = 0; i < nSpectrumSamples; ++i) {
            fprintf(f, "%f", channels[i]);
            if (i != nSpectrumSamples - 1) fprintf(f, ", ");
        }
        fprintf(f, "]");
    }

    CoefficientSpectrum & operator = (const CoefficientSpectrum &s) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] = s.channels[i];
        return *this;
    }

    CoefficientSpectrum & operator += (const CoefficientSpectrum &s2) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] += s2.channels[i];
        return *this;
    }

    CoefficientSpectrum operator + (const CoefficientSpectrum &s2) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] += s2.channels[i];
        return ret;
    }

    CoefficientSpectrum operator - (const CoefficientSpectrum &s2) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] -= s2.channels[i];
        return ret;
    }

    CoefficientSpectrum operator / (const CoefficientSpectrum &s2) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] /= s2.channels[i];
        return ret;
    }

    CoefficientSpectrum operator * (const CoefficientSpectrum &sp) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] *= sp.channels[i];
        return ret;
    }

    CoefficientSpectrum & operator *= (const CoefficientSpectrum &sp) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] *= sp.channels[i];
        return *this;
    }

    CoefficientSpectrum operator * (Float a) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] *= a;
        return ret;
    }

    CoefficientSpectrum & operator *= (Float a) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] *= a;
        return *this;
    }

    inline friend CoefficientSpectrum operator * (Float a, const CoefficientSpectrum &s) {
        return s * a;
    }

    CoefficientSpectrum operator / (Float a) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] /= a;
        return ret;
    }

    CoefficientSpectrum & operator /= (Float a) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] /= a;
        return *this;
    }

    bool operator == (const CoefficientSpectrum &sp) const {
        for (int i = 0; i < nSpectrumSamples; ++i)
            if (channels[i] != sp.channels[i]) return false;
        return true;
    }

    bool operator != (const CoefficientSpectrum &sp) const {
        return !(*this == sp);
    }

    bool isBlack() const {
        for (int i = 0; i < nSpectrumSamples; ++i)
            if (channels[i] != 0.0) return false;
        return true;
    }

    inline friend CoefficientSpectrum sqrt(const CoefficientSpectrum &s) {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] = sqrt(s.channels[i]);
        return ret;
    }

    template <int n>
    inline friend CoefficientSpectrum<n> pow(const CoefficientSpectrum<n> &s, Float e) {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] = pow(s.channels[i], e);
        return ret;
    }

    CoefficientSpectrum operator - () const {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] = -channels[i];
        return ret;
    }

    inline friend CoefficientSpectrum exp(const CoefficientSpectrum &s) {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] = exp(s.channels[i]);
        return ret;
    }

    inline friend ostream & operator << (ostream &os, const CoefficientSpectrum &s) {
        return os << s.toString();
    }

    string toString() const {
        std::string str = "[ ";
        for (int i = 0; i < nSpectrumSamples; ++i) {
            str += StringPrintf("%f", channels[i]);
            if (i + 1 < nSpectrumSamples) str += ", ";
        }
        str += " ]";
        return str;
    }

    CoefficientSpectrum clamp(Float low = 0, Float high = numeric_limits<Float>::infinity()) const {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i)
            ret.channels[i] = clamp(channels[i], low, high);
        return ret;
    }

    Float maxComp() const {
        Float m = channels[0];
        for (int i = 1; i < nSpectrumSamples; ++i)
            m = max(m, channels[i]);
        return m;
    }

    bool hasNaNs() const {
        for (int i = 0; i < nSpectrumSamples; ++i)
            if (std::isnan(channels[i])) return true;
        return false;
    }

    bool write(FILE *f) const {
        for (int i = 0; i < nSpectrumSamples; ++i)
            if (fprintf(f, "%f ", channels[i]) < 0) return false;
        return true;
    }

    bool read(FILE *f) {
        for (int i = 0; i < nSpectrumSamples; ++i) {
            double v;
            if (fscanf(f, "%lf ", &v) != 1) return false;
            channels[i] = v;
        }
        return true;
    }

    Float & operator [] (int i) {
        return channels[i];
    }

    Float operator [] (int i) const {
        return channels[i];
    }

    static constexpr int NUM_SAMPLES = nSpectrumSamples;

protected:
    Float channels[nSpectrumSamples];
};

class RGBSpectrum : public CoefficientSpectrum<3> {
public:
    RGBSpectrum(Float v = 0.f) : CoefficientSpectrum<3>(v) {}
    RGBSpectrum(const CoefficientSpectrum<3> &v) : CoefficientSpectrum<3>(v) {}

    static RGBSpectrum fromRGB(const Float rgb[3]) {
        RGBSpectrum s;
        s.channels[0] = rgb[0];
        s.channels[1] = rgb[1];
        s.channels[2] = rgb[2];
        return s;
    }

    static RGBSpectrum fromXYZ(const Float xyz[3]) {
        RGBSpectrum r;
        XYZToRGB(xyz, r.channels);
        return r;
    }

    void toRGB(Float *rgb) const {
        rgb[0] = channels[0];
        rgb[1] = channels[1];
        rgb[2] = channels[2];
    }

    static RGBSpectrum fromSampled(const Float *lambda, const Float *v, int n);

    const RGBSpectrum & toRGBSpectrum() const { return *this; }
    void toXYZ(Float xyz[3]) const { RGBToXYZ(channels, xyz); }

    Float luminance() const {
        static const Float YWeight[3] = { 0.212671f, 0.715160f, 0.072169f };
        return YWeight[0] * channels[0] + YWeight[1] * channels[1] + YWeight[2] * channels[2];
    }

private:
    static constexpr int NUM_RGB_TO_SPEC_SAMPLES = 32;
    /* static const Float RGB2SpectLambda[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectWhite[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectCyan[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectMagenta[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectYellow[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectRed[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectGreen[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBRefl2SpectBlue[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectWhite[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectCyan[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectMagenta[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectYellow[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectRed[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectGreen[NUM_RGB_TO_SPEC_SAMPLES];
    static const Float RGBIllum2SpectBlue[NUM_RGB_TO_SPEC_SAMPLES]; */

    static void XYZToRGB(const Float xyz[3], Float rgb[3]) {
        rgb[0] = 3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
        rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
        rgb[2] = 0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];
    }

    static void RGBToXYZ(const Float rgb[3], Float xyz[3]) {
        xyz[0] = 0.412453f * rgb[0] + 0.357580f * rgb[1] + 0.180423f * rgb[2];
        xyz[1] = 0.212671f * rgb[0] + 0.715160f * rgb[1] + 0.072169f * rgb[2];
        xyz[2] = 0.019334f * rgb[0] + 0.119193f * rgb[1] + 0.950227f * rgb[2];
    }
};


#endif // SPECTRUM_H
