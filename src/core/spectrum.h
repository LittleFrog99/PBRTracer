#ifndef CORE_SPECTRUM
#define CORE_SPECTRUM

#include "stringprint.h"

enum class SpectrumType {
    Reflectance,
    Illuminant
};

namespace SpectrumUtil {

constexpr int sampledLambdaStart = 400;
constexpr int sampledLambdaEnd = 700;
constexpr int nSpectralSamples = 60;

constexpr int nCIESamples = 471;
extern const float CIE_lambda[nCIESamples];
extern const float CIE_X[nCIESamples];
extern const float CIE_Y[nCIESamples];
extern const float CIE_Z[nCIESamples];
constexpr float CIE_Y_integral = 106.856895f;

bool samplesAreSorted(const float *lambda, const float *vals, int n);
void sortSamples(float *lambda, float *vals, int n);
float interpolateSamples(const float *lambda, const float *vals, int n, float l);
float averageSamples(const float *lambda, const float *v, int n, float lambda0, float lambda1);

void blackbody(const float *lambda, int n, float T, float *Le); // lambda in nanometers
void blackbodyNormalized(const float *lambda, int n, float T, float *Le);

inline void XYZToRGB(const float xyz[3], float rgb[3]) {
    rgb[0] = 3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
    rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
    rgb[2] = 0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];
}

inline void RGBToXYZ(const float rgb[3], float xyz[3]) {
    xyz[0] = 0.412453f * rgb[0] + 0.357580f * rgb[1] + 0.180423f * rgb[2];
    xyz[1] = 0.212671f * rgb[0] + 0.715160f * rgb[1] + 0.072169f * rgb[2];
    xyz[2] = 0.019334f * rgb[0] + 0.119193f * rgb[1] + 0.950227f * rgb[2];
}

};

template <int nSpectrumSamples>
class CoefficientSpectrum {
public:
    CoefficientSpectrum(float v = 0.f) {
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

    CoefficientSpectrum operator * (float a) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] *= a;
        return ret;
    }

    CoefficientSpectrum & operator *= (float a) {
        for (int i = 0; i < nSpectrumSamples; ++i) channels[i] *= a;
        return *this;
    }

    inline friend CoefficientSpectrum operator * (float a, const CoefficientSpectrum &s) {
        return s * a;
    }

    CoefficientSpectrum operator / (float a) const {
        CoefficientSpectrum ret = *this;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] /= a;
        return ret;
    }

    CoefficientSpectrum & operator /= (float a) {
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

    template <int n>
    inline friend CoefficientSpectrum<n> sqrt(const CoefficientSpectrum<n> &s) {
        CoefficientSpectrum ret;
        for (int i = 0; i < n; ++i) ret.channels[i] = sqrt(s.channels[i]);
        return ret;
    }

    template <int n>
    inline friend CoefficientSpectrum<n> pow(const CoefficientSpectrum<n> &s, float e) {
        CoefficientSpectrum ret;
        for (int i = 0; i < n; ++i) ret.channels[i] = pow(s.channels[i], e);
        return ret;
    }

    template <int n>
    inline friend CoefficientSpectrum<n> exp(const CoefficientSpectrum<n> &s) {
        CoefficientSpectrum ret;
        for (int i = 0; i < n; ++i) ret.channels[i] = exp(s.channels[i]);
        return ret;
    }

    CoefficientSpectrum operator - () const {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i) ret.channels[i] = -channels[i];
        return ret;
    }

    inline friend ostream & operator << (ostream &os, const CoefficientSpectrum &s) {
        return os << s.toString();
    }

    string toString() const {
        string str = "[ ";
        for (int i = 0; i < nSpectrumSamples; ++i) {
            str += STRING_PRINTF("%f", channels[i]);
            if (i + 1 < nSpectrumSamples) str += ", ";
        }
        str += " ]";
        return str;
    }

    CoefficientSpectrum clamp(float low = 0, float high = INFINITY) const {
        CoefficientSpectrum ret;
        for (int i = 0; i < nSpectrumSamples; ++i)
            ret.channels[i] = Math::clamp(channels[i], low, high);
        return ret;
    }

    float maxComp() const {
        float m = channels[0];
        for (int i = 1; i < nSpectrumSamples; ++i)
            m = max(m, channels[i]);
        return m;
    }

    bool hasNaNs() const {
        for (int i = 0; i < nSpectrumSamples; ++i)
            if (isnan(channels[i])) return true;
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

    float & operator [] (int i) { return channels[i]; }
    float operator [] (int i) const { return channels[i]; }

    static constexpr int NUM_SAMPLES = nSpectrumSamples;

protected:
    float channels[nSpectrumSamples];
};

using namespace SpectrumUtil;

class SampledSpectrum : public CoefficientSpectrum<nSpectralSamples> {
public:
    SampledSpectrum(float v = 0.0f) : CoefficientSpectrum(v) {}
    SampledSpectrum(const CoefficientSpectrum<nSpectralSamples> &v)
        : CoefficientSpectrum<nSpectralSamples>(v) {}
    inline SampledSpectrum(const RGBSpectrum &r, SpectrumType t);

    static void init();

    static SampledSpectrum fromSampled(const float *lambda, const float *v, int n);
    static SampledSpectrum fromRGB(const float rgb[3], SpectrumType type);

    static SampledSpectrum fromXYZ(const float xyz[3], SpectrumType type) {
        float rgb[3];
        XYZToRGB(xyz, rgb);
        return fromRGB(rgb, type);
    }

    void toXYZ(float xyz[3]) const {
        xyz[0] = xyz[1] = xyz[2] = 0;
        for (int i = 0; i < nSpectralSamples; i++) {
            xyz[0] = X.channels[i] * channels[i];
            xyz[1] = Y.channels[i] * channels[i];
            xyz[2] = Z.channels[i] * channels[i];
        }
        float scale = float(sampledLambdaEnd - sampledLambdaStart) / float(CIE_Y_integral * nSpectralSamples);
        xyz[0] *= scale; xyz[1] *= scale; xyz[2] *= scale;
    }

    void toRGB(float rgb[3]) const {
        float xyz[3];
        toXYZ(xyz);
        XYZToRGB(xyz, rgb);
    }

    inline RGBSpectrum toRGBSpectrum() const;

    float luminance() const {
        float yy = 0.f;
        for (int i = 0; i < nSpectralSamples; ++i)
            yy += Y.channels[i] * channels[i];
        return yy * float(sampledLambdaEnd - sampledLambdaStart) / float(CIE_Y_integral * nSpectralSamples);
    }

private:
    static SampledSpectrum X, Y, Z;
    static SampledSpectrum rgbRefl2SpectWhite, rgbRefl2SpectCyan, rgbRefl2SpectMagenta,
    rgbRefl2SpectYellow, rgbRefl2SpectRed, rgbRefl2SpectGreen, rgbRefl2SpectBlue;
    static SampledSpectrum rgbIllum2SpectWhite, rgbIllum2SpectCyan, rgbIllum2SpectMagenta,
    rgbIllum2SpectYellow, rgbIllum2SpectRed, rgbIllum2SpectGreen, rgbIllum2SpectBlue;

    static constexpr int nRGB2SpectSamples = 32;
    static const float RGB2SpectLambda[nRGB2SpectSamples];
    static const float RGBRefl2SpectWhite[nRGB2SpectSamples];
    static const float RGBRefl2SpectCyan[nRGB2SpectSamples];
    static const float RGBRefl2SpectMagenta[nRGB2SpectSamples];
    static const float RGBRefl2SpectYellow[nRGB2SpectSamples];
    static const float RGBRefl2SpectRed[nRGB2SpectSamples];
    static const float RGBRefl2SpectGreen[nRGB2SpectSamples];
    static const float RGBRefl2SpectBlue[nRGB2SpectSamples];
    static const float RGBIllum2SpectWhite[nRGB2SpectSamples];
    static const float RGBIllum2SpectCyan[nRGB2SpectSamples];
    static const float RGBIllum2SpectMagenta[nRGB2SpectSamples];
    static const float RGBIllum2SpectYellow[nRGB2SpectSamples];
    static const float RGBIllum2SpectRed[nRGB2SpectSamples];
    static const float RGBIllum2SpectGreen[nRGB2SpectSamples];
    static const float RGBIllum2SpectBlue[nRGB2SpectSamples];
};

class RGBSpectrum : public CoefficientSpectrum<3> {
public:
    RGBSpectrum(float v = 0.f) : CoefficientSpectrum<3>(v) {}
    RGBSpectrum(const CoefficientSpectrum<3> &v) : CoefficientSpectrum<3>(v) {}

    static RGBSpectrum fromRGB(const float rgb[3]) {
        RGBSpectrum s;
        s.channels[0] = rgb[0];
        s.channels[1] = rgb[1];
        s.channels[2] = rgb[2];
        return s;
    }

    static RGBSpectrum fromXYZ(const float xyz[3]) {
        RGBSpectrum r;
        XYZToRGB(xyz, r.channels);
        return r;
    }

    static RGBSpectrum fromSampled(const float *lambda, const float *v, int n);

    void toRGB(float rgb[3]) const {
        rgb[0] = channels[0];
        rgb[1] = channels[1];
        rgb[2] = channels[2];
    }

    RGBSpectrum toRGBSpectrum() const { return *this; }

    void toXYZ(float xyz[3]) const { RGBToXYZ(channels, xyz); }

    float luminance() const {
        static const float YWeight[3] = { 0.212671f, 0.715160f, 0.072169f };
        return YWeight[0] * channels[0] + YWeight[1] * channels[1] + YWeight[2] * channels[2];
    }
};

inline RGBSpectrum SampledSpectrum::toRGBSpectrum() const {
    float rgb[3];
    toRGB(rgb);
    return RGBSpectrum::fromRGB(rgb);
}

inline SampledSpectrum::SampledSpectrum(const RGBSpectrum &r, SpectrumType t) {
    float rgb[3];
    r.toRGB(rgb);
    *this = fromRGB(rgb, t);
}


#endif // CORE_SPECTRUM
