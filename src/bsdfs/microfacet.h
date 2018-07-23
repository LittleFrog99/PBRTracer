#ifndef MICROFACET_H
#define MICROFACET_H

#include "core/bsdf.h"

class OrenNayar : public BxDF {
public:
    OrenNayar(const Spectrum &R, float sigma)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {
        sigma = radians(sigma);
        float sigma2 = SQ(sigma);
        A = 1.0f - sigma2 / (2.0f * (sigma2 + 0.33f));
        B = 0.45f * sigma2 / (sigma2 + 0.09f);
    }

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const;

    string toString() const {
        return "[ OrenNayar R: " + R.toString() + STRING_PRINTF(" A: %f B: %f ]", A, B);
    }

private:
    const Spectrum R;
    float A, B;
};

class MicrofacetDistribution {
public:
    virtual float D(const Vector3f &wh) const = 0; // normal distribution function
    virtual float lambda(const Vector3f &w) const = 0;
    virtual string toString() const = 0;

    float G1(const Vector3f &w) const { // masking-shadowing function
        return 1.0f / (1.0f + lambda(w));
    }

    inline static float roughnessToAlpha(float roughness) {
        roughness = max(roughness, 1e-3f);
        float x = log(roughness);
        return 1.62142f + 0.819955f * x + 0.1734f * SQ(x) + 0.0171201f * CUB(x) + 0.000640711f * QUAD(x);
    }

protected:
    MicrofacetDistribution(bool sampleVisibleArea) : sampleVisibleArea(sampleVisibleArea) {}
    const bool sampleVisibleArea;
};

inline ostream & operator << (ostream &os, const MicrofacetDistribution &md) {
    os << md.toString();
    return os;
}

class BeckmannDistribution : public MicrofacetDistribution {
public:
    BeckmannDistribution(float alphaX, float alphaY, bool sampleVis = true)
        : MicrofacetDistribution(sampleVis), alphaX(alphaX), alphaY(alphaY) {}

    float D(const Vector3f &wh) const {
        float tanTheta2 = tan2Theta(wh);
        if (isinf(tanTheta2)) return 0.0f;
        float cos4Theta = SQ(cos2Theta(wh));
        return exp(-tanTheta2 * (cos2Phi(wh) / SQ(alphaX) + sin2Phi(wh) / SQ(alphaY)))
                / (PI * alphaX * alphaY * cos4Theta);
    }

    string toString() const {
        return STRING_PRINTF("[ BeckmannDistribution alphax: %f alphay: %f ]", alphaX, alphaY);
    }

private:
    const float alphaX, alphaY;
};

class TrowbridgeReitzDistribution : public MicrofacetDistribution {
public:
    TrowbridgeReitzDistribution(float alphaX, float alphaY, bool sampleVis = true)
        : MicrofacetDistribution(sampleVis), alphaX(alphaX), alphaY(alphaY) {}

    float D(const Vector3f &wh) const {
        float tanTheta2 = tan2Theta(wh);
        if (isinf(tanTheta2)) return 0.0f;
        const float cos4Theta = SQ(cos2Theta(wh));
        float e = tanTheta2 * (cos2Phi(wh) / SQ(alphaX) + sin2Phi(wh) / SQ(alphaY));
        return 1.0f / (PI * alphaX * alphaY * cos4Theta * SQ(1 + e));
    }

    string toString() const {
        return STRING_PRINTF("[ TrowbridgeReitzDistribution alphax: %f alphay: %f ]", alphaX, alphaY);
    }

private:
    const float alphaX, alphaY;
};

#endif // MICROFACET_H
