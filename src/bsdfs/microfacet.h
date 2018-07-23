#ifndef MICROFACET_H
#define MICROFACET_H

#include "fresnel.h"

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

    float G(const Vector3f &wo, const Vector3f &wi) const {
        return 1.0f / (1.0f + lambda(wo) + lambda(wi));
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

    float lambda(const Vector3f &w) const {
        float absTanTheta = abs(tanTheta(w));
        if (isinf(absTanTheta)) return 0.0f;
        float alpha = sqrt(cos2Phi(w) * SQ(alphaX) + sin2Phi(w) * SQ(alphaY));
        float a = 1.0f / (alpha * absTanTheta);
        if (a >= 1.6f) return 0.0f;
        return (1 - 1.259f * a + 0.396f * SQ(a)) / (3.535f * a + 2.181f * SQ(a));
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

    float lambda(const Vector3f &w) const {
        float absTanTheta = abs(tanTheta(w));
        if (isinf(absTanTheta)) return 0.0;
        float alpha = sqrt(cos2Phi(w) * SQ(alphaX) + sin2Phi(w) * SQ(alphaY));
        float alpha2Tan2Theta = (alpha * absTanTheta) * (alpha * absTanTheta);
        return (-1 + sqrt(1.f + alpha2Tan2Theta)) / 2;
    }

    string toString() const {
        return STRING_PRINTF("[ TrowbridgeReitzDistribution alphax: %f alphay: %f ]", alphaX, alphaY);
    }

private:
    const float alphaX, alphaY;
};

class MicrofacetReflection : public BxDF {
public:
    MicrofacetReflection(const Spectrum &R, MicrofacetDistribution *distrib, Fresnel *fresnel)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_GLOSSY)), R(R), distrib(distrib), fresnel(fresnel) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const;

    string toString() const {
        return "[ MicrofacetReflection R: " + R.toString() + " distribution: " + distrib->toString() +
               " fresnel: " + fresnel->toString() + " ]";
    }

private:
    const Spectrum R;
    const MicrofacetDistribution *distrib;
    const Fresnel *fresnel;
};

class MicrofacetTransmission : public BxDF {
public:
    MicrofacetTransmission(const Spectrum &T, MicrofacetDistribution *distrib, float etaA, float etaB,
                           TransportMode mode)
        : BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_GLOSSY)), T(T), distrib(distrib),
          etaA(etaA), etaB(etaB), fresnel(etaA, etaB), mode(mode) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const;

    string toString() const {
        return "[ MicrofacetTransmission T: " + T.toString() + " distribution: " + distrib->toString() +
               STRING_PRINTF(" etaA: %f etaB: %f", etaA, etaB) + " fresnel: " + fresnel.toString() +
               " mode : " + to_string(mode) + " ]";
    }

private:
    const Spectrum T;
    MicrofacetDistribution *distrib;
    float etaA, etaB;
    FresnelDielectric fresnel;
    TransportMode mode;
};

class FresnelBlend : public BxDF {
public:
    FresnelBlend(const Spectrum &Rd, const Spectrum &Rs, MicrofacetDistribution *distrib)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_GLOSSY)), Rd(Rd), Rs(Rs), distrib(distrib) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const;

    string toString() const {
        return "[ FresnelBlend Rd: " + Rd.toString() + " Rs: " + Rs.toString() +
               " distribution: " + distrib->toString() + " ]";
    }

    Spectrum schlickFresnel(float cosTheta) const {
        return Rs + POW5(1 - cosTheta) * (Spectrum(1.0f) - Rs);
    }

private:
    Spectrum Rd, Rs;
    MicrofacetDistribution *distrib;
};

#endif // MICROFACET_H
