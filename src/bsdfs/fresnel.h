#ifndef BSDF_FRESNEL
#define BSDF_FRESNEL

#include "core/bsdf.h"

class Fresnel {
public:
    virtual Spectrum evaluate(Float cosThetaI) const = 0;
    virtual string toString() const = 0;

    static Float dielectric_Fr(Float cosThetaI, Float etaI, Float etaT);
    static Spectrum conductor_Fr(Float cosThetaI, const Spectrum &etaI, const Spectrum &etaT,
                                 const Spectrum &k);
};

class FresnelConductor : public Fresnel {
public:
    FresnelConductor(const Spectrum &etaI, const Spectrum &etaT, const Spectrum &k)
        : etaI(etaI), etaT(etaT), k(k) {}

    Spectrum evaluate(Float cosThetaI) const {
        return conductor_Fr(cosThetaI, etaI, etaT, k);
    }

    string toString() const {
        return "[ FresnelConductor etaI: " + etaI.toString() + " etaT: " + etaT.toString()
                + " k: " + k.toString() + " ]";
    }

private:
    Spectrum etaI, etaT, k;
};

class FresnelDielectric : public Fresnel {
public:
    FresnelDielectric(Float etaI, Float etaT) : etaI(etaI), etaT(etaT) {}

    Spectrum evaluate(Float cosThetaI) const {
        return dielectric_Fr(cosThetaI, etaI, etaT);
    }

    string toString() const {
        return STRING_PRINTF("[ FrenselDielectric etaI: %f etaT: %f ]", etaI, etaT);
    }

private:
    Float etaI, etaT;
};

class FresnelNoOp : public Fresnel {
public:
    Spectrum evaluate(Float) const { return 1.0f; }

    string toString() const {
        return "[ FresnelNoOp ]";
    }
};

class SpecularReflection : public BxDF {
public:
    SpecularReflection(const Spectrum &R, Fresnel *fresnel)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_SPECULAR)), R(R), fresnel(fresnel) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const { return 0.0f; }

    Spectrum sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf,
                      BxDFType *sampledType = nullptr) const {
        *wi = Vector3f(-wo.x, -wo.y, wo.z);
        *pdf = 1;
        return fresnel->evaluate(cosTheta(*wi)) * R / absCosTheta(*wi);
    }

    string toString() const {
        return "[ SpecularReflection R: " + R.toString() + " fresnel: " + fresnel->toString() + " ]";
    }

private:
    const Spectrum R;
    const Fresnel *fresnel;
};

class SpecularTransmission : public BxDF {
public:
    SpecularTransmission(const Spectrum &T, Float etaA, Float etaB, TransportMode mode)
        : BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR)), T(T), etaA(etaA), etaB(etaB),
          fresnel(etaA, etaB), mode(mode) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const { return 0.0f; }

    Spectrum sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf,
                      BxDFType *sampledType = nullptr) const;

    string toString() const {
        return "[ SpecularTransmission: T: " + T.toString() +
                STRING_PRINTF(" etaA: %f etaB: %f ", etaA, etaB) +
                " fresnel: " + fresnel.toString() +
                " mode : " + (mode == TransportMode::Radiance ? "RADIANCE" : "IMPORTANCE") + " ]";
    }

private:
    const Spectrum T;
    const Float etaA, etaB;
    const FresnelDielectric fresnel; // a conductor doesn't transmit light
    const TransportMode mode;
};

class FresnelSpecular : public BxDF {
public:
    FresnelSpecular(const Spectrum &R, const Spectrum &T, Float etaA, Float etaB, TransportMode mode)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_SPECULAR)),
          R(R), T(T), etaA(etaA), etaB(etaB), fresnel(etaA, etaB), mode(mode) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const { return 0.0f; }

    Spectrum sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf,
                      BxDFType *sampledType = nullptr) const;

    string toString() const {
        return "[ FresnelSpecular R: " + R.toString() + " T: " + T.toString() +
                STRING_PRINTF(" etaA: %f etaB: %f ", etaA, etaB) +
                " mode : " + (mode == TransportMode::Radiance ? "RADIANCE" : "IMPORTANCE") + " ]";
    }

private:
    const Spectrum R, T;
    const Float etaA, etaB;
    const FresnelDielectric fresnel;
    const TransportMode mode;
};


#endif // BSDF_FRESNEL
