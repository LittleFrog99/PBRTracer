#include "fresnel.h"

Float Fresnel::dielectric_Fr(Float cosThetaI, Float etaI, Float etaT) {
    cosThetaI = clamp(cosThetaI, -1, 1);
    // Potentially swap indices of refraction
    bool entering = cosThetaI > 0.0f;
    if (!entering) {
        swap(etaI, etaT);
        cosThetaI = abs(cosThetaI);
    }
    // Compute _cosThetaT_ using Snell's law
    Float sinThetaI = sqrt(max(0.0f, 1 - SQ(cosThetaI)));
    Float sinThetaT = etaI / etaT * sinThetaI;
    if (sinThetaI >= 1) return 1; // total internal reflection
    Float cosThetaT = sqrt(max(0.0f, 1 - SQ(sinThetaT)));
    Float rPara = (etaT * cosThetaI - etaI * cosThetaT) / (etaT * cosThetaI + etaI * cosThetaT);
    Float rPerp = (etaI * cosThetaI - etaT * cosThetaT) / (etaI * cosThetaI + etaT * cosThetaT);
    return 0.5f * (SQ(rPara) + SQ(rPerp));
}

Spectrum Fresnel::conductor_Fr(Float cosThetaI, const Spectrum &etaI, const Spectrum &etaT,
                               const Spectrum &k)
{
    cosThetaI = clamp(cosThetaI, -1, 1);
    Spectrum eta = etaT / etaI;
    Spectrum etak = k / etaI;

    Float cosThetaI2 = cosThetaI * cosThetaI;
    Float sinThetaI2 = 1.0f - cosThetaI2;
    Spectrum eta2 = eta * eta;
    Spectrum etak2 = etak * etak;

    Spectrum t0 = eta2 - etak2 - sinThetaI2;
    Spectrum a2plusb2 = sqrt(t0 * t0 + 4 * eta2 * etak2);
    Spectrum t1 = a2plusb2 + cosThetaI2;
    Spectrum a = sqrt(0.5f * (a2plusb2 + t0));
    Spectrum t2 = 2.0f * cosThetaI * a;
    Spectrum Rs = (t1 - t2) / (t1 + t2);

    Spectrum t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
    Spectrum t4 = t2 * sinThetaI2;
    Spectrum Rp = Rs * (t3 - t4) / (t3 + t4);

    return 0.5 * (Rp + Rs);
}

Spectrum SpecularTransmission::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf,
                                        BxDFType *sampledType) const
{
    // Figure out which eta is incident and which is transmitted
    bool entering = cosTheta(wo) > 0;
    Float etaI = entering ? etaA : etaB;
    Float etaT = entering ? etaB : etaA;
    // Compute ray direction for specular transmission
    if (!refract(wo, faceforward(Normal3f(0, 0, 1), wo), etaI / etaT, wi)) return 0.0f;
    *pdf = 1;
    Spectrum ft = T * (Spectrum(1.0f) - fresnel.evaluate(cosTheta(*wi)));
    // TODO: Account for non-symmetry with transmission to different medium
    return ft / absCosTheta(*wi);
}
