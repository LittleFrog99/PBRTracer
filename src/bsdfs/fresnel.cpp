#include "fresnel.h"

float Fresnel::dielectric_Fr(float cosThetaI, float etaI, float etaT) {
    cosThetaI = clamp(cosThetaI, -1, 1);
    // Potentially swap indices of refraction
    bool entering = cosThetaI > 0.0f;
    if (!entering) {
        swap(etaI, etaT);
        cosThetaI = abs(cosThetaI);
    }
    // Compute _cosThetaT_ using Snell's law
    float sinThetaI = sqrt(max(0.0f, 1 - SQ(cosThetaI)));
    float sinThetaT = etaI / etaT * sinThetaI;
    if (sinThetaI >= 1) return 1; // total internal reflection
    float cosThetaT = sqrt(max(0.0f, 1 - SQ(sinThetaT)));
    float rPara = (etaT * cosThetaI - etaI * cosThetaT) / (etaT * cosThetaI + etaI * cosThetaT);
    float rPerp = (etaI * cosThetaI - etaT * cosThetaT) / (etaI * cosThetaI + etaT * cosThetaT);
    return 0.5f * (SQ(rPara) + SQ(rPerp));
}

Spectrum Fresnel::conductor_Fr(float cosThetaI, const Spectrum &etaI, const Spectrum &etaT,
                               const Spectrum &k)
{
    cosThetaI = clamp(cosThetaI, -1, 1);
    Spectrum eta = etaT / etaI;
    Spectrum etak = k / etaI;

    float cosThetaI2 = cosThetaI * cosThetaI;
    float sinThetaI2 = 1.0f - cosThetaI2;
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

float Fresnel::moment1(float eta) {
    float eta2 = eta * eta, eta3 = eta2 * eta, eta4 = eta3 * eta, eta5 = eta4 * eta;
    if (eta < 1)
        return 0.45966f - 1.73965f * eta + 3.37668f * eta2 - 3.904945f * eta3 + 2.49277f * eta4
                - 0.68441f * eta5;
    else
        return -4.61686f + 11.1136f * eta - 10.4646f * eta2 + 5.11455f * eta3 - 1.27198f * eta4
                + 0.12746f * eta5;
}

float Fresnel::moment2(float eta) {
    float eta2 = eta * eta, eta3 = eta2 * eta, eta4 = eta3 * eta, eta5 = eta4 * eta;
    if (eta < 1) {
        return 0.27614f - 0.87350f * eta + 1.12077f * eta2 - 0.65095f * eta3 + 0.07883f * eta4
                + 0.04860f * eta5;
    } else {
        float r_eta = 1 / eta, r_eta2 = r_eta * r_eta, r_eta3 = r_eta2 * r_eta;
        return -547.033f + 45.3087f * r_eta3 - 218.725f * r_eta2 + 458.843f * r_eta + 404.557f * eta
                - 189.519f * eta2 + 54.9327f * eta3 - 9.00603f * eta4 + 0.63942f * eta5;
    }
}

Spectrum SpecularTransmission::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, float *pdf,
                                        BxDFType *sampledType) const
{
    // Figure out which eta is incident and which is transmitted
    bool entering = cosTheta(wo) > 0;
    float etaI = entering ? etaA : etaB;
    float etaT = entering ? etaB : etaA;
    // Compute ray direction for specular transmission
    if (!refract(wo, faceforward(Normal3f(0, 0, 1), wo), etaI / etaT, wi)) return 0.0f;
    *pdf = 1;
    Spectrum ft = T * (Spectrum(1.0f) - fresnel.evaluate(cosTheta(*wi)));
    // TODO: Account for non-symmetry with transmission to different medium
    return ft / absCosTheta(*wi);
}

Spectrum FresnelSpecular::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u, float *pdf,
                                   BxDFType *sampledType) const
{
    float F = Fresnel::dielectric_Fr(cosTheta(wo), etaA, etaB);
    if (u[0] < F) { // reflection
        *wi = Vector3f(-wo.x, -wo.y, wo.z);
        if (sampledType)
            *sampledType = BxDFType(BSDF_SPECULAR | BSDF_REFLECTION);
        *pdf = F;
        return F * R / absCosTheta(*wi);
    } else {
        bool entering = cosTheta(wo) > 0;
        float etaI = entering ? etaA : etaB;
        float etaT = entering ? etaB : etaA;
        if (!refract(wo, faceforward(Normal3f(0, 0, 1), wo), etaI / etaT, wi))
            return 0;
        Spectrum ft = T * (1 - F);
        // TODO: Account for non-symmetry with transmission to different medium
        if (sampledType)
            *sampledType = BxDFType(BSDF_SPECULAR | BSDF_TRANSMISSION);
        *pdf = 1 - F;
        return ft / absCosTheta(*wi);
    }
}
