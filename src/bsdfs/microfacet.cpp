#include "microfacet.h"

Spectrum OrenNayar::compute_f(const Vector3f &wo, const Vector3f &wi) const {
    float sinThetaI = sinTheta(wi);
    float sinThetaO = sinTheta(wo);
    float maxCos = 0;
    if (sinThetaI > 1e-4f && sinThetaO > 1e-4f) {
        float sinPhiI = sinPhi(wi), cosPhiI = cosPhi(wi);
        float sinPhiO = sinPhi(wo), cosPhiO = cosPhi(wo);
        float dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
        maxCos = max(0.0f, dCos);
    }
    float sinAlpha, tanBeta;
    if (absCosTheta(wi) > absCosTheta(wo)) {
        sinAlpha = sinThetaO;
        tanBeta = sinThetaI / absCosTheta(wi);
    } else {
        sinAlpha = sinThetaI;
        tanBeta = sinThetaO / absCosTheta(wo);
    }
    return R * INV_PI * (A + B * maxCos * sinAlpha * tanBeta);
}

Spectrum MicrofacetReflection::compute_f(const Vector3f &wo, const Vector3f &wi) const {
    float cosThetaO = absCosTheta(wo), cosThetaI = absCosTheta(wi);
    Vector3f wh = wi + wo;
    if (cosThetaI == 0 || cosThetaO == 0) return Spectrum(0.0f); // degenerate cases
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0.0f);
    wh = normalize(wh);
    Spectrum F = fresnel->evaluate(dot(wi, wh));
    return R * distrib->D(wh) * distrib->G(wo, wi) * F / (4 * cosThetaI * cosThetaO);
}

Spectrum MicrofacetTransmission::compute_f(const Vector3f &wo, const Vector3f &wi) const {
    if (sameHemisphere(wo, wi)) return 0;  // reflection only

    float cosThetaO = cosTheta(wo);
    float cosThetaI = cosTheta(wi);
    if (cosThetaI == 0 || cosThetaO == 0) return Spectrum(0);

    // Compute _wh_ from _wo_ and _wi_ for microfacet transmission
    float eta = cosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    Vector3f wh = normalize(wo + wi * eta);
    if (wh.z < 0) wh = -wh;

    Spectrum F = fresnel.evaluate(dot(wo, wh));

    float sqrtDenom = dot(wo, wh) + eta * dot(wi, wh);
    float factor = (mode == TransportMode::Radiance) ? (1 / eta) : 1;

    return (Spectrum(1.0f) - F) * T * abs(distrib->D(wh) * distrib->G(wo, wi) * SQ(eta) *
                                          absDot(wi, wh) * absDot(wo, wh) * SQ(factor) /
                                          (cosThetaI * cosThetaO * SQ(sqrtDenom)));
}

Spectrum FresnelBlend::compute_f(const Vector3f &wo, const Vector3f &wi) const {
    Spectrum diffuse = (28.0f / (23.f * PI)) * Rd * (Spectrum(1.f) - Rs) *
                       (1 - POW5(1 - 0.5f * absCosTheta(wi))) * (1 - POW5(1 - 0.5f * absCosTheta(wo)));
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Spectrum(0);
    wh = normalize(wh);
    Spectrum specular = distrib->D(wh) / (4 * absDot(wi, wh) * max(absCosTheta(wi), absCosTheta(wo))) *
                        schlickFresnel(dot(wi, wh));
    return diffuse + specular;
}
