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

