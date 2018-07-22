#include "microfacet.h"

OrenNayar::OrenNayar(const Spectrum &R, float sigma)
    : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {
    sigma = radians(sigma);
    float sigma2 = SQ(sigma);
    A = 1.0f - sigma2 / (2.0f * (sigma2 + 0.33f));
    B = 0.45f * sigma2 / (sigma2 + 0.09f);
}

Spectrum OrenNayar::compute_f(const Vector3f &wo, const Vector3f &wi) const {
    float sinThetaI = sinTheta(wi);
    float sinThetaO = sinTheta(wo);
    float maxCos = 0;
    if (sinThetaI > 1e-4 && sinThetaO > 1e-4) {
        float sinPhiI = sinPhi(wi), cosPhiI = cosPhi(wi), sinPhiO = sinPhi(wo), cosPhiO = cosPhi(wo);
        float dCos = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
        maxCos = max(0.0f, dCos);
    }
}
