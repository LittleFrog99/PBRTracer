#include "microfacet.h"
#include "core/sampling.h"

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

float BeckmannDistribution::D(const Vector3f &wh) const {
    float tanTheta2 = tan2Theta(wh);
    if (isinf(tanTheta2)) return 0.0f;
    float cos4Theta = SQ(cos2Theta(wh));
    return exp(-tanTheta2 * (cos2Phi(wh) / SQ(alphaX) + sin2Phi(wh) / SQ(alphaY)))
            / (PI * alphaX * alphaY * cos4Theta);
}

float BeckmannDistribution::lambda(const Vector3f &w) const {
    float absTanTheta = abs(tanTheta(w));
    if (isinf(absTanTheta)) return 0.0f;
    float alpha = sqrt(cos2Phi(w) * SQ(alphaX) + sin2Phi(w) * SQ(alphaY));
    float a = 1.0f / (alpha * absTanTheta);
    if (a >= 1.6f) return 0.0f;
    return (1 - 1.259f * a + 0.396f * SQ(a)) / (3.535f * a + 2.181f * SQ(a));
}

Vector3f BeckmannDistribution::sample_wh(const Vector3f &wo, const Point2f &u) const {
    if (!sampleVisibleArea) { // full distribution
        // Compute tan^2θ and φ
        float tan2Theta, phi;
        if (alphaX == alphaY) {
            float logSample = log(u[0]);
            DCHECK(!isinf(logSample));
            tan2Theta = -SQ(alphaX) * logSample;
            phi = u[1] * 2 * PI;
        } else { // anisotropic roughness
            float logSample = log(u[0]);
            DCHECK(!isinf(logSample));
            phi = atan(alphaY / alphaX * tan(2 * PI * u[1] + 0.5f * PI));
            if (u[1] > 0.5f) phi += PI;
            float sinPhi = sin(phi), cosPhi = cos(phi);
            tan2Theta = -logSample / (SQ(cosPhi) / SQ(alphaX) + SQ(sinPhi) / SQ(alphaY));
        }

        // Map sampled angles to normal direction _wh_
        float cosTheta = 1 / sqrt(1 + tan2Theta);
        float sinTheta = sqrt(max(0.0f, 1 - SQ(cosTheta)));
        Vector3f wh = sphericalDirection(sinTheta, cosTheta, phi);
        if (!sameHemisphere(wo, wh)) wh = -wh;
        return wh;
    } else {
        // Sample visible area of normals for Beckmann distribution
        Vector3f wh;
        bool flip = wo.z < 0;
        wh = beckmannSample(flip ? -wo : wo, u);
        if (flip) wh = -wh;
        return wh;
    }
}

void BeckmannDistribution::beckmannSample11(float cosThetaI, const Point2f &u, float *slope_x, float *slope_y)
{
    // Normal incidence
    if (cosThetaI > .9999) {
        float r = sqrt(-log(1.0f - u[0]));
        float sinPhi = sin(2 * PI * u[1]);
        float cosPhi = cos(2 * PI * u[1]);
        *slope_x = r * cosPhi;
        *slope_y = r * sinPhi;
        return;
    }

    // Handle discontinuities
    float sinThetaI = sqrt(max(0.0f, 1.0f - cosThetaI * cosThetaI));
    float tanThetaI = sinThetaI / cosThetaI;
    float cotThetaI = 1 / tanThetaI;

    // Search interval, parameterized in the erf() domain */
    float a = -1, c = Math::erf(cotThetaI);
    float sample_x = max(u[0], 1e-6f);

    // Start with a good initial guess
    // float b = (1-sample_x) * a + sample_x * c;
    float thetaI = acos(cosThetaI);
    float fit = 1 + thetaI * (-0.876f + thetaI * (0.4265f - 0.0594f * thetaI));
    float b = c - (1 + c) * pow(1 - sample_x, fit);

    // Normalization factor for the CDF
    static const float SQRT_PI_INV = 1.f / sqrt(PI);
    float normalization = 1 / (1 + c + SQRT_PI_INV * tanThetaI * exp(-cotThetaI * cotThetaI));

    int it = 0;
    while (++it < 10) {
        // Bisection criterion
        if (!(b >= a && b <= c)) b = 0.5f * (a + c);

        // Evaluate the CDF and PDF
        float invErf = erfInv(b);
        float value = normalization * (1 + b + SQRT_PI_INV * tanThetaI * exp(-invErf * invErf)) - sample_x;
        float derivative = normalization * (1 - invErf * tanThetaI);

        if (abs(value) < 1e-5f) break;

        // Update bisection intervals
        if (value > 0) c = b;
        else a = b;
        b -= value / derivative;
    }

    // Convert back into a slope value
    *slope_x = erfInv(b);

    // Simulate Y component
    *slope_y = erfInv(2.0f * max(u[1], 1e-6f - 1.0f));

    CHECK(!isinf(*slope_x));
    CHECK(!isnan(*slope_x));
    CHECK(!isinf(*slope_y));
    CHECK(!isnan(*slope_y));
}

Vector3f BeckmannDistribution::beckmannSample(const Vector3f &wi, const Point2f &u) const
{
    // Stretch wi
    Vector3f wiStretched = normalize(Vector3f(alphaX * wi.x, alphaY * wi.y, wi.z));

    // Simulate P22_{wi}(x_slope, y_slope, 1, 1)
    float slope_x, slope_y;
    beckmannSample11(cosTheta(wiStretched), u, &slope_x, &slope_y);

    // Rotate
    float tmp = cosPhi(wiStretched) * slope_x - sinPhi(wiStretched) * slope_y;
    slope_y = sinPhi(wiStretched) * slope_x + cosPhi(wiStretched) * slope_y;
    slope_x = tmp;

    // Unstretch
    slope_x = alphaX * slope_x;
    slope_y = alphaY * slope_y;

    // Compute normal
    return normalize(Vector3f(-slope_x, -slope_y, 1.f));
}

float TrowbridgeReitzDistribution::D(const Vector3f &wh) const {
    float tanTheta2 = tan2Theta(wh);
    if (isinf(tanTheta2)) return 0.0f;
    const float cos4Theta = SQ(cos2Theta(wh));
    float e = tanTheta2 * (cos2Phi(wh) / SQ(alphaX) + sin2Phi(wh) / SQ(alphaY));
    return 1.0f / (PI * alphaX * alphaY * cos4Theta * SQ(1 + e));
}

float TrowbridgeReitzDistribution::lambda(const Vector3f &w) const {
    float absTanTheta = abs(tanTheta(w));
    if (isinf(absTanTheta)) return 0.0;
    float alpha = sqrt(cos2Phi(w) * SQ(alphaX) + sin2Phi(w) * SQ(alphaY));
    float alpha2Tan2Theta = (alpha * absTanTheta) * (alpha * absTanTheta);
    return (-1 + sqrt(1.f + alpha2Tan2Theta)) / 2;
}

Vector3f TrowbridgeReitzDistribution::sample_wh(const Vector3f &wo, const Point2f &u) const {
    Vector3f wh;
    if (!sampleVisibleArea) {
        float cosTheta = 0, phi = (2 * PI) * u[1];
        if (alphaX == alphaY) {
            float tanTheta2 = alphaX * alphaX * u[0] / (1.0f - u[0]);
            cosTheta = 1 / sqrt(1 + tanTheta2);
        } else {
            phi = atan(alphaY / alphaX * tan(2 * PI * u[1] + .5f * PI));
            if (u[1] > .5f) phi += PI;
            float sinPhi = sin(phi), cosPhi = cos(phi);
            const float alpha2 = 1 / (SQ(cosPhi) / SQ(alphaX) + SQ(sinPhi) / SQ(alphaY));
            float tanTheta2 = alpha2 * u[0] / (1 - u[0]);
            cosTheta = 1 / sqrt(1 + tanTheta2);
        }
        float sinTheta = sqrt(max((float)0., (float)1. - cosTheta * cosTheta));
        wh = sphericalDirection(sinTheta, cosTheta, phi);
        if (!sameHemisphere(wo, wh)) wh = -wh;
    } else {
        bool flip = wo.z < 0;
        wh = trowbridgeReitzSample(flip ? -wo : wo, u);
        if (flip) wh = -wh;
    }
    return wh;
}

void TrowbridgeReitzDistribution::trowbridgeReitzSample11(float cosTheta, Point2f u,
                                                          float *slope_x, float *slope_y)
{
    // Normal incidence
    if (cosTheta > .9999f) {
        float r = sqrt(u[0] / (1 - u[0]));
        float phi = 6.28318530718f * u[1];
        *slope_x = r * cos(phi);
        *slope_y = r * sin(phi);
        return;
    }

    float sinTheta = sqrt(max(0.0f, 1 - cosTheta * cosTheta));
    float tanTheta = sinTheta / cosTheta;
    float a = 1 / tanTheta;
    float G1 = 2 / (1 + sqrt(1.f + 1.f / (a * a)));

    // Sample slope_x
    float A = 2 * u[0] / G1 - 1;
    float tmp = 1.f / (A * A - 1.f);
    if (tmp > 1e10) tmp = 1e10f;
    float B = tanTheta;
    float D = sqrt(
        max(float(B * B * tmp * tmp - (A * A - B * B) * tmp), 0.0f));
    float slope_x_1 = B * tmp - D;
    float slope_x_2 = B * tmp + D;
    *slope_x = (A < 0 || slope_x_2 > 1.f / tanTheta) ? slope_x_1 : slope_x_2;

    // Sample slope_y
    float S;
    if (u[1] > 0.5f) {
        S = 1.f;
        u[1] = 2.f * (u[1] - .5f);
    } else {
        S = -1.f;
        u[1] = 2.f * (.5f - u[1]);
    }

    float z = (u[1] * (u[1] * (u[1] * 0.27385f - 0.73369f) + 0.46341f)) /
            (u[1] * (u[1] * (u[1] * 0.093073f + 0.309420f) - 1.000000f) + 0.597999f);
    *slope_y = S * z * sqrt(1.f + *slope_x * *slope_x);

    CHECK(!isinf(*slope_y));
    CHECK(!isnan(*slope_y));
}

Vector3f TrowbridgeReitzDistribution::trowbridgeReitzSample(const Vector3f &wi, const Point2f &u) const
{
    // Stretch wi
    Vector3f wiStretched = normalize(Vector3f(alphaX * wi.x, alphaY * wi.y, wi.z));

    // Simulate P22_{wi}(x_slope, y_slope, 1, 1)
    float slope_x, slope_y;
    trowbridgeReitzSample11(cosTheta(wiStretched), u, &slope_x, &slope_y);

    // Rotate
    float tmp = cosPhi(wiStretched) * slope_x - sinPhi(wiStretched) * slope_y;
    slope_y = sinPhi(wiStretched) * slope_x + cosPhi(wiStretched) * slope_y;
    slope_x = tmp;

    // Unstretch
    slope_x = alphaX * slope_x;
    slope_y = alphaY * slope_y;

    // Compute normal
    return normalize(Vector3f(-slope_x, -slope_y, 1.));
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

Spectrum MicrofacetReflection::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u, float *pdf,
                                        BxDFType *sampledType) const
{
    Vector3f wh = distrib->sample_wh(wo, u);
    *wi = reflect(wo, Normal3f(wh));
    if (!sameHemisphere(wo, *wi)) return 0.0f;
    *pdf = distrib->pdf(wo, wh) / (4 * dot(wo, wh));
    return compute_f(wo, *wi);
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

Spectrum MicrofacetTransmission::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u, float *pdf,
                                          BxDFType *sampledType) const
{
    Vector3f wh = distrib->sample_wh(wo, u);
    float eta = cosTheta(wo) > 0 ? (etaA / etaB) : (etaB / etaA);
    if (!refract(wo, Normal3f(wh), eta, wi)) return 0.0f;
    *pdf = this->pdf(wo, *wi);
    return compute_f(wo, *wi);
}

float MicrofacetTransmission::pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (sameHemisphere(wo, wi)) return 0;

    // Compute half vector
    float eta = cosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    Vector3f wh = normalize(wo + wi * eta);

    // Compute change of variables _dwh\_dwi_ for microfacet transmission
    float sqrtDenom = dot(wo, wh) + eta * dot(wi, wh);
    float dwh_dwi = abs((eta * eta * dot(wi, wh)) / (sqrtDenom * sqrtDenom));
    return distrib->pdf(wo, wh) * dwh_dwi;
}

Spectrum FresnelBlend::compute_f(const Vector3f &wo, const Vector3f &wi) const {
    Spectrum diffuse = (28.0f / (23.f * PI)) * Rd * (Spectrum(1.f) - Rs) *
                       (1 - POW5(1 - 0.5f * absCosTheta(wi))) * (1 - POW5(1 - 0.5f * absCosTheta(wo)));
    Vector3f wh = wi + wo;
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return 0;
    wh = normalize(wh);
    Spectrum specular = distrib->D(wh) / (4 * absDot(wi, wh) * max(absCosTheta(wi), absCosTheta(wo))) *
                        schlickFresnel(dot(wi, wh));
    return diffuse + specular;
}

Spectrum FresnelBlend::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &uOrig, float *pdf,
                                BxDFType *sampledType) const
{
    Point2f u = uOrig;
    if (u[0] < 0.5f) { // diffuse term
        u[0] *= 2;
        *wi = Sampling::cosineSampleHemisphere(u);
        if (wo.z < 0) wi->z *= -1;
    } else { // glossy term
        u[0] = 2 * (u[0] - 0.5f);
        Vector3f wh = distrib->sample_wh(wo, u);
        *wi = reflect(wo, Normal3f(wh));
        if (!sameHemisphere(wo, *wi)) return 0.0f;
    }
    *pdf = this->pdf(wo, *wi);
    return compute_f(wo, *wi);
}

float FresnelBlend::pdf(const Vector3f &wo, const Vector3f &wi) const {
    if (!sameHemisphere(wo, wi)) return 0;
    Vector3f wh = normalize(wo + wi);
    float pdf_wh = distrib->pdf(wo, wh);
    return 0.5f * (absCosTheta(wi) * INV_PI + pdf_wh / (4 * dot(wo, wh)));
}
