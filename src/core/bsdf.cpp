#include "bsdf.h"
#include "core/sampling.h"

Spectrum BxDF::sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u, float *pdf,
                        BxDFType *sampledType) const
{
    *wi = Sampling::cosineSampleHemisphere(u);
    if (wo.z < 0) wi->z *= -1;
    *pdf = this->pdf(wo, *wi);
    return compute_f(wo, *wi);
}

// Any BxDF that overrides sample_f() must overrrides this method
float BxDF::pdf(const Vector3f &wo, const Vector3f &wi) const {
    return sameHemisphere(wo, wi) ? absCosTheta(wi) * INV_PI : 0;
}

Spectrum BxDF::rho_hd(const Vector3f &wo, int nSamples, const Point2f *u) const {
    Spectrum r;
    for (int i = 0; i < nSamples; i++) {
        Vector3f wi;
        float pdf = 0;
        Spectrum f = sample_f(wo, &wi, u[i], &pdf);
        if (pdf > 0)
            r += f * absCosTheta(wi) / pdf;
    }
    return r / nSamples;
}

Spectrum BxDF::rho_hh(int nSamples, const Point2f *u1, const Point2f *u2) const {
    Spectrum r;
    for (int i = 0; i < nSamples; i++) {
        Vector3f wo, wi;
        wo = Sampling::uniformSampleHemisphere(u1[i]);
        float pdfo = Sampling::uniformHemispherePdf(), pdfi;
        Spectrum f = sample_f(wo, &wi, u2[i], &pdfi);
        if (pdfi > 0)
            r += f * absCosTheta(wo) * absCosTheta(wi) / (pdfo * pdfi);
    }
    return r / (PI * nSamples);
}

Spectrum BSDF::compute_f(const Vector3f &woW, const Vector3f &wiW, BxDFType flags) const {
    Vector3f wi = worldToLocal(wiW), wo = worldToLocal(woW);
    bool reflect = dot(wiW, ng) * dot(woW, ng) > 0; // use geometry normal to decide which BxDF to use
    Spectrum f(0.f);
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->matchesFlags(flags) && ((reflect && (bxdfs[i]->type & BSDF_REFLECTION)) ||
                                              (!reflect && (bxdfs[i]->type & BSDF_TRANSMISSION))))
            f += bxdfs[i]->compute_f(wo, wi);
    return f;
}

Spectrum BSDF::sample_f(const Vector3f &woWorld, Vector3f *wiWorld, const Point2f &u, float *pdf,
                        BxDFType type, BxDFType *sampledType) const
{
    // Choose which BxDF to sample
    int matchingComps = numComponents(type);
    if (matchingComps == 0) {
        *pdf = 0;
        return 0;
    }
    int comp = min(int(floor(u[0] * matchingComps)), matchingComps - 1);
    BxDF *bxdf = nullptr;
    int count = comp;
    for (int i = 0; i < nBxDFs; i++)
        if (bxdfs[i]->matchesFlags(type) && count-- == 0) {
            bxdf = bxdfs[i];
            break;
        }

    // Remap sample u to [0, 1)^2
    Point2f uRemapped(u[0] * matchingComps - comp, u[1]);
    // Sample chosen BxDF
    Vector3f wi, wo = worldToLocal(woWorld);
    *pdf = 0;
    if (sampledType) *sampledType = bxdf->type;
    Spectrum f = bxdf->sample_f(wo, &wi, uRemapped, pdf, sampledType);
    if (*pdf == 0) return 0;
    *wiWorld = localToWorld(wi);

    // Compute overall PDF with all matching BxDFs
    if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1) { // skip perfect specular
        for (int i = 0; i < nBxDFs; i++)
            if (bxdfs[i] != bxdf && bxdfs[i]->matchesFlags(type))
                *pdf += bxdfs[i]->pdf(wo, wi);
    }
    *pdf /= matchingComps;

    // Compute value of BSDF for sampled direction
    if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1)
        f = compute_f(woWorld, *wiWorld, type);
    return f;
}

float BSDF::pdf(const Vector3f &woWorld, const Vector3f &wiWorld, BxDFType flags) const {
    if (nBxDFs == 0.f) return 0.f;
    Vector3f wo = worldToLocal(woWorld), wi = worldToLocal(wiWorld);
    if (wo.z == 0) return 0.;
    float pdf = 0.f;
    int matchingComps = 0;
    for (int i = 0; i < nBxDFs; ++i)
        if (bxdfs[i]->matchesFlags(flags)) {
            ++matchingComps;
            pdf += bxdfs[i]->pdf(wo, wi);
        }
    float v = matchingComps > 0 ? pdf / matchingComps : 0.f;
    return v;
}
