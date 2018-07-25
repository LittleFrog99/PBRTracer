#include "bsdf.h"

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
