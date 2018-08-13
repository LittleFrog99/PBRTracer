#include "uber.h"
#include "bsdfs/microfacet.h"
#include "bsdfs/lambertian.h"
#include "core/texture.h"
#include "paramset.h"

void UberMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                              bool allowMultipleLobes) const
{
    // Perform bump mapping with _bumpMap_, if present
    if (bumpMap) bump(bumpMap, si);
    float e = eta->evaluate(*si);

    Spectrum op = opacity->evaluate(*si).clamp();
    Spectrum t = (-op + Spectrum(1.f)).clamp();
    if (!t.isBlack()) {
        si->bsdf = ARENA_ALLOC(arena, BSDF)(*si, 1.f);
        BxDF *tr = ARENA_ALLOC(arena, SpecularTransmission)(t, 1.f, 1.f, mode);
        si->bsdf->add(tr);
    } else
        si->bsdf = ARENA_ALLOC(arena, BSDF)(*si, e);

    Spectrum kd = op * Kd->evaluate(*si).clamp();
    if (!kd.isBlack()) {
        BxDF *diff = ARENA_ALLOC(arena, LambertianReflection)(kd);
        si->bsdf->add(diff);
    }

    Spectrum ks = op * Ks->evaluate(*si).clamp();
    if (!ks.isBlack()) {
        Fresnel *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.f, e);
        float roughu, roughv;
        if (roughnessu)
            roughu = roughnessu->evaluate(*si);
        else
            roughu = roughness->evaluate(*si);
        if (roughnessv)
            roughv = roughnessv->evaluate(*si);
        else
            roughv = roughu;
        if (remapRoughness) {
            roughu = TrowbridgeReitzDistribution::roughnessToAlpha(roughu);
            roughv = TrowbridgeReitzDistribution::roughnessToAlpha(roughv);
        }
        MicrofacetDistribution *distrib = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(roughu, roughv);
        BxDF *spec = ARENA_ALLOC(arena, MicrofacetReflection)(ks, distrib, fresnel);
        si->bsdf->add(spec);
    }

    Spectrum kr = op * Kr->evaluate(*si).clamp();
    if (!kr.isBlack()) {
        Fresnel *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.f, e);
        si->bsdf->add(ARENA_ALLOC(arena, SpecularReflection)(kr, fresnel));
    }

    Spectrum kt = op * Kt->evaluate(*si).clamp();
    if (!kt.isBlack())
        si->bsdf->add(ARENA_ALLOC(arena, SpecularTransmission)(kt, 1.f, e, mode));
}

UberMaterial * UberMaterial::create(const TextureParams &mp) {
    SpectrumTexture Kd = mp.getTexture("Kd", Spectrum(0.25f));
    SpectrumTexture Ks = mp.getTexture("Ks", Spectrum(0.25f));
    SpectrumTexture Kr = mp.getTexture("Kr", Spectrum(0.f));
    SpectrumTexture Kt = mp.getTexture("Kt", Spectrum(0.f));
    FloatTexture roughness = mp.getTexture("roughness", .1f);
    FloatTexture uroughness = mp.getFloatTextureOrNull("uroughness");
    FloatTexture vroughness = mp.getFloatTextureOrNull("vroughness");
    FloatTexture eta = mp.getFloatTextureOrNull("eta");
    if (!eta) eta = mp.getTexture("index", 1.5f);
    SpectrumTexture opacity = mp.getTexture("opacity", Spectrum(1.f));
    FloatTexture bumpMap = mp.getFloatTextureOrNull("bumpmap");
    bool remapRoughness = mp.findBool("remaproughness", true);
    return new UberMaterial(Kd, Ks, Kr, Kt, roughness, uroughness, vroughness, opacity, eta, bumpMap,
                            remapRoughness);
}
