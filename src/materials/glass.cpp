#include "glass.h"
#include "bsdfs/microfacet.h"
#include "core/texture.h"
#include "paramset.h"

void GlassMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                               bool allowMultipleLobes) const
{
    if (bumpMap) bump(bumpMap, si);
    float eta = index->evaluate(*si);
    float urough = uRoughness->evaluate(*si);
    float vrough = vRoughness->evaluate(*si);
    Spectrum R = Kr->evaluate(*si).clamp();
    Spectrum T = Kt->evaluate(*si).clamp();

    // Initialize _bsdf_ for smooth or rough dielectric
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si, eta);

    if (R.isBlack() && T.isBlack()) return;

    bool isSpecular = (urough == 0 && vrough == 0);
    if (isSpecular && allowMultipleLobes) {
        si->bsdf->add(ARENA_ALLOC(arena, FresnelSpecular)(R, T, 1.f, eta, mode));
    } else {
        if (remapRoughness) {
            urough = TrowbridgeReitzDistribution::roughnessToAlpha(urough);
            vrough = TrowbridgeReitzDistribution::roughnessToAlpha(vrough);
        }
        MicrofacetDistribution *distrib =
                isSpecular ? nullptr
                           : ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(urough, vrough);
        if (!R.isBlack()) {
            Fresnel *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.f, eta);
            if (isSpecular)
                si->bsdf->add(ARENA_ALLOC(arena, SpecularReflection)(R, fresnel));
            else
                si->bsdf->add(ARENA_ALLOC(arena, MicrofacetReflection)(R, distrib, fresnel));
        }
        if (!T.isBlack()) {
            if (isSpecular)
                si->bsdf->add(ARENA_ALLOC(arena, SpecularTransmission)(T, 1.f, eta, mode));
            else
                si->bsdf->add(ARENA_ALLOC(arena, MicrofacetTransmission)(T, distrib, 1.f, eta, mode));
        }
    }

}


GlassMaterial * GlassMaterial::create(const TextureParams &mp) {
    shared_ptr<Texture<Spectrum>> Kr = mp.getTexture("Kr", Spectrum(1.f));
    shared_ptr<Texture<Spectrum>> Kt = mp.getTexture("Kt", Spectrum(1.f));
    shared_ptr<Texture<float>> eta = mp.getFloatTextureOrNull("eta");
    if (!eta) eta = mp.getTexture("index", 1.5f);
    shared_ptr<Texture<float>> roughu = mp.getTexture("uroughness", 0.f);
    shared_ptr<Texture<float>> roughv = mp.getTexture("vroughness", 0.f);
    shared_ptr<Texture<float>> bumpMap = mp.getFloatTextureOrNull("bumpmap");
    bool remapRoughness = mp.findBool("remaproughness", true);
    return new GlassMaterial(Kr, Kt, roughu, roughv, eta, bumpMap,
                             remapRoughness);
}
