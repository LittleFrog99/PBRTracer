#include "plastic.h"
#include "bsdfs/lambertian.h"
#include "bsdfs/microfacet.h"
#include "paramset.h"

void PlasticMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                                 TransportMode mode, bool allowMultipleLobes) const
{
    if (bumpMap) bump(bumpMap, si);
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si);
    Spectrum kd = Kd->evaluate(*si).clamp(); // diffuse component
    if (!kd.isBlack())
        si->bsdf->add(ARENA_ALLOC(arena, LambertianReflection)(kd));
    Spectrum ks = Ks->evaluate(*si).clamp();
    if (!ks.isBlack()) {
        auto *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.0f, 1.5f); // eta = 1.5
        float rough = roughness->evaluate(*si);
        if (remapRoughness) rough = MicrofacetDistribution::roughnessToAlpha(rough);
        auto *distrib = ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(rough, rough); // isotropic
        auto *spec = ARENA_ALLOC(arena, MicrofacetReflection)(ks, distrib, fresnel);
        si->bsdf->add(spec);
    }
}

PlasticMaterial * PlasticMaterial::create(const TextureParams &mp) {
    shared_ptr<Texture<Spectrum>> Kd = mp.getTexture("Kd", Spectrum(0.25f));
    shared_ptr<Texture<Spectrum>> Ks = mp.getTexture("Ks", Spectrum(0.25f));
    shared_ptr<Texture<float>> roughness = mp.getTexture("roughness", 0.1f);
    shared_ptr<Texture<float>> bumpMap = mp.getFloatTextureOrNull("bumpmap");
    bool remapRoughness = mp.findBool("remaproughness", true);
    return new PlasticMaterial(Kd, Ks, roughness, bumpMap, remapRoughness);
}
