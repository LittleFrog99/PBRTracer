#include "matte.h"
#include "bsdfs/lambertian.h"
#include "bsdfs/microfacet.h"
#include "paramset.h"

void MatteMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                               TransportMode mode, bool allowMultipleLobes) const
{
    if (bumpMap) bump(bumpMap, si);
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si);
    Spectrum r = Kd->evaluate(*si).clamp();
    float sig = clamp(sigma->evaluate(*si), 0, 90);
    if (!r.isBlack()) {
        if (sig == 0)
            si->bsdf->add(ARENA_ALLOC(arena, LambertianReflection)(r));
        else
            si->bsdf->add(ARENA_ALLOC(arena, OrenNayar)(r, sig));
    }
}

MatteMaterial * MatteMaterial::create(const TextureParams &mp) {
    shared_ptr<Texture<Spectrum>> Kd = mp.getTexture("Kd", Spectrum(0.5f));
    shared_ptr<Texture<float>> sigma = mp.getTexture("sigma", 0.0f);
    shared_ptr<Texture<float>> bumpMap = mp.getFloatTextureOrNull("bumpmap");
    return new MatteMaterial(Kd, sigma, bumpMap);
}
