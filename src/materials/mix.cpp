#include "mix.h"
#include "core/interaction.h"
#include "core/bsdf.h"
#include "paramset.h"

void MixMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                             TransportMode mode, bool allowMultipleLobes) const
{
    Spectrum s1 = scale->evaluate(*si).clamp();
    Spectrum s2 = (Spectrum(1.0f) - s1).clamp();
    auto si2 = *si;
    m1->computeScatteringFunctions(si, arena, mode, allowMultipleLobes);
    m2->computeScatteringFunctions(&si2, arena, mode, allowMultipleLobes);
    for (int i = 0; i < si->bsdf->numComponents(); i++)
        si->bsdf->bxdfs[i] = ARENA_ALLOC(arena, ScaledBxDF)(si->bsdf->bxdfs[i], s1);
    for (int i = 0; i < si2.bsdf->numComponents(); i++)
        si->bsdf->add(ARENA_ALLOC(arena, ScaledBxDF)(si2.bsdf->bxdfs[i], s2));
}

MixMaterial * MixMaterial::create(const TextureParams &mp, const shared_ptr<Material> &m1,
                                  const shared_ptr<Material> &m2) {
    shared_ptr<Texture<Spectrum>> scale = mp.getTexture("amount", Spectrum(0.5f));
    return new MixMaterial(m1, m2, scale);
}
