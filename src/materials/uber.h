#ifndef MATERIAL_UBER
#define MATERIAL_UBER

#include "core/material.h"

class UberMaterial : public Material {
public:
    UberMaterial(const SpectrumTexture &Kd, const SpectrumTexture &Ks, const SpectrumTexture &Kr,
                 const SpectrumTexture &Kt, const FloatTexture &roughness, const FloatTexture &roughnessu,
                 const FloatTexture &roughnessv, const SpectrumTexture &opacity, const FloatTexture &eta,
                 const FloatTexture &bumpMap, bool remapRoughness)
        : Kd(Kd), Ks(Ks), Kr(Kr), Kt(Kt), opacity(opacity), roughness(roughness), roughnessu(roughnessu),
          roughnessv(roughnessv), eta(eta), bumpMap(bumpMap), remapRoughness(remapRoughness) {}

    static UberMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                    bool allowMultipleLobes) const;

private:
    SpectrumTexture Kd, Ks, Kr, Kt, opacity;
    FloatTexture roughness, roughnessu, roughnessv, eta,
    bumpMap;
    bool remapRoughness;
};

#endif // MATERIAL_UBER
