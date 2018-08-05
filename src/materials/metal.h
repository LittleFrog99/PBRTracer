#ifndef MATERIAL_METAL
#define MATERIAL_METAL

#include "core/material.h"

class MetalMaterial : public Material {
public:
    MetalMaterial(const shared_ptr<Texture<Spectrum>> &eta, const shared_ptr<Texture<Spectrum>> &k,
                  const shared_ptr<Texture<float>> &roughness, const shared_ptr<Texture<float>> &uRoughness,
                  const shared_ptr<Texture<float>> &vRoughness, const shared_ptr<Texture<float>> &bumpMap,
                  bool remapRoughness)
        : eta(eta), k(k), roughness(roughness), uRoughness(uRoughness), vRoughness(vRoughness),
          bumpMap(bumpMap), remapRoughness(remapRoughness) {}

    static MetalMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                    bool allowMultipleLobes) const;

private:
    shared_ptr<Texture<Spectrum>> eta, k;
    shared_ptr<Texture<float>> roughness, uRoughness, vRoughness;
    shared_ptr<Texture<float>> bumpMap;
    bool remapRoughness;

    static constexpr int CopperSamples = 56;
    static const float CopperWavelengths[CopperSamples];
    static const float CopperN[CopperSamples];
    static const float CopperK[CopperSamples];
};

#endif // MATERIAL_METAL
