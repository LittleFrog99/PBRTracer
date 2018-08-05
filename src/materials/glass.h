#ifndef MATERIAL_GLASS
#define MATERIAL_GLASS

#include "core/material.h"

class GlassMaterial : public Material {
public:
    GlassMaterial(const shared_ptr<Texture<Spectrum>> &Kr, const shared_ptr<Texture<Spectrum>> &Kt,
                  const shared_ptr<Texture<float>> &uRoughness, const shared_ptr<Texture<float>> &vRoughness,
                  const shared_ptr<Texture<float>> &index, const shared_ptr<Texture<float>> &bumpMap,
                  bool remapRoughness)
        : Kr(Kr), Kt(Kt), uRoughness(uRoughness), vRoughness(vRoughness), index(index), bumpMap(bumpMap),
          remapRoughness(remapRoughness) {}

    static GlassMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                    bool allowMultipleLobes) const;

private:
    shared_ptr<Texture<Spectrum>> Kr, Kt;
    shared_ptr<Texture<float>> uRoughness, vRoughness;
    shared_ptr<Texture<float>> index;
    shared_ptr<Texture<float>> bumpMap;
    bool remapRoughness;
};

#endif // MATERIAL_GLASS
