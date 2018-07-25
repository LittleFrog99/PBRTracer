#ifndef PLASTIC_H
#define PLASTIC_H

#include "core/material.h"

class PlasticMaterial : public Material {
public:
    PlasticMaterial(shared_ptr<Texture<Spectrum>> &Kd, shared_ptr<Texture<Spectrum>> &Ks,
                    shared_ptr<Texture<float>> &roughness, shared_ptr<Texture<float>> &bumpMap,
                    bool remapRoughness)
        : Kd(Kd), Ks(Ks), roughness(roughness), bumpMap(bumpMap), remapRoughness(remapRoughness) {}

    static PlasticMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                    TransportMode mode, bool allowMultipleLobes) const;

private:
    shared_ptr<Texture<Spectrum>> Kd, Ks;
    shared_ptr<Texture<float>> roughness, bumpMap;
    const bool remapRoughness;
};

#endif // PLASTIC_H
