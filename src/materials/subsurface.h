#ifndef MATERIAL_SUBSURFACE
#define MATERIAL_SUBSURFACE

#include "core/material.h"
#include "bsdfs/diffusion.h"

class SubsurfaceMaterial : public Material {
public:
    SubsurfaceMaterial(float scale, const shared_ptr<Texture<Spectrum>> &Kr,
                       const shared_ptr<Texture<Spectrum>> &Kt,
                       const shared_ptr<Texture<Spectrum>> &sigma_a,
                       const shared_ptr<Texture<Spectrum>> &sigma_s,
                       float g, float eta,
                       const shared_ptr<Texture<float>> &uRoughness,
                       const shared_ptr<Texture<float>> &vRoughness,
                       const shared_ptr<Texture<float>> &bumpMap,
                       bool remapRoughness);

    static SubsurfaceMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,TransportMode mode,
                                    bool allowMultipleLobes) const;

private:
    float scale;
    shared_ptr<Texture<Spectrum>> Kr, Kt, sigma_a, sigma_s;
    shared_ptr<Texture<float>> uRoughness, vRoughness, bumpMap;
    const float eta;
    const bool remapRoughness;
    BSSRDFTable table;
};

class KdSubsurfaceMaterial : public Material {
public:
    KdSubsurfaceMaterial(float scale, const shared_ptr<Texture<Spectrum>> &Kd,
                         const shared_ptr<Texture<Spectrum>> &Kr,
                         const shared_ptr<Texture<Spectrum>> &Kt,
                         const shared_ptr<Texture<Spectrum>> &mfp,
                         float g, float eta,
                         const shared_ptr<Texture<float>> &uRoughness,
                         const shared_ptr<Texture<float>> &vRoughness,
                         const shared_ptr<Texture<float>> &bumpMap,
                         bool remapRoughness);

    static KdSubsurfaceMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                    bool allowMultipleLobes) const;

private:
    float scale;
    shared_ptr<Texture<Spectrum>> Kd, Kr, Kt, mfp;
    shared_ptr<Texture<float>> uRoughness, vRoughness, bumpMap;
    const float eta;
    const bool remapRoughness;
    BSSRDFTable table;
};

#endif // MATERIAL_SUBSURFACE
