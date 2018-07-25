#ifndef MATERIAL_MATTE
#define MATERIAL_MATTE

#include "core/material.h"

class MatteMaterial : public Material {
public:
    MatteMaterial(shared_ptr<Texture<Spectrum>> &Kd, shared_ptr<Texture<float>> &sigma,
                  shared_ptr<Texture<float>> &bumpMap)
        : Kd(Kd), sigma(sigma), bumpMap(bumpMap) {}

    static MatteMaterial * create(const TextureParams &mp);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                    TransportMode mode, bool allowMultipleLobes) const;

private:
    shared_ptr<Texture<Spectrum>> Kd;
    shared_ptr<Texture<float>> sigma, bumpMap;
};

#endif // MATERIAL_MATTE
