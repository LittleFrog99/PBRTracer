#ifndef MIX_H
#define MIX_H

#include "core/material.h"

class MixMaterial : public Material {
public:
    MixMaterial(const shared_ptr<Material> &m1, const shared_ptr<Material> &m2,
                const shared_ptr<Texture<Spectrum>> &scale)
        : m1(m1), m2(m2), scale(scale) {}

    static MixMaterial * create(const TextureParams &mp, const shared_ptr<Material> &m1,
                                const shared_ptr<Material> &m2);

    void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena, TransportMode mode,
                                    bool allowMultipleLobes) const;

private:
    shared_ptr<Material> m1, m2;
    shared_ptr<Texture<Spectrum>> scale;
};

#endif // MIX_H
