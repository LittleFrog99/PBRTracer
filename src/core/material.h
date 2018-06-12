#ifndef MATERIAL_H
#define MATERIAL_H


#include "memory.h"
#include "texture.h"

class SurfaceInteraction;

enum class TransportMode { Radiance, Importance };

class Material {
public:
    virtual void computeScatteringFuncs(SurfaceInteraction *si, MemoryArena &arena,
                                        TransportMode mode, bool allowMultipleLobes) const = 0;
    virtual ~Material();
    static void bump(const shared_ptr<Texture<Float>> &d, SurfaceInteraction *si);
};

#endif // MATERIAL_H
