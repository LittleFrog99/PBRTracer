#ifndef CORE_MATERIAL
#define CORE_MATERIAL


#include "memory.h"
#include "texture.h"

class SurfaceInteraction;

enum class TransportMode { Radiance, Importance };

namespace std {
string to_string(TransportMode mode) {
    return (mode == TransportMode::Radiance) ? "RADIANCE" : "IMPORTANCE";
}
}

class Material {
public:
    virtual void computeScatteringFuncs(SurfaceInteraction *si, MemoryArena &arena,
                                        TransportMode mode, bool allowMultipleLobes) const = 0;
    virtual ~Material();
    static void bump(const shared_ptr<Texture<float>> &d, SurfaceInteraction *si);
};

#endif // CORE_MATERIAL
