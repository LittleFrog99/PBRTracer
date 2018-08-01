#ifndef CORE_MATERIAL
#define CORE_MATERIAL

#include "memory.h"

struct SurfaceInteraction;
class TextureParams;
template <class T> class Texture;

enum class TransportMode { Radiance, Importance };

namespace std {
inline string to_string(TransportMode mode) {
    return (mode == TransportMode::Radiance) ? "RADIANCE" : "IMPORTANCE";
}
}

class Material {
public:
    virtual void computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                            TransportMode mode, bool allowMultipleLobes) const = 0;
    virtual ~Material() {}
    static void bump(const shared_ptr<Texture<float>> &d, SurfaceInteraction *si);
};

#endif // CORE_MATERIAL
