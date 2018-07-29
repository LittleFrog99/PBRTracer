#ifndef TEXTURE_WINDY
#define TEXTURE_WINDY

#include "core/texture.h"
#include "paramset.h"

template <class T>
class WindyTexture : public Texture<T> {
public:
    WindyTexture(unique_ptr<TextureMapping3D> mapping) : mapping(move(mapping)) {}

    T evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f p = mapping->map(si, &dpdx, &dpdy);
        float windStrength = Noise::fBm(p, dpdx, dpdy, 0.5, 3);
        float waveHeight = Noise::fBm(p, dpdx, dpdy, 0.5f, 6);
        return abs(windStrength) * waveHeight;
    }

    static WindyTexture<T> * create(const Transform &tex2world, const TextureParams &tp) {
        unique_ptr<TextureMapping3D> map(new TransformMapping3D(tex2world));
        return new WindyTexture<T>(move(map));
    }

private:
    unique_ptr<TextureMapping3D> mapping;
};

#endif // TEXTURE_WINDY
