#ifndef TEXTURE_UV
#define TEXTURE_UV

#include "core/texture.h"
#include "paramset.h"

template <class T>
class UVTexture : public Texture<T> {
public:
    UVTexture(unique_ptr<TextureMapping2D> mapping) : mapping(move(mapping)) {}

    T evaluate(const SurfaceInteraction &si) const {
        Vector2f dstdx, dstdy;
        Point2f st = mapping->map(si, &dstdx, &dstdy);
        float rgb[3] = { st[0] - floor(st[0]), st[1] - floor(st[1]), 0 };
        Spectrum spec = Spectrum::fromRGB(rgb);
        T out;
        convertOut(spec, &out);
        return out;
    }

    static UVTexture * create(const Transform &tex2world, const TextureParams &tp) {
        auto map = TextureMapping2D::create(tex2world, tp);
        return new UVTexture<T>(move(map));
    }

private:
    static void convertOut(Spectrum &rgb, Spectrum *out) { *out = rgb; }
    static void convertOut(Spectrum &rgb, float *out) { *out = rgb.luminance(); }

    unique_ptr<TextureMapping2D> mapping;
};

#endif // TEXTURE_UV
