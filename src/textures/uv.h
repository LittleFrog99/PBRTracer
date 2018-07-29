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
        // Initialize 2D texture mapping _map_ from _tp_
        unique_ptr<TextureMapping2D> map;
        string type = tp.findString("mapping", "uv");
        if (type == "uv") {
            float su = tp.findFloat("uscale", 1.);
            float sv = tp.findFloat("vscale", 1.);
            float du = tp.findFloat("udelta", 0.);
            float dv = tp.findFloat("vdelta", 0.);
            map.reset(new UVMapping2D(su, sv, du, dv));
        } else if (type == "spherical")
            map.reset(new SphericalMapping2D(tex2world.inverse()));
        else if (type == "cylindrical")
            map.reset(new CylindricalMapping2D(tex2world.inverse()));
        else if (type == "planar")
            map.reset(new PlanarMapping2D(tp.findVector3f("v1", Vector3f(1, 0, 0)),
                                          tp.findVector3f("v2", Vector3f(0, 1, 0)),
                                          tp.findFloat("udelta", 0.f), tp.findFloat("vdelta", 0.f)));
        else {
            ERROR("2D texture mapping \"%s\" unknown", type.c_str());
            map.reset(new UVMapping2D);
        }
        return new UVTexture<T>(move(map));
    }

private:
    static void convertOut(Spectrum &rgb, Spectrum *out) { *out = rgb; }
    static void convertOut(Spectrum &rgb, float *out) { *out = rgb.luminance(); }

    unique_ptr<TextureMapping2D> mapping;
};

#endif // TEXTURE_UV
