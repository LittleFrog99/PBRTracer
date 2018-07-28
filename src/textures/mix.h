#ifndef TEXTURE_MIX
#define TEXTURE_MIX

#include "paramset.h"

template <class T1, class T2>
class ScaleTexture : public Texture<T2> {
public:
    ScaleTexture(const shared_ptr<Texture<T1>> &tex1, const shared_ptr<Texture<T2>> &tex2)
        : tex1(tex1), tex2(tex2) {}

    T2 evaluate(const SurfaceInteraction &si) const {
        return tex1->evaluate(si) * tex2->evaluate(si);
    }

    static ScaleTexture<T1, T2> * create(const Transform &tex2world, const TextureParams &tp) {
        return new ScaleTexture<T1, T2>(tp.getTexture("tex1", T1(1.0f)), tp.getTexture("tex2", T2(1.0f)));
    }

private:
    shared_ptr<Texture<T1>> tex1;
    shared_ptr<Texture<T2>> tex2;
};

template <class T>
class MixTexture : public Texture<T> {
public:
    MixTexture(const shared_ptr<Texture<T>> &tex1, const shared_ptr<Texture<T>> &tex2,
               const shared_ptr<Texture<float>> &amount)
        : tex1(tex1), tex2(tex2), amount(amount) {}

    T evaluate(const SurfaceInteraction &si) const {
        T t1 = tex1->evaluate(si), t2 = tex2->evaluate(si);
        float amt = amount->evaluate(si);
        return (1 - amt) * t1 + amt * t2;
    }

    static MixTexture<T> * create(const Transform &tex2world, const TextureParams &tp) {
        return new MixTexture<T>(tp.getTexture("tex1", T(0.0f)), tp.getTexture("tex2", T(1.0f)),
                                 tp.getTexture("amount", 0.5f));
    }

private:
    shared_ptr<Texture<T>> tex1, tex2;
    shared_ptr<Texture<float>> amount;
};

template <class T>
class BilerpTexture : public Texture<T> {
public:
    BilerpTexture(unique_ptr<TextureMapping2D> mapping, const T &v00, const T &v01, const T &v10,
                  const T &v11)
        : mapping(move(mapping)), v00(v00), v01(v01), v10(v10), v11(v11) {}

    T evaluate(const SurfaceInteraction &si) const {
        Vector2f dstdx, dstdy;
        Point2f st = mapping->map(si, &dstdx, &dstdy);
        return (1-st[0]) * (1-st[1]) * v00 + (1-st[0]) * (st[1]) * v01 +
                ( st[0]) * (1-st[1]) * v10 + ( st[0]) * (st[1]) * v11;
    }

    static BilerpTexture<T> * create(const Transform &tex2world, const TextureParams &tp);

private:
    static void construct(const TextureParams &tp, unique_ptr<TextureMapping2D> mapping,
                          BilerpTexture<float> **tex) {
        *tex = new BilerpTexture<float>(move(mapping), tp.findFloat("v00", 0.0f), tp.findFloat("v01", 1.0f),
                                        tp.findFloat("v10", 0.0f), tp.findFloat("v11", 1.0f));
    }

    static void construct(const TextureParams &tp, unique_ptr<TextureMapping2D> mapping,
                          BilerpTexture<Spectrum> **tex) {
        *tex = new BilerpTexture<Spectrum>(move(mapping), tp.findSpectrum("v00", 0.0f),
                                           tp.findSpectrum("v01", 1.0f), tp.findSpectrum("v10", 0.0f),
                                           tp.findSpectrum("v11", 1.0f));
    }

    unique_ptr<TextureMapping2D> mapping;
    const T v00, v01, v10, v11;
};

template <class T>
BilerpTexture<T> * BilerpTexture<T>::create(const Transform &tex2world, const TextureParams &tp) {
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
                                      tp.findFloat("udelta", 0.0f), tp.findFloat("vdelta", 0.0f)));
    else {
        ERROR("2D texture mapping \"%s\" unknown", type.c_str());
        map.reset(new UVMapping2D);
    }
    BilerpTexture<T> *ptr;
    construct(tp, move(map), &ptr);
    return ptr;
}


#endif // TEXTURE_MIX
