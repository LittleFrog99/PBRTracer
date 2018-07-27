#ifndef TEXTURE_MIX
#define TEXTURE_MIX

#include "core/texture.h"

template <class T1, class T2>
class ScaleTexture : public Texture<T2> {
public:
    ScaleTexture(const shared_ptr<Texture<T1>> &tex1, const shared_ptr<Texture<T2>> &tex2)
        : tex1(tex1), tex2(tex2) {}

    T2 evaluate(const SurfaceInteraction &si) const {
        return tex1->evaluate(si) * tex2->evaluate(si);
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

private:
    unique_ptr<TextureMapping2D> mapping;
    const T v00, v01, v10, v11;
};


#endif // TEXTURE_MIX
