#ifndef TEXTURE_CHECKERBOARD
#define TEXTURE_CHECKERBOARD

#include "core/texture.h"

enum class AAMethod { None, ClosedForm };

template <class T>
class Checkerboard2DTexture : public Texture<T> {
public:
    Checkerboard2DTexture(unique_ptr<TextureMapping2D> mapping, shared_ptr<Texture<T>> &tex1,
                          shared_ptr<Texture<T>> &tex2, AAMethod aaMethod)
        : mapping(move(mapping)), tex1(tex1), tex2(tex2), aaMethod(aaMethod) {}

    T evaluate(const SurfaceInteraction &si) const;

private:
    unique_ptr<TextureMapping2D> mapping;
    shared_ptr<Texture<T>> tex1, tex2;
    const AAMethod aaMethod;
};

template <class T>
class Checkerboard3DTexture : public Texture<T> {
public:
    Checkerboard3DTexture(unique_ptr<TextureMapping3D> mapping, shared_ptr<Texture<T>> &tex1,
                          shared_ptr<Texture<T>> &tex2)
        : mapping(move(mapping)), tex1(tex1), tex2(tex2) {}

    T evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f p = mapping->map(si, &dpdx, &dpdy);
        if (int(floor(p.x) + floor(p.y) + floor(p.z)) % 2 == 0)
            return tex1->evaluate(si);
        else return tex2->evaluate(si);
    }

private:
    unique_ptr<TextureMapping3D> mapping;
    shared_ptr<Texture<T>> tex1, tex2;
};

template <class T>
class CheckerboardTextureCreator {
public:
    static Texture<T> * create(const Transform &tex2world, const TextureParams &tp);
private:
    CheckerboardTextureCreator();
};

extern template class Checkerboard2DTexture<float>;
extern template class Checkerboard2DTexture<Spectrum>;
extern template class CheckerboardTextureCreator<float>;
extern template class CheckerboardTextureCreator<Spectrum>;

#endif // TEXTURE_CHECKERBOARD
