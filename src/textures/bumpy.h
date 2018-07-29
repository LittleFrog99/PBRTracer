#ifndef TEXTURE_BUMPY
#define TEXTURE_BUMPY

#include "core/texture.h"
#include "paramset.h"

template <class T>
class FBmTexture : public Texture<T> {
public:
    FBmTexture(unique_ptr<TextureMapping3D> mapping, float omega, int octaves)
        : mapping(move(mapping)), omega(omega), octaves(octaves) {}

    T evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f p = mapping->map(si, &dpdx, &dpdy);
        return Noise::fBm(p, dpdx, dpdy, omega, octaves);
    }

    static FBmTexture<T> * create(const Transform &tex2world, const TextureParams &tp) {
        unique_ptr<TextureMapping3D> map(new TransformMapping3D(tex2world));
        return new FBmTexture<T>(move(map), tp.findInt("octaves", 8), tp.findFloat("roughness", .5f));
    }

private:
    unique_ptr<TextureMapping3D> mapping;
    const float omega;
    const int octaves;
};

template <class T>
class WrinkledTexture : public Texture<T> {
public:
    WrinkledTexture(unique_ptr<TextureMapping3D> mapping, float omega, int octaves)
        : mapping(move(mapping)), omega(omega), octaves(octaves) {}

    T evaluate(const SurfaceInteraction &si) const {
        Vector3f dpdx, dpdy;
        Point3f p = mapping->map(si, &dpdx, &dpdy);
        return Noise::turbulence(p, dpdx, dpdy, omega, octaves);
    }

    static WrinkledTexture<T> * create(const Transform &tex2world, const TextureParams &tp) {
        unique_ptr<TextureMapping3D> map(new TransformMapping3D(tex2world));
        return new WrinkledTexture<T>(move(map), tp.findInt("octaves", 8), tp.findFloat("roughness", .5f));
    }

private:
    unique_ptr<TextureMapping3D> mapping;
    const float omega;
    const int octaves;
};

#endif // TEXTURE_BUMPY
