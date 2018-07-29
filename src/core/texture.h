#ifndef CORE_TEXTURE
#define CORE_TEXTURE

#include "transform.h"
#include "interaction.h"

/* Texture Interface */
template <class T>
class Texture {
public:
    virtual T evaluate(const SurfaceInteraction &si) const = 0;
    virtual ~Texture() {}
};

template <class T>
class ConstantTexture : public Texture<T> {
public:
    ConstantTexture(const T &value) : value(value) {}
    T evaluate(const SurfaceInteraction &) const { return value; }

    static ConstantTexture<T> * create(const Transform &tex2world, const TextureParams &tp);

private:
    T value;
};

extern template class ConstantTexture<float>;
extern template class ConstantTexture<Spectrum>;

/* Texture Mapping Interfaces and Subclasses */
class TextureMapping2D {
public:
    virtual ~TextureMapping2D() {}
    virtual Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const = 0;
    static unique_ptr<TextureMapping2D> create(const Transform &tex2world, const TextureParams &tp);
};

class UVMapping2D : public TextureMapping2D {
public:
    UVMapping2D(float su = 1, float sv = 1, float du = 0, float dv = 0)
        : su(su), sv(sv), du(du), dv(dv) {}

    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const {
        *dstdx = Vector2f(su * si.dudx, sv * si.dvdx);
        *dstdy = Vector2f(su * si.dudy, sv * si.dvdy);
        return Point2f(su * si.uv[0] + du, sv * si.uv[1] + dv);
    }

private:
    const float su, sv, du, dv;
};

class SphericalMapping2D : public TextureMapping2D {
public:
    SphericalMapping2D(const Transform &worldToTexture) : worldToTexture(worldToTexture) {}

    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const;

private:
    Point2f sphere(const Point3f &p) const {
        Vector3f vec = normalize(worldToTexture(p) - Point3f());
        float theta = sphericalTheta(vec), phi = sphericalPhi(vec);
        return Point2f(theta / INV_PI, phi / INV_TWO_PI);
    }

    const Transform worldToTexture;
};

class CylindricalMapping2D : public TextureMapping2D {
public:
    CylindricalMapping2D(const Transform &worldToTexture) : worldToTexture(worldToTexture) {}

    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const;

private:
    Point2f cylinder(const Point3f &p) const {
        Vector3f vec = normalize(worldToTexture(p) - Point3f(0, 0, 0));
        return Point2f((PI + atan2(vec.y, vec.x)) * INV_TWO_PI, vec.z);
    }

    const Transform worldToTexture;
};

class PlanarMapping2D : public TextureMapping2D {
public:
    PlanarMapping2D(const Vector3f &vs, const Vector3f &vt, float ds = 0, float dt = 0)
        : vs(vs), vt(vt), ds(ds), dt(dt) {}

    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const {
        Vector3f vec(si.p);
        *dstdx = Vector2f(dot(si.dpdx, vs), dot(si.dpdx, vt));
        *dstdy = Vector2f(dot(si.dpdy, vs), dot(si.dpdy, vt));
        return Point2f(ds + dot(vec, vs), dt + dot(vec, vt));
    }

private:
    const Vector3f vs, vt;
    const float ds, dt;
};

class TextureMapping3D {
public:
    virtual ~TextureMapping3D() {}
    virtual Point3f map(const SurfaceInteraction &si, Vector3f *dpdx, Vector3f *dpdy) const = 0;
};

class TransformMapping3D : public TextureMapping3D {
public:
    TransformMapping3D(const Transform &worldToTexture) : worldToTexture(worldToTexture) {}

    Point3f map(const SurfaceInteraction &si, Vector3f *dpdx, Vector3f *dpdy) const {
        *dpdx = worldToTexture(si.dpdx);
        *dpdy = worldToTexture(si.dpdy);
        return worldToTexture(si.p);
    }

private:
    const Transform worldToTexture;
};

namespace Noise {

float perlin(float x, float y = 0.5f, float z = 0.5f);
float fBm(const Point3f &p, const Vector3f &dpdx, const Vector3f &dpdy, float omega, int maxOctaves);
float turbulence(const Point3f &p, const Vector3f &dpdx, const Vector3f &dpdy, float omega, int maxOctaves);
inline float perlin(const Point3f &p) { return perlin(p.x, p.y, p.z); }

constexpr int NOISE_PERM_SIZE = 256;
extern int NOISE_PERM[2 * NOISE_PERM_SIZE];

}

#endif // CORE_TEXTURE
