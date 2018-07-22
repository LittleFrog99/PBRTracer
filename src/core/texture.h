#ifndef CORE_TEXTURE
#define CORE_TEXTURE

#include "transform.h"

template <typename T>
class Texture {
public:
    virtual T evaluate(const SurfaceInteraction &) const = 0;
    virtual ~Texture() {}
};

template <class T>
class ConstantTexture : public Texture<T> {
public:
    ConstantTexture(const T &value) : value(value) {}
    T evaluate(const SurfaceInteraction &) const { return value; }

private:
    T value;
};

class TextureMapping2D {
public:
    virtual ~TextureMapping2D();
    virtual Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const = 0;
};

class UVMapping2D : public TextureMapping2D {
public:
    UVMapping2D(float su = 1, float sv = 1, float du = 0, float dv = 0);
    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const;

private:
    const float su, sv, du, dv;
};

class SphericalMapping2D : public TextureMapping2D {
public:
    SphericalMapping2D(const Transform &worldToTexture)
        : worldToTexture(worldToTexture) {}
    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const;

private:
    Point2f sphere(const Point3f &P) const;
    const Transform worldToTexture;
};

class CylindricalMapping2D : public TextureMapping2D {
public:
    CylindricalMapping2D(const Transform &worldToTexture)
        : worldToTexture(worldToTexture) {}
    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const;

private:
    Point2f cylinder(const Point3f &p) const {
        Vector3f vec = normalize(worldToTexture(p) - Point3f(0, 0, 0));
        return Point2f((PI + std::atan2(vec.y, vec.x)) * INV_TWO_PI, vec.z);
    }
    const Transform worldToTexture;
};

class PlanarMapping2D : public TextureMapping2D {
public:
    PlanarMapping2D(const Vector3f &vs, const Vector3f &vt, float ds = 0, float dt = 0)
        : vs(vs), vt(vt), ds(ds), dt(dt) {}
    Point2f map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const;

private:
    const Vector3f vs, vt;
    const float ds, dt;
};

class TextureMapping3D {
public:
    virtual ~TextureMapping3D();
    virtual Point3f map(const SurfaceInteraction &si, Vector3f *dpdx, Vector3f *dpdy) const = 0;
};

class IdentityMapping3D : public TextureMapping3D {
public:
    IdentityMapping3D(const Transform &worldToTexture)
        : worldToTexture(worldToTexture) {}
    Point3f map(const SurfaceInteraction &si, Vector3f *dpdx, Vector3f *dpdy) const;

private:
    const Transform worldToTexture;
};

#endif // CORE_TEXTURE
