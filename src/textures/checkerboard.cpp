#include "checkerboard.h"
#include "paramset.h"

template <class T>
T Checkerboard2DTexture<T>::evaluate(const SurfaceInteraction &si) const {
    Vector2f dstdx, dstdy;
    Point2f st = mapping->map(si, &dstdx, &dstdy);
    if (aaMethod == AAMethod::None) {
        if (int(floorf(st[0]) + floorf(st[1])) % 2 == 0)
            return tex1->evaluate(si);
        else return tex2->evaluate(si);
    } else {
        // Evaluate single check if filter is entirely in one of them
        float ds = max(abs(dstdx[0]), abs(dstdy[0]));
        float dt = max(abs(dstdx[1]), abs(dstdy[1]));
        float s0 = st[0] - ds, s1 = st[0] + ds;
        float t0 = st[1] - dt, t1 = st[1] + dt;
        if (floorf(s0) == floorf(s1) && floorf(t0) == floorf(t1)) {
            if (int(floorf(st[0]) + floorf(st[1])) % 2 == 0)
                return tex1->evaluate(si);
            else return tex2->evaluate(si);
        }

        // Apply box filter to checkerboard region
        constexpr auto bumpInt = [] (float x) {
            return floorf(x / 2) + 2 * max(x / 2 - floorf(x / 2) - 0.5f, 0.0f);
        };
        float sint = (bumpInt(s1) - bumpInt(s0)) / (2 * ds);
        float tint = (bumpInt(t1) - bumpInt(t0)) / (2 * dt);
        float area2 = sint + tint - 2 * sint * tint;
        if (ds > 1 || dt > 1) area2 = 0.5f;
        return (1 - area2) * tex1->evaluate(si) + area2 * tex2->evaluate(si);
    }
}

template <class T>
Texture<T> * CheckerboardTextureCreator<T>::create(const Transform &tex2world, const TextureParams &tp)
{
    int dim = tp.findInt("dimension", 2);
    if (dim != 2 && dim != 3) {
        ERROR("%d dimensional checkerboard texture not supported", dim);
        return nullptr;
    }
    shared_ptr<Texture<T>> tex1 = tp.getTexture("tex1", T(1.0f));
    shared_ptr<Texture<T>> tex2 = tp.getTexture("tex2", T(0.0f));
    if (dim == 2) {
        // Initialize 2D texture mapping _map_ from _tp_
        unique_ptr<TextureMapping2D> map;
        string type = tp.findString("mapping", "uv");
        if (type == "uv") {
            float su = tp.findFloat("uscale", 1.0f);
            float sv = tp.findFloat("vscale", 1.0f);
            float du = tp.findFloat("udelta", 0.0f);
            float dv = tp.findFloat("vdelta", 0.0f);
            map.reset(new UVMapping2D(su, sv, du, dv));
        } else if (type == "spherical")
            map.reset(new SphericalMapping2D(tex2world.inverse()));
        else if (type == "cylindrical")
            map.reset(new CylindricalMapping2D(tex2world.inverse()));
        else if (type == "planar")
            map.reset(new PlanarMapping2D(
                tp.findVector3f("v1", Vector3f(1, 0, 0)), tp.findVector3f("v2", Vector3f(0, 1, 0)),
                tp.findFloat("udelta", 0.f), tp.findFloat("vdelta", 0.f)));
        else {
            ERROR("2D texture mapping \"%s\" unknown", type.c_str());
            map.reset(new UVMapping2D);
        }

        // Compute _aaMethod_ for _CheckerboardTexture_
        string aa = tp.findString("aamode", "closedform");
        AAMethod aaMethod;
        if (aa == "none")
            aaMethod = AAMethod::None;
        else if (aa == "closedform")
            aaMethod = AAMethod::ClosedForm;
        else {
            WARNING("Antialiasing mode \"%s\" not understood by Checkerboard2DTexture; using \"closedform\"",
                    aa.c_str());
            aaMethod = AAMethod::ClosedForm;
        }
        return new Checkerboard2DTexture<T>(move(map), tex1, tex2, aaMethod);
    } else {
        // Initialize 3D texture mapping _map_ from _tp_
        unique_ptr<TextureMapping3D> map(new TransformMapping3D(tex2world));
        return new Checkerboard3DTexture<T>(move(map), tex1, tex2);
    }
}

template class Checkerboard2DTexture<float>;
template class Checkerboard2DTexture<Spectrum>;
template class CheckerboardTextureCreator<float>;
template class CheckerboardTextureCreator<Spectrum>;
