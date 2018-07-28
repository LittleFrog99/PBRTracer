#include "texture.h"
#include "paramset.h"

Point2f SphericalMapping2D::map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const
{
    Point2f st = sphere(si.p);
    // Compute texture coordinate differentials for sphere (u, v) mapping
    const float delta = .1f;
    Point2f stDeltaX = sphere(si.p + delta * si.dpdx);
    *dstdx = (stDeltaX - st) / delta;
    Point2f stDeltaY = sphere(si.p + delta * si.dpdy);
    *dstdy = (stDeltaY - st) / delta;
    // Handle sphere mapping discontinuity for coordinate differentials
    if ((*dstdx)[1] > 0.5f) (*dstdx)[1] = 1 - (*dstdx)[1];
    else if ((*dstdx)[1] < -.5f) (*dstdx)[1] = -((*dstdx)[1] + 1);
    if ((*dstdy)[1] > 0.5f) (*dstdy)[1] = 1 - (*dstdy)[1];
    else if ((*dstdy)[1] < -.5f) (*dstdy)[1] = -((*dstdy)[1] + 1);
    return st;
}

Point2f CylindricalMapping2D::map(const SurfaceInteraction &si, Vector2f *dstdx, Vector2f *dstdy) const
{
    Point2f st = cylinder(si.p);
    // Compute texture coordinate differentials for sphere (u, v) mapping
    const float delta = .1f;
    Point2f stDeltaX = cylinder(si.p + delta * si.dpdx);
    *dstdx = (stDeltaX - st) / delta;
    Point2f stDeltaY = cylinder(si.p + delta * si.dpdy);
    *dstdy = (stDeltaY - st) / delta;
    // Handle sphere mapping discontinuity for coordinate differentials
    if ((*dstdx)[1] > 0.5f) (*dstdx)[1] = 1 - (*dstdx)[1];
    else if ((*dstdx)[1] < -.5f) (*dstdx)[1] = -((*dstdx)[1] + 1);
    if ((*dstdy)[1] > 0.5f) (*dstdy)[1] = 1 - (*dstdy)[1];
    else if ((*dstdy)[1] < -.5f) (*dstdy)[1] = -((*dstdy)[1] + 1);
    return st;
}

template <class T>
ConstantTexture<T> * ConstantTexture<T>::create(const Transform &tex2world, const TextureParams &tp) {
    return new ConstantTexture<T>(tp.findFloat("value", 1.f));
}

template class ConstantTexture<float>;
template class ConstantTexture<Spectrum>;
