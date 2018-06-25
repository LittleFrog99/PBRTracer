#include "interaction.h"
#include "shape.h"

SurfaceInteraction::SurfaceInteraction(const Point3f &p, const Vector3f &pError,
                                       const Point2f &uv, const Vector3f &wo,
                                       const Vector3f &dpdu, const Vector3f &dpdv,
                                       const Normal3f &dndu, const Normal3f &dndv,
                                       Float time, const Shape *sh)
    : Interaction(p, Normal3f(normalize(cross(dpdu, dpdv))), pError, wo, time, nullptr),
      uv(uv), dpdu(dpdu), dpdv(dpdv), dndu(dndu), dndv(dndv), shape(sh)
{
    // Initialize shading geometry from true geometry
    shading.n = n;
    shading.dpdu = dpdu;
    shading.dpdv = dpdv;
    shading.dndu = dndu;
    shading.dndv = dndv;

    // Adjust normal based on orientation and handedness
    if (shape && (shape->reverseOrientation ^ shape->transformSwapsHandedness)) {
        n *= -1;
        shading.n *= -1;
    }
}

void SurfaceInteraction::setShadingGeometry(const Vector3f &dpdus, const Vector3f &dpdvs,
                                            const Normal3f &dndus, const Normal3f &dndvs,
                                            bool orientationIsAuthoritative)
{
    // Compute shading.n for SurfaceInteraction
    shading.n = Normal3f(normalize(cross(dpdus, dpdvs)));
    if (shape && (shape->reverseOrientation ^
    shape->transformSwapsHandedness))
    shading.n = -shading.n;
    if (orientationIsAuthoritative)
        n = faceforward(n, shading.n);
    else
        shading.n = faceforward(shading.n, n);

    // Initialize shading partial derivative values
    shading.dpdu = dpdus;
    shading.dpdv = dpdvs;
    shading.dndu = dndus;
    shading.dndv = dndvs;
}
