#include "interaction.h"
#include "shape.h"
#include "primitive.h"

SurfaceInteraction::SurfaceInteraction(const Point3f &p, const Vector3f &pError,
                                       const Point2f &uv, const Vector3f &wo,
                                       const Vector3f &dpdu, const Vector3f &dpdv,
                                       const Normal3f &dndu, const Normal3f &dndv,
                                       float time, const Shape *sh)
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
    if (shape && (shape->reverseOrientation ^ shape->transformSwapsHandedness))
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

void SurfaceInteraction::computeScatteringFunctions(const RayDifferential &ray, MemoryArena &arena,
                                                bool allowMultipleLobes, TransportMode mode)
{
    computeDifferentials(ray);
    primitive->computeScatteringFunctions(this, arena, mode, allowMultipleLobes);
}

void SurfaceInteraction::computeDifferentials(const RayDifferential &ray) const {
    if (ray.hasDifferentials) {
        // Compute auxiliary intersection points with plane
        float d = dot(n, Vector3f(p));
        float tx = -(dot(n, Vector3f(ray.rxOrigin)) - d) / dot(n, ray.rxDirection);
        Point3f px = ray.rxOrigin + tx * ray.rxDirection;
        float ty = -(dot(n, Vector3f(ray.ryOrigin)) - d) / dot(n, ray.ryDirection);
        Point3f py = ray.ryOrigin + ty * ray.ryDirection;
        dpdx = px - p;
        dpdy = py - p;

        // Compute (u, v) offsets at auxiliary points
        // Choose two dimensions to use for ray offset computation
        int dim[2];
        if (abs(n.x) > abs(n.y) && abs(n.x) > abs(n.z)) {
            dim[0] = 1; dim[1] = 2;
        } else if (abs(n.y) > abs(n.z)) {
            dim[0] = 0; dim[1] = 2;
        } else {
            dim[0] = 0; dim[1] = 1;
        }
        // Initialize A, Bx, and By matrices for offset computation
        float A[2][2] = { { dpdu[dim[0]], dpdv[dim[0]] }, { dpdu[dim[1]], dpdv[dim[1]] } };
        float Bx[2] = { px[dim[0]] - p[dim[0]], px[dim[1]] - p[dim[1]] };
        float By[2] = { py[dim[0]] - p[dim[0]], py[dim[1]] - p[dim[1]] };

        if (!solveLinear2x2(A, Bx, &dudx, &dvdx)) dudx = dvdx = 0;
        if (!solveLinear2x2(A, By, &dudy, &dvdy)) dudy = dvdy = 0;
    } else  {
        dudx = dvdx = 0;
        dudy = dvdy = 0;
        dpdx = dpdy = Vector3f(0, 0, 0);
    }
}

Spectrum SurfaceInteraction::compute_Le(const Vector3f &w) const {
    const auto area = primitive->getAreaLight();
    return area ? area->compute_L(*this, w) : 0.0f;
}

bool SurfaceInteraction::isEmissive() const {
    if (primitive) return primitive->getAreaLight();
    return false;
}
