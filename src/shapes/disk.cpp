#include "disk.h"
#include "core/stats.h"
#include "efloat.h"

bool Disk::intersect(const Ray &worldRay, Float *tHit, SurfaceInteraction *isect,
                     bool testAlphaTexture) const
{
    ProfilePhase p(Profiler::Stage::ShapeIntersect);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute plane intersection for disk
    if (ray.d.z == 0) return false;
    Float tShapeHit = (height - ray.o.z) / ray.d.z;
    if (tShapeHit <= 0 || tShapeHit >= ray.tMax) return false;

    // See if hit point is in inside disk radii and phiMax
    Point3f pHit = ray(tShapeHit);
    Float distSq = SQ(pHit.x) + SQ(pHit.y);
    if (distSq > SQ(radius) || distSq < SQ(innerRadius)) return false;
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;
    if (phi > phiMax) return false;

    // Find parametric representation of disk hit
    Float u = phi / phiMax;
    Float rHit = sqrt(distSq);
    Float v = (radius - rHit) / (radius - innerRadius);

    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
    Vector3f dpdv = Vector3f(pHit.x, pHit.y, 0) * (innerRadius - radius) / rHit;
    Normal3f dndu(0, 0, 0), dndv(0, 0, 0);

    // Compute error bounds for cylinder intersection
    Vector3f pError(0, 0, 0);

    // Initialize $SurfaceInteraction from parametric information
    *isect = (*objectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d,
                                                 dpdu, dpdv, dndu, dndv, ray.time, this));

    // Update $tHit for quadratic intersection
    *tHit = Float(tShapeHit);

    return true;
}

bool Disk::intersectP(const Ray &worldRay, bool testAlphaTexture) const  {
    ProfilePhase p(Profiler::Stage::ShapeIntersect);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute plane intersection for disk
    if (ray.d.z == 0) return false;
    Float tShapeHit = (height - ray.o.z) / ray.d.z;
    if (tShapeHit <= 0 || tShapeHit >= ray.tMax) return false;

    // See if hit point is in inside disk radii and phiMax
    Point3f pHit = ray(tShapeHit);
    Float distSq = SQ(pHit.x) + SQ(pHit.y);
    if (distSq > SQ(radius) || distSq < SQ(innerRadius)) return false;
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;
    if (phi > phiMax) return false;

    return true;
}
