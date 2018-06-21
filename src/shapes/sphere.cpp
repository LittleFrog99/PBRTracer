#include "sphere.h"
#include "stats.h"
#include "efloat.h"

bool Sphere::intersect(const Ray &worldRay, Float *tHit,SurfaceInteraction *isect,
                       bool testAlphaTexture) const
{
    ProfilePhase p(Profiler::Stage::ShapeIntersect);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute quadratic sphere coefficients
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.x);
    auto a = SQ(dx) + SQ(dy) + SQ(dz);
    auto b = 2.0 * (dx * ox + dy * oy + dz * oz);
    auto c = SQ(ox) + SQ(oy) + SQ(oz) - SQ(EFloat(radius));

    // Solve quadratic equation for t values
    EFloat t0, t1;
    if (solveQuadratic(a, b, c, &t0, &t1)) return false;
    if (t0.upperBound() > ray.tMax || t1.lowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.lowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.upperBound() > ray.tMax)
            return false;
    }

    // Compute sphere hit position and phi
    Point3f pHit = ray(Float(tShapeHit));
    pHit *= radius / distance(pHit, Point3f(0, 0, 0));
    if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius; // why this?
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test sphere intersection against clipping paramaters
    // Find parametric representation of sphere hit
    // Compute error bounds for sphere intersection
    // Initialize $SurfaceInteraction from parametric information
    // Update $tHit for quadratic intersection
    return true;
}
