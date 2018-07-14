#include "cylinder.h"
#include "core/stats.h"
#include "core/paramset.h"
#include "efloat.h"

bool Cylinder::intersect(const Ray &worldRay, Float *tHit, SurfaceInteraction *isect,
                         bool testAlphaTexture) const
{
    ProfilePhase p(Profiler::Stage::ShapeIntersect);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute quadratic cylinder coefficients
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.x);
    auto a = SQ(dx) + SQ(dy);
    auto b = 2.0 * (dx * ox + dy * oy);
    auto c = SQ(ox) + SQ(oy) - SQ(EFloat(radius));

    // Solve quadratic equation for t values
    EFloat t0, t1;
    if (!solveQuadratic(a, b, c, &t0, &t1)) return false;
    if (t0.upperBound() > ray.tMax || t1.lowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.lowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.upperBound() > ray.tMax)
            return false;
    }

    // Compute cylinder hit position and phi
    Point3f pHit = ray(Float(tShapeHit));
    Float hitRadius = sqrt(SQ(pHit.x) + SQ(pHit.y));
    pHit.x *= radius / hitRadius;
    pHit.y *= radius / hitRadius;
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test cylinder intersection against clipping paramaters
    if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) {
        if (tShapeHit == t1) return false;
        if (t1.upperBound() > ray.tMax) return false;
        tShapeHit = t1;

        Point3f pHit = ray(Float(tShapeHit));
        Float hitRadius = sqrt(SQ(pHit.x) + SQ(pHit.y));
        pHit.x *= radius / hitRadius;
        pHit.y *= radius / hitRadius;
        Float phi = atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * PI;

        if (pHit.z < zMin || pHit.z > zMax || phi > phiMax)
            return false;
    }

    // Find parametric representation of cylinder hit
    Float u = phi / phiMax;
    Float v = (pHit.z - zMin) / (zMax - zMin);

    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
    Vector3f dpdv(0, 0, zMax - zMin);
    Vector3f d2pduu = -SQ(phiMax) * Vector3f(pHit.x, pHit.y, 0);
    Vector3f d2pduv(0, 0, 0), d2pdvv(0, 0, 0);
    Float E = dot(dpdu, dpdu), F = dot(dpdu, dpdv), G = dot(dpdv, dpdv);
    Vector3f N = normalize(cross(dpdu, dpdv));
    Float e = dot(N, d2pduu), f = dot(N, d2pduv), g = dot(N, d2pdvv);
    Float invEGF2 = 1.0 / (E * G - F * F);
    auto dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu + (e * F - f * E) * invEGF2 * dpdv);
    auto dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu + (f * F - g * E) * invEGF2 * dpdv);

    // Compute error bounds for cylinder intersection
    Vector3f pError = gamma(3) * abs(Vector3f(pHit));

    // Initialize $SurfaceInteraction from parametric information
    *isect = (*objectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d,
                                                 dpdu, dpdv, dndu, dndv, ray.time, this));

    // Update $tHit for quadratic intersection
    *tHit = Float(tShapeHit);

    return true;
}

bool Cylinder::intersectP(const Ray &worldRay, bool testAlphaTexture) const {
    ProfilePhase p(Profiler::Stage::ShapeIntersectP);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute quadratic cylinder coefficients
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.x);
    auto a = SQ(dx) + SQ(dy);
    auto b = 2.0 * (dx * ox + dy * oy);
    auto c = SQ(ox) + SQ(oy) - SQ(EFloat(radius));

    // Solve quadratic equation for t values
    EFloat t0, t1;
    if (!solveQuadratic(a, b, c, &t0, &t1)) return false;
    if (t0.upperBound() > ray.tMax || t1.lowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.lowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.upperBound() > ray.tMax)
            return false;
    }

    // Compute cylinder hit position and phi
    Point3f pHit = ray(Float(tShapeHit));
    Float hitRadius = sqrt(SQ(pHit.x) + SQ(pHit.y));
    pHit.x *= radius / hitRadius;
    pHit.y *= radius / hitRadius;
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test cylinder intersection against clipping paramaters
    if (pHit.z < zMin || pHit.z > zMax || phi > phiMax) {
        if (tShapeHit == t1) return false;
        if (t1.upperBound() > ray.tMax) return false;
        tShapeHit = t1;

        Point3f pHit = ray(Float(tShapeHit));
        Float hitRadius = sqrt(SQ(pHit.x) + SQ(pHit.y));
        pHit.x *= radius / hitRadius;
        pHit.y *= radius / hitRadius;
        Float phi = atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * PI;

        if (pHit.z < zMin || pHit.z > zMax || phi > phiMax)
            return false;
    }

    return true;
}

shared_ptr<Shape> Cylinder::create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                   const ParamSet &params)
{
    Float radius = params.findOneFloat("radius", 1);
    Float zmin = params.findOneFloat("zmin", -1);
    Float zmax = params.findOneFloat("zmax", 1);
    Float phimax = params.findOneFloat("phimax", 360);
    return std::make_shared<Cylinder>(o2w, w2o, reverseOrientation, radius, zmin, zmax, phimax);
}
