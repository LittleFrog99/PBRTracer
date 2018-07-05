#include "sphere.h"
#include "core/stats.h"
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
    if (!solveQuadratic(a, b, c, &t0, &t1)) return false;
    if (t0.upperBound() > ray.tMax || t1.lowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.lowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.upperBound() > ray.tMax)
            return false;
    }

    // Compute sphere hit position and phi(azimuth)
    Point3f pHit = ray(Float(tShapeHit));
    pHit *= radius / distance(pHit, Point3f(0, 0, 0));
    if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius; // why this?
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test sphere intersection against clipping paramaters
    if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) // t0
            || phi > phiMax) {
        if (tShapeHit == t1) return false;
        if (t1.upperBound() > ray.tMax) return false;
        tShapeHit = t1;

        Point3f pHit = ray(Float(tShapeHit));
        pHit *= radius / distance(pHit, Point3f(0, 0, 0));
        if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
        Float phi = atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * PI;

        if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) // again with t1
                || phi > phiMax)
            return false;
    }

    // Find parametric representation of sphere hit
    Float u = phi / phiMax;
    Float theta = acos(clamp(pHit.z / radius, -1.0, 1.0));
    Float v = (theta - thetaMin) / (thetaMax - thetaMin);
    Float zRadius = sqrt(SQ(pHit.x) + SQ(pHit.y));
    Float invZRadius = 1.0 / zRadius;
    Float cosPhi = pHit.x * invZRadius;
    Float sinPhi = pHit.y * invZRadius;

    auto dpdu = Vector3f(-phiMax * pHit.y, phiMax * pHit.x, 0);
    auto dpdv = (thetaMax - thetaMin) * Vector3f(pHit.z * cosPhi, pHit.z * sinPhi, -radius * sin(theta));
    auto d2pduu = -SQ(phiMax) * Vector3f(pHit.x, pHit.y, 0);
    auto d2pduv = (thetaMax - thetaMin) * pHit.z * phiMax * Vector3f(-sinPhi, cosPhi, 0);
    auto d2pdvv = -SQ(thetaMax - thetaMin) * Vector3f(pHit);
    Float E = dot(dpdu, dpdu), F = dot(dpdu, dpdv), G = dot(dpdv, dpdv);
    Vector3f N = normalize(cross(dpdu, dpdv));
    Float e = dot(N, d2pduu), f = dot(N, d2pduv), g = dot(N, d2pdvv);
    Float invEGF2 = 1.0 / (E * G - F * F);
    auto dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu + (e * F - f * E) * invEGF2 * dpdv);
    auto dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu + (f * F - g * E) * invEGF2 * dpdv);

    // Compute error bounds for sphere intersection
    Vector3f pError = gamma(5) * abs(Vector3f(pHit));

    // Initialize $SurfaceInteraction from parametric information
    *isect = (*objectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d,
                                                 dpdu, dpdv, dndu, dndv, ray.time, this));

    // Update $tHit for quadratic intersection
    *tHit = Float(tShapeHit);

    return true;
}

bool Sphere::intersectP(const Ray &worldRay, bool testAlphaTexture) const {
    ProfilePhase p(Profiler::Stage::ShapeIntersectP);

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

    // Compute sphere hit position and phi(azimuth)
    Point3f pHit = ray(Float(tShapeHit));
    pHit *= radius / distance(pHit, Point3f(0, 0, 0));
    if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius; // why this?
    Float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test sphere intersection against clipping paramaters
    if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) // t0
            || phi > phiMax) {
        if (tShapeHit == t1) return false;
        if (t1.upperBound() > ray.tMax) return false;
        tShapeHit = t1;

        Point3f pHit = ray(Float(tShapeHit));
        pHit *= radius / distance(pHit, Point3f(0, 0, 0));
        if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
        Float phi = atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * PI;

        if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) // again with t1
                || phi > phiMax)
            return false;
    }

    return true;
}
