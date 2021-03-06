#include "sphere.h"
#include "stats.h"
#include "paramset.h"
#include "efloat.h"
#include "core/sampling.h"

bool Sphere::intersect(const Ray &worldRay, float *tHit,SurfaceInteraction *isect,
                       bool testAlphaTexture) const
{
    ProfilePhase p(Stage::ShapeIntersect);
    float phi;
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute quadratic sphere coefficients

    // Initialize _EFloat_ ray coordinate values
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat a = dx * dx + dy * dy + dz * dz;
    EFloat b = 2 * (dx * ox + dy * oy + dz * oz);
    EFloat c = ox * ox + oy * oy + oz * oz - EFloat(radius) * EFloat(radius);

    // Solve quadratic equation for _t_ values
    EFloat t0, t1;
    if (!solveQuadratic(a, b, c, &t0, &t1)) return false;

    // Check quadric shape _t0_ and _t1_ for nearest intersection
    if (t0.upperBound() > ray.tMax || t1.lowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.lowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.upperBound() > ray.tMax) return false;
    }

    // Compute sphere hit position and $\phi$
    pHit = ray(float(tShapeHit));

    // Refine sphere intersection point
    pHit *= radius / distance(pHit, Point3f(0, 0, 0));
    if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
    phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test sphere intersection against clipping parameters
    if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) ||
        phi > phiMax) {
        if (tShapeHit == t1) return false;
        if (t1.upperBound() > ray.tMax) return false;
        tShapeHit = t1;
        // Compute sphere hit position and $\phi$
        pHit = ray(float(tShapeHit));

        // Refine sphere intersection point
        pHit *= radius / distance(pHit, Point3f(0, 0, 0));
        if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
        phi = atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * PI;
        if ((zMin > -radius && pHit.z < zMin) ||
            (zMax < radius && pHit.z > zMax) || phi > phiMax)
            return false;
    }

    // Find parametric representation of sphere hit
    float u = phi / phiMax;
    float theta = acos(clamp(pHit.z / radius, -1, 1));
    float v = (theta - thetaMin) / (thetaMax - thetaMin);

    // Compute sphere $\dpdu$ and $\dpdv$
    float zRadius = sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
    float invZRadius = 1 / zRadius;
    float cosPhi = pHit.x * invZRadius;
    float sinPhi = pHit.y * invZRadius;
    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
    Vector3f dpdv = (thetaMax - thetaMin) * Vector3f(pHit.z * cosPhi, pHit.z * sinPhi, -radius * sin(theta));

    // Compute sphere $\dndu$ and $\dndv$
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f(pHit.x, pHit.y, 0);
    Vector3f d2Pduv =
        (thetaMax - thetaMin) * pHit.z * phiMax * Vector3f(-sinPhi, cosPhi, 0.);
    Vector3f d2Pdvv = -(thetaMax - thetaMin) * (thetaMax - thetaMin) * Vector3f(pHit.x, pHit.y, pHit.z);

    // Compute coefficients for fundamental forms
    float E = dot(dpdu, dpdu);
    float F = dot(dpdu, dpdv);
    float G = dot(dpdv, dpdv);
    Vector3f N = normalize(cross(dpdu, dpdv));
    float e = dot(N, d2Pduu);
    float f = dot(N, d2Pduv);
    float g = dot(N, d2Pdvv);

    // Compute $\dndu$ and $\dndv$ from fundamental form coefficients
    float invEGF2 = 1 / (E * G - F * F);
    Normal3f dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu + (e * F - f * E) * invEGF2 * dpdv);
    Normal3f dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu + (f * F - g * E) * invEGF2 * dpdv);

    // Compute error bounds for sphere intersection
    Vector3f pError = gamma(5) * abs((Vector3f)pHit);

    // Initialize _SurfaceInteraction_ from parametric information
    *isect = (*objectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d, dpdu, dpdv, dndu, dndv,
                                                 ray.time, this));

    // Update _tHit_ for quadric intersection
    *tHit = float(tShapeHit);
    return true;
}

bool Sphere::intersectP(const Ray &worldRay, bool testAlphaTexture) const {
    ProfilePhase p(Stage::ShapeIntersectP);
    float phi;
    Point3f pHit;
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute quadratic sphere coefficients

    // Initialize _EFloat_ ray coordinate values
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat a = dx * dx + dy * dy + dz * dz;
    EFloat b = 2 * (dx * ox + dy * oy + dz * oz);
    EFloat c = ox * ox + oy * oy + oz * oz - EFloat(radius) * EFloat(radius);

    // Solve quadratic equation for _t_ values
    EFloat t0, t1;
    if (!solveQuadratic(a, b, c, &t0, &t1)) return false;

    // Check quadric shape _t0_ and _t1_ for nearest intersection
    if (t0.upperBound() > ray.tMax || t1.lowerBound() <= 0) return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.lowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.upperBound() > ray.tMax) return false;
    }

    // Compute sphere hit position and $\phi$
    pHit = ray(float(tShapeHit));

    // Refine sphere intersection point
    pHit *= radius / distance(pHit, Point3f(0, 0, 0));
    if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
    phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;

    // Test sphere intersection against clipping parameters
    if ((zMin > -radius && pHit.z < zMin) || (zMax < radius && pHit.z > zMax) ||
        phi > phiMax) {
        if (tShapeHit == t1) return false;
        if (t1.upperBound() > ray.tMax) return false;
        tShapeHit = t1;
        // Compute sphere hit position and $\phi$
        pHit = ray(float(tShapeHit));

        // Refine sphere intersection point
        pHit *= radius / distance(pHit, Point3f(0, 0, 0));
        if (pHit.x == 0 && pHit.y == 0) pHit.x = 1e-5f * radius;
        phi = atan2(pHit.y, pHit.x);
        if (phi < 0) phi += 2 * PI;
        if ((zMin > -radius && pHit.z < zMin) ||
            (zMax < radius && pHit.z > zMax) || phi > phiMax)
            return false;
    }
    return true;
}

Interaction Sphere::sample(const Point2f &u, float *pdf) const {
    Point3f pObj = Point3f() + radius * Sampling::uniformSampleSphere(u);
    Interaction it;
    it.n = normalize((*objectToWorld)(Normal3f(pObj.x, pObj.y, pObj.z)));
    if (reverseOrientation) it.n *= -1;
    pObj *= radius / distance(pObj, Point3f());
    Vector3f pObjError = gamma(5) * abs(Vector3f(pObj));
    it.p = (*objectToWorld)(pObj, pObjError, &it.pError);
    *pdf = Shape::pdf(it);
    return it;
}

Interaction Sphere::sample(const Interaction &ref, const Point2f &u, float *pdf) const {
    // Compute coordinate system for sphere sampling
    Point3f pCenter = (*objectToWorld)(Point3f());
    Vector3f wcZ = normalize(pCenter - ref.p);
    Vector3f wcX, wcY;
    coordinateSystem(wcZ, &wcX, &wcY);

    // Sample uniformly on sphere if p is inside it
    Point3f pOrigin = offsetRayOrigin(ref.p, ref.pError, ref.n, pCenter - ref.p);
    if (distanceSq(pOrigin, pCenter) <= SQ(radius))
        return sample(u, pdf);

    // Sample sphere uniformly inside subtended cone
    // Compute φ and θ for sample in cone
    float sinThetaMax2 = radius * radius / distanceSq(ref.p, pCenter);
    float cosThetaMax = sqrt(max(0.0f, 1 - sinThetaMax2));
    float cosTheta = (1 - u[0]) + u[0] * cosThetaMax;
    float sinTheta = sqrt(max(0.0f, 1 - cosTheta * cosTheta));
    float phi = u[1] * 2 * PI;

    // Compute angle α from center of sphere to sampled point on surface
    float dc = distance(ref.p, pCenter);
    float ds = dc * cosTheta - sqrt(max(0.0f, SQ(radius) - SQ(dc * sinTheta)));
    float cosAlpha = (SQ(dc) + SQ(radius) - SQ(ds)) / (2 * dc * radius);
    float sinAlpha = sqrt(max(0.0f, 1 - cosAlpha * cosAlpha));

    // Compute surface normal and sampled point on sphere
    Vector3f nWorld = sphericalDirection(sinAlpha, cosAlpha, phi, -wcX, -wcY, -wcZ);
    Point3f pWorld = pCenter + radius * Point3f(nWorld.x, nWorld.y, nWorld.z);

    // Return _Interaction_ for sampled point on sphere
    Interaction it;
    it.p = pWorld;
    it.pError = gamma(5) * abs(Vector3f(pWorld));
    it.n = Normal3f(nWorld);
    if (reverseOrientation) it.n *= -1;
    *pdf = 1 / (2 * PI * (1 - cosThetaMax));

    return it;
}

float Sphere::pdf(const Interaction &ref, const Vector3f &wi) const {
    Point3f pCenter = (*objectToWorld)(Point3f());
    Point3f pOrigin = offsetRayOrigin(ref.p, ref.pError, ref.n, pCenter - ref.p);
    if (distanceSq(pOrigin, pCenter) <= SQ(radius))
        return Shape::pdf(ref, wi);
    float sinThetaMax2 = SQ(radius) / distanceSq(ref.p, pCenter);
    float cosThetaMax = sqrt(max(0.0f, 1 - sinThetaMax2));
    return Sampling::uniformConePdf(cosThetaMax);
}

shared_ptr<Shape> Sphere::create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                 const ParamSet &params)
{
    float radius = params.findOneFloat("radius", 1.f);
    float zmin = params.findOneFloat("zmin", -radius);
    float zmax = params.findOneFloat("zmax", radius);
    float phimax = params.findOneFloat("phimax", 360.f);
    return make_shared<Sphere>(o2w, w2o, reverseOrientation, radius, zmin, zmax, phimax);
}
