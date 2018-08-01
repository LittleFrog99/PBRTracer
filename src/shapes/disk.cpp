#include "disk.h"
#include "stats.h"
#include "paramset.h"
#include "efloat.h"
#include "core/sampling.h"

bool Disk::intersect(const Ray &worldRay, float *tHit, SurfaceInteraction *isect,
                     bool testAlphaTexture) const
{
    ProfilePhase p(Stage::ShapeIntersect);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute plane intersection for disk
    if (ray.d.z == 0) return false;
    float tShapeHit = (height - ray.o.z) / ray.d.z;
    if (tShapeHit <= 0 || tShapeHit >= ray.tMax) return false;

    // See if hit point is in inside disk radii and phiMax
    Point3f pHit = ray(tShapeHit);
    float distSq = SQ(pHit.x) + SQ(pHit.y);
    if (distSq > SQ(radius) || distSq < SQ(innerRadius)) return false;
    float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;
    if (phi > phiMax) return false;

    // Find parametric representation of disk hit
    float u = phi / phiMax;
    float rHit = sqrt(distSq);
    float v = (radius - rHit) / (radius - innerRadius);

    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
    Vector3f dpdv = Vector3f(pHit.x, pHit.y, 0) * (innerRadius - radius) / rHit;
    Normal3f dndu(0, 0, 0), dndv(0, 0, 0);

    // Compute error bounds for cylinder intersection
    Vector3f pError(0, 0, 0);

    // Initialize $SurfaceInteraction from parametric information
    *isect = (*objectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d,
                                                 dpdu, dpdv, dndu, dndv, ray.time, this));

    // Update $tHit for quadratic intersection
    *tHit = float(tShapeHit);

    return true;
}

bool Disk::intersectP(const Ray &worldRay, bool testAlphaTexture) const  {
    ProfilePhase p(Stage::ShapeIntersect);

    // Tranform Ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute plane intersection for disk
    if (ray.d.z == 0) return false;
    float tShapeHit = (height - ray.o.z) / ray.d.z;
    if (tShapeHit <= 0 || tShapeHit >= ray.tMax) return false;

    // See if hit point is in inside disk radii and phiMax
    Point3f pHit = ray(tShapeHit);
    float distSq = SQ(pHit.x) + SQ(pHit.y);
    if (distSq > SQ(radius) || distSq < SQ(innerRadius)) return false;
    float phi = atan2(pHit.y, pHit.x);
    if (phi < 0) phi += 2 * PI;
    if (phi > phiMax) return false;

    return true;
}

Interaction Disk::sample(const Point2f &u, float *pdf) const {
    Point2f pd = Sampling::concentricSampleDisk(u);
    Point3f pObj(pd.x * radius, pd.y * radius, height);
    Interaction it;
    it.n = normalize((*objectToWorld)(Normal3f(0, 0, 1)));
    if (reverseOrientation) it.n *= -1;
    it.p = (*objectToWorld)(pObj, Vector3f(), &it.pError);
    *pdf = this->pdf(it);
    return it;
}

shared_ptr<Shape> Disk::create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                               const ParamSet &params)
{
    float height = params.findOneFloat("height", 0.);
    float radius = params.findOneFloat("radius", 1);
    float inner_radius = params.findOneFloat("innerradius", 0);
    float phimax = params.findOneFloat("phimax", 360);
    return std::make_shared<Disk>(o2w, w2o, reverseOrientation, height, radius, inner_radius, phimax);
}
