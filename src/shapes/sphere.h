#ifndef SPHERE_H
#define SPHERE_H

#include "core/shape.h"

class Sphere : public Shape {
public:
    Sphere(const Transform *objToWorld, const Transform *worldToObj, bool revOrient,
           Float radius, Float zMin, Float zMax, Float phiMax)
        : Shape(objToWorld, worldToObj, revOrient), radius(radius),
          zMin(clamp(min(zMin, zMax), -radius, radius)),
          zMax(clamp(max(zMin, zMax), -radius, radius)),
          thetaMin(acos(clamp(zMin / radius, -1, 1))),
          thetaMax(acos(clamp(zMax / radius, -1, 1))),
          phiMax(radians(clamp(phiMax, 0, 360))) {}

    Bounds3f objectBound() const {
        return Bounds3f(Point3f(-radius, -radius, zMin), Point3f( radius, radius, zMax));
    }

    bool intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture = true) const;
    bool intersectP(const Ray &ray, bool testAlphaTexture = true) const;

    Float area() const { return phiMax * radius * (zMax - zMin); }

private:
    const Float radius;
    const Float zMin, zMax;
    const Float thetaMin, thetaMax; // polar
    const Float phiMax; // azimuth
};

#endif // SPHERE_H
