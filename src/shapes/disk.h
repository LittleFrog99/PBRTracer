#ifndef DISK_H
#define DISK_H

#include "core/shape.h"

class Disk : public Shape {
public:
    Disk(const Transform *objToWorld, const Transform *worldToObj, bool revOrient,
         Float height, Float radius, Float innerRadius, Float phiMax)
        : Shape(objToWorld, worldToObj, revOrient),
          height(height), radius(radius), innerRadius(innerRadius),
          phiMax(radians(clamp(phiMax, 0, 360)))
    {}

    Bounds3f objectBound() const {
           return Bounds3f(Point3f(-radius, -radius, height), Point3f( radius,  radius, height));
    }

    bool intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture = true) const;
    bool intersectP(const Ray &ray, bool testAlphaTexture = true) const;

    Float area() const { return 0.5 * phiMax * (SQ(radius) - SQ(innerRadius)); }

private:
    Float height, radius, innerRadius, phiMax;
};

#endif // DISK_H
