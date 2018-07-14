#ifndef SHAPE_CYLINDER
#define SHAPE_CYLINDER

#include "core/shape.h"

class Cylinder : public Shape {
public:
    Cylinder(const Transform *objToWorld, const Transform *worldToObj, bool revOrient, Float radius,
             Float zMin, Float zMax, Float phiMax)
        : Shape(objToWorld, worldToObj, revOrient), radius(radius),
          zMin(min(zMin, zMax)), zMax(max(zMin, zMax)), phiMax(radians(clamp(phiMax, 0, 360))) {}

    static shared_ptr<Shape> create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                    const ParamSet &params);

    Bounds3f objectBound() const {
           return Bounds3f(Point3f(-radius, -radius, zMin), Point3f( radius,  radius, zMax));
    }

    bool intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture = true) const;
    bool intersectP(const Ray &ray, bool testAlphaTexture = true) const;

    Float area() const { return phiMax * radius * (zMax - zMin); }

private:
    Float radius, zMin, zMax, phiMax;
};

#endif // SHAPE_CYLINDER
