#ifndef SHAPE_SPHERE
#define SHAPE_SPHERE

#include "core/shape.h"

class Sphere : public Shape {
public:
    Sphere(const Transform *objToWorld, const Transform *worldToObj, bool revOrient,
           float radius, float zMin, float zMax, float phiMax)
        : Shape(objToWorld, worldToObj, revOrient), radius(radius),
          zMin(clamp(min(zMin, zMax), -radius, radius)),
          zMax(clamp(max(zMin, zMax), -radius, radius)),
          thetaMin(acos(clamp(zMin / radius, -1, 1))),
          thetaMax(acos(clamp(zMax / radius, -1, 1))),
          phiMax(radians(clamp(phiMax, 0, 360))) {}

    static shared_ptr<Shape> create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                    const ParamSet &params);

    Bounds3f objectBound() const {
        return Bounds3f(Point3f(-radius, -radius, zMin), Point3f( radius, radius, zMax));
    }

    bool intersect(const Ray &r, float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture = true) const;
    bool intersectP(const Ray &ray, bool testAlphaTexture = true) const;

    float area() const { return phiMax * radius * (zMax - zMin); }

private:
    const float radius;
    const float zMin, zMax;
    const float thetaMin, thetaMax; // polar
    const float phiMax; // azimuth
};

#endif // SHAPE_SPHERE
