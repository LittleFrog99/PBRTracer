#ifndef SHAPE_BEZIER
#define SHAPE_BEZIER

#include "core/shape.h"

enum class CurveType {
    Flat, // always oriented to face the ray being intersected with them
    Cylinder, // will compute a shading normal to make it appear to be a cylinder
    Ribbon // has fixed orientation
};

struct CurveCommon {
    CurveCommon(const Point3f c[4], float width0, float width1, CurveType type, const Normal3f *norm)
        : type(type), ctrlPts{ c[0], c[1], c[2], c[3] }, width{ width0, width1 } {
        if (norm) {
            n[0] = normalize(norm[0]);
            n[1] = normalize(norm[1]);
            normalAngle = acos(clamp(dot(n[0], n[1]), 0, 1));
            invSinNormalAngle = 1.0 / sin(normalAngle);
        }
    }

    const CurveType type;
    const Point3f ctrlPts[4]; // in curve's object space
    const float width[2];
    Normal3f n[2];
    float normalAngle, invSinNormalAngle;
};

class Curve : public Shape { // Bezier basis curve
public:
    Curve(const Transform *objToWorld, const Transform *worldToObj, bool invOrient,
          const shared_ptr<CurveCommon> &common, float uMin, float uMax)
        : Shape(objToWorld, worldToObj, invOrient), common(common), uMin(uMin), uMax(uMax) {}

    static vector<shared_ptr<Shape>> create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                            const Point3f *c, float w0, float w1, CurveType type,
                                            const Normal3f *norm, int splitDepth);
    static vector<shared_ptr<Shape>> create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                            const ParamSet &params);


    Bounds3f objectBound() const;
    bool intersect(const Ray &r, float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const;
    float area() const;

private:
    Point3f blossomBezier(float u0, float u1, float u2) const;
    void getContrlPoints(Point3f cp[4]) const;
    static void subdivideBezier(const Point3f cp[4], Point3f cpSplit[7]);
    static Point3f evaluateBezier(const Point3f cp[4], float u, Vector3f *deriv = nullptr);

    bool recursiveIntersect(const Ray &ray, float *tHit, SurfaceInteraction *isect, const Point3f cp[4],
                            const Transform &rayToObject, float u0, float u1, int depth) const;

    shared_ptr<CurveCommon> common;
    float uMin, uMax;
};

#endif // SHAPE_BEZIER
