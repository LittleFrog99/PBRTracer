#include "curve.h"
#include "core/stats.h"

Bounds3f Curve::objectBound() const {
    Point3f ctrlPts[4];
    getContrlPoints(ctrlPts);
    return unionOf(Bounds3f(ctrlPts[0], ctrlPts[1]), Bounds3f(ctrlPts[2], ctrlPts[3]));
}

Point3f Curve::blossomBezier(Float u0, Float u1, Float u2) const {
    const Point3f *p = common->ctrlPts;
    Point3f a[3] = { lerp(u0, p[0], p[1]), lerp(u0, p[1], p[2]), lerp(u0, p[2], p[3]) };
    Point3f b[2] = { lerp(u1, a[0], a[1]), lerp(u1, a[1], a[2]) };
    return lerp(u2, b[0], b[1]);
}

void Curve::getContrlPoints(Point3f cp[4]) const {
    cp[0] = blossomBezier(uMin, uMin, uMin);
    cp[1] = blossomBezier(uMin, uMin, uMax);
    cp[2] = blossomBezier(uMin, uMax, uMax);
    cp[3] = blossomBezier(uMax, uMax, uMax);
}

void Curve::subdivideBezier(const Point3f cp[4], Point3f cpSplit[7]) {
    cpSplit[0] = cp[0];
    cpSplit[1] = (cp[0] + cp[1]) / 2;
    cpSplit[2] = (cp[0] + 2 * cp[1] + cp[2]) / 4;
    cpSplit[3] = (cp[0] + 3 * cp[1] + 3 * cp[2] + cp[3]) / 8;
    cpSplit[4] = (cp[1] + 2 * cp[2] + cp[3]) / 4;
    cpSplit[5] = (cp[2] + cp[3]) / 2;
    cpSplit[6] = cp[3];
}

Point3f Curve::evaluateBezier(const Point3f cp[4], Float u, Vector3f *deriv) {
       Point3f cp1[3] = { lerp(u, cp[0], cp[1]), lerp(u, cp[1], cp[2]),
                          lerp(u, cp[2], cp[3]) };
       Point3f cp2[2] = { lerp(u, cp1[0], cp1[1]), lerp(u, cp1[1], cp1[2]) };
       if (deriv) *deriv = 3 * (cp2[1] - cp2[0]);
       return lerp(u, cp2[0], cp2[1]);
   }

bool Curve::intersect(const Ray &worldRay, Float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const
{
    ProfilePhase p(isect ? Profiler::Stage::CurveIntersect : Profiler::Stage::CurveIntersectP);

    // Transform ray to object space
    Vector3f oErr, dErr;
    Ray ray = (*worldToObject)(worldRay, &oErr, &dErr);

    // Compute object space control points for curve segment
    Point3f cpObj[4];
    getContrlPoints(cpObj);

    // Project curve control points to plane perpendicular to ray
    Vector3f dx, dy;
    coordSystem(ray.d, &dx, &dy);
    Transform objToRay = Transform::lookAt(ray.o, ray.o + ray.d, dx);
    Point3f cpRay[4] = { objToRay(cpObj[0]), objToRay(cpObj[1]), objToRay(cpObj[2]), objToRay(cpObj[3]) };

    // Compute refinement depth for curve
    Float L0 = 0;
    for (int i = 0; i < 2; ++i)
        L0 = max(L0, max(max(abs(cpRay[i].x - 2 * cpRay[i + 1].x + cpRay[i + 2].x),
                             abs(cpRay[i].y - 2 * cpRay[i + 1].y + cpRay[i + 2].y)),
                         abs(cpRay[i].z - 2 * cpRay[i + 1].z + cpRay[i + 2].z)));

    Float eps = max(common->width[0], common->width[1]) * .05f;  // width / 20
    auto log2 = [](Float v) -> int {
        if (v < 1) return 0;
        uint32_t bits = floatToBits(v);
        return (bits >> 23) - 127 + (bits & (1 << 22) ? 1 : 0);
    };
    // Compute log base 4 by dividing log2 in half.
    int r0 = log2(SQRT_TWO * 6.f * L0 / (8.f * eps)) / 2;
    int maxDepth = clamp(r0, 0, 10);

    return recursiveIntersect(ray, tHit, isect, cpRay, objToRay.inverse(), uMin, uMax, maxDepth);
}

bool Curve::recursiveIntersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect, const Point3f cp[4],
                               const Transform &rayToObject, Float u0, Float u1, int depth) const
{
    // Try to cull curve segment versus ray
    Bounds3f  curveBounds = unionOf(Bounds3f(cp[0], cp[1]), Bounds3f(cp[2], cp[3]));
    Float maxWidth = max(lerp(u0, common->width[0], common->width[1]), lerp(u1, common->width[0], common->width[1]));
    curveBounds = expand(curveBounds, 0.5 * maxWidth);
    Float rayLength = ray.d.length();
    Float zMax = rayLength * ray.tMax;
    Bounds3f rayBounds(Point3f(0, 0, 0), Point3f(0, 0, zMax));
    if (overlaps(curveBounds, rayBounds) == false) return false;

    if (depth > 0) {
        // Split curve segments into sub-segments and test for intersection
        Float uMid = 0.5 * (u0 + u1);
        Point3f cpSplit[7];
        subdivideBezier(cp, cpSplit);
        return recursiveIntersect(ray, tHit, isect, &cpSplit[0], rayToObject, u0, uMin, depth - 1) ||
                recursiveIntersect(ray, tHit, isect, &cpSplit[3], rayToObject, uMid, u1, depth - 1);
    } else {
        // Intersect ray with curve segment
        // Test ray against segment endpoint boundaries
        Float edge = (cp[1].y - cp[0].y) * -cp[0].y + cp[0].x * (cp[0].x - cp[1].x); // start point
        if (edge < 0) return false;
        edge = (cp[2].y - cp[3].y) * -cp[3].y + cp[3].x * (cp[3].x - cp[2].x); // end point
        if (edge < 0) return false;

        // Compute minimum distance to sample point
        Vector2f segDir = Point2f(cp[3]) - Point2f(cp[0]);
        Float denom = segDir.lengthSq();
        if (denom == 0) return false;
        Float w = dot(-Vector2f(cp[0]), segDir) / denom; // the relative position on the sub-segment

        // Compute u coordinate of curve intersection point and $hitWidth
        Float u = clamp(lerp(w, u0, u1), u0, u1); // position on the whole spline
        Float hitWidth = lerp(u, common->width[0], common->width[1]);
        Normal3f nHit;
        if (common->type == CurveType::Ribbon) {
            Float sin0 = sin((1 - u) * common->normalAngle) * common->invSinNormalAngle; // slerp
            Float sin1 = sin(u * common->normalAngle) * common->invSinNormalAngle;
            nHit = sin0 * common->n[0] + sin1 * common->n[1];
            hitWidth *= absDot(nHit, ray.d) / rayLength;
        }

        // Test intersection point against curve width
        Vector3f dpcdw; // pc === point on the curve
        Point3f pc = evaluateBezier(cp, clamp(w, 0, 1), &dpcdw);
        Float ptCurveDist2 = Vector2f(pc).lengthSq();
        if (ptCurveDist2 > 0.25 * SQ(hitWidth)) return false;
        if (pc.z < 0 || pc.z > zMax) return false;

        // Compute v coordinate of curve intersection point
        Float ptCurveDist = sqrt(ptCurveDist2);
        Float edgeFunc = dpcdw.x * -pc.y + pc.x * dpcdw.y;
        Float v = (edgeFunc > 0) ? 0.5f + ptCurveDist / hitWidth : 0.5f - ptCurveDist / hitWidth;

        // Compute hit t and partial derivatives for curve intersection
        if (tHit) {
            *tHit = pc.z / rayLength;
            Vector3f pError(2 * hitWidth, 2 * hitWidth, 2 * hitWidth);
            Vector3f dpdu, dpdv;
            evaluateBezier(common->ctrlPts, u, &dpdu);
            if (common->type == CurveType::Ribbon)
                dpdv = normalize(cross(nHit, dpdu)) * hitWidth;
            else {
                auto dpduPlane = rayToObject.inverse()(dpdu);
                auto dpdvPlane = normalize(Vector3f(-dpduPlane.y, dpduPlane.x, 0)) * hitWidth;
                if (common->type == CurveType::Cylinder) {
                    Float theta = lerp(v, -90.0, 90.0);
                    auto rot = Transform::rotate(theta, dpduPlane);
                    dpdvPlane = rot(dpdvPlane);
                }
                dpdv = rayToObject(dpdvPlane);
            }
            *isect = (*objectToWorld)(SurfaceInteraction(ray(pc.z), pError, Point2f(u, v), -ray.d, dpdu, dpdv,
                                                         Normal3f(), Normal3f(), ray.time, this));
        }
        return true;
    }
}

Float Curve::area() const {
    Point3f ctrlPts[4];
    getContrlPoints(ctrlPts);
    Float width0 = lerp(uMin, common->width[0], common->width[1]);
    Float width1 = lerp(uMax, common->width[0], common->width[1]);
    Float avgWidth = 0.5 * (width0 + width1);
    Float approxLength = 0;
    for (unsigned i = 0; i < 3; i++)
        approxLength += distance(ctrlPts[i], ctrlPts[i + 1]);
    return approxLength * avgWidth;
}


