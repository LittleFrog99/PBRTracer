#include "curve.h"
#include "stats.h"
#include "paramset.h"

Bounds3f Curve::objectBound() const {
    Point3f ctrlPts[4];
    getContrlPoints(ctrlPts);
    return unionOf(Bounds3f(ctrlPts[0], ctrlPts[1]), Bounds3f(ctrlPts[2], ctrlPts[3]));
}

Point3f Curve::blossomBezier(float u0, float u1, float u2) const {
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

Point3f Curve::evaluateBezier(const Point3f cp[4], float u, Vector3f *deriv) {
       Point3f cp1[3] = { lerp(u, cp[0], cp[1]), lerp(u, cp[1], cp[2]),
                          lerp(u, cp[2], cp[3]) };
       Point3f cp2[2] = { lerp(u, cp1[0], cp1[1]), lerp(u, cp1[1], cp1[2]) };
       if (deriv) *deriv = 3 * (cp2[1] - cp2[0]);
       return lerp(u, cp2[0], cp2[1]);
   }

bool Curve::intersect(const Ray &worldRay, float *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const
{
    ProfilePhase p(isect ? Stage::CurveIntersect : Stage::CurveIntersectP);

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
    float L0 = 0;
    for (int i = 0; i < 2; ++i)
        L0 = max(L0, max(max(abs(cpRay[i].x - 2 * cpRay[i + 1].x + cpRay[i + 2].x),
                             abs(cpRay[i].y - 2 * cpRay[i + 1].y + cpRay[i + 2].y)),
                         abs(cpRay[i].z - 2 * cpRay[i + 1].z + cpRay[i + 2].z)));

    float eps = max(common->width[0], common->width[1]) * .05f;  // width / 20
    auto log2 = [](float v) -> int {
        if (v < 1) return 0;
        uint32_t bits = floatToBits(v);
        return (bits >> 23) - 127 + (bits & (1 << 22) ? 1 : 0);
    };
    // Compute log base 4 by dividing log2 in half.
    int r0 = log2(SQRT_TWO * 6.f * L0 / (8.f * eps)) / 2;
    int maxDepth = clamp(r0, 0, 10);

    return recursiveIntersect(ray, tHit, isect, cpRay, objToRay.inverse(), uMin, uMax, maxDepth);
}

bool Curve::recursiveIntersect(const Ray &ray, float *tHit, SurfaceInteraction *isect, const Point3f cp[4],
                               const Transform &rayToObject, float u0, float u1, int depth) const
{
    // Try to cull curve segment versus ray
    Bounds3f  curveBounds = unionOf(Bounds3f(cp[0], cp[1]), Bounds3f(cp[2], cp[3]));
    float maxWidth = max(lerp(u0, common->width[0], common->width[1]), lerp(u1, common->width[0], common->width[1]));
    curveBounds = expand(curveBounds, 0.5 * maxWidth);
    float rayLength = ray.d.length();
    float zMax = rayLength * ray.tMax;
    Bounds3f rayBounds(Point3f(0, 0, 0), Point3f(0, 0, zMax));
    if (overlaps(curveBounds, rayBounds) == false) return false;

    if (depth > 0) {
        // Split curve segments into sub-segments and test for intersection
        float uMid = 0.5 * (u0 + u1);
        Point3f cpSplit[7];
        subdivideBezier(cp, cpSplit);
        return recursiveIntersect(ray, tHit, isect, &cpSplit[0], rayToObject, u0, uMin, depth - 1) ||
                recursiveIntersect(ray, tHit, isect, &cpSplit[3], rayToObject, uMid, u1, depth - 1);
    } else {
        // Intersect ray with curve segment
        // Test ray against segment endpoint boundaries
        float edge = (cp[1].y - cp[0].y) * -cp[0].y + cp[0].x * (cp[0].x - cp[1].x); // start point
        if (edge < 0) return false;
        edge = (cp[2].y - cp[3].y) * -cp[3].y + cp[3].x * (cp[3].x - cp[2].x); // end point
        if (edge < 0) return false;

        // Compute minimum distance to sample point
        Vector2f segDir = Point2f(cp[3]) - Point2f(cp[0]);
        float denom = segDir.lengthSq();
        if (denom == 0) return false;
        float w = dot(-Vector2f(cp[0]), segDir) / denom; // the relative position on the sub-segment

        // Compute u coordinate of curve intersection point and $hitWidth
        float u = clamp(lerp(w, u0, u1), u0, u1); // position on the whole spline
        float hitWidth = lerp(u, common->width[0], common->width[1]);
        Normal3f nHit;
        if (common->type == CurveType::Ribbon) {
            float sin0 = sin((1 - u) * common->normalAngle) * common->invSinNormalAngle; // slerp
            float sin1 = sin(u * common->normalAngle) * common->invSinNormalAngle;
            nHit = sin0 * common->n[0] + sin1 * common->n[1];
            hitWidth *= absDot(nHit, ray.d) / rayLength;
        }

        // Test intersection point against curve width
        Vector3f dpcdw; // pc === point on the curve
        Point3f pc = evaluateBezier(cp, clamp(w, 0, 1), &dpcdw);
        float ptCurveDist2 = Vector2f(pc).lengthSq();
        if (ptCurveDist2 > 0.25 * SQ(hitWidth)) return false;
        if (pc.z < 0 || pc.z > zMax) return false;

        // Compute v coordinate of curve intersection point
        float ptCurveDist = sqrt(ptCurveDist2);
        float edgeFunc = dpcdw.x * -pc.y + pc.x * dpcdw.y;
        float v = (edgeFunc > 0) ? 0.5f + ptCurveDist / hitWidth : 0.5f - ptCurveDist / hitWidth;

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
                    float theta = lerp(v, -90.0, 90.0);
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

float Curve::area() const {
    Point3f ctrlPts[4];
    getContrlPoints(ctrlPts);
    float width0 = lerp(uMin, common->width[0], common->width[1]);
    float width1 = lerp(uMax, common->width[0], common->width[1]);
    float avgWidth = 0.5 * (width0 + width1);
    float approxLength = 0;
    for (unsigned i = 0; i < 3; i++)
        approxLength += distance(ctrlPts[i], ctrlPts[i + 1]);
    return approxLength * avgWidth;
}

vector<shared_ptr<Shape>> Curve::create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                        const Point3f *c, float w0, float w1, CurveType type,
                                        const Normal3f *norm, int splitDepth)
{
    std::vector<std::shared_ptr<Shape>> segments;
    std::shared_ptr<CurveCommon> common =
        std::make_shared<CurveCommon>(c, w0, w1, type, norm);
    const int nSegments = 1 << splitDepth;
    segments.reserve(nSegments);
    for (int i = 0; i < nSegments; ++i) {
        float uMin = i / (float)nSegments;
        float uMax = (i + 1) / (float)nSegments;
        segments.push_back(std::make_shared<Curve>(o2w, w2o, reverseOrientation, common, uMin, uMax));
    }
    return segments;
}

vector<shared_ptr<Shape>> Curve::create(const Transform *o2w, const Transform *w2o, bool reverseOrientation,
                                        const ParamSet &params)
{
    float width = params.findOneFloat("width", 1.f);
    float width0 = params.findOneFloat("width0", width);
    float width1 = params.findOneFloat("width1", width);

    int degree = params.findOneInt("degree", 3);
    if (degree != 2 && degree != 3) {
        ERROR("Invalid degree %d: only degree 2 and 3 curves are supported.",
              degree);
        return {};
    }

    std::string basis = params.findOneString("basis", "bezier");
    if (basis != "bezier" && basis != "bspline") {
        ERROR("Invalid basis \"%s\": only \"bezier\" and \"bspline\" are "
              "supported.", basis.c_str());
        return {};
    }

    int ncp;
    const Point3f *cp = params.findPoint3f("P", &ncp);
    int nSegments;
    if (basis == "bezier") {
        // After the first segment, which uses degree+1 control points,
        // subsequent segments reuse the last control point of the previous
        // one and then use degree more control points.
        if (((ncp - 1 - degree) % degree) != 0) {
            ERROR("Invalid number of control points %d: for the degree %d "
                  "Bezier basis %d + n * %d are required, for n >= 0.", ncp,
                  degree, degree + 1, degree);
            return {};
        }
        nSegments = (ncp - 1) / degree;
    } else {
        if (ncp < degree + 1) {
            ERROR("Invalid number of control points %d: for the degree %d "
                  "b-spline basis, must have >= %d.", ncp, degree, degree + 1);
            return {};
        }
        nSegments = ncp - degree;
    }


    CurveType type;
    std::string curveType = params.findOneString("type", "flat");
    if (curveType == "flat")
        type = CurveType::Flat;
    else if (curveType == "ribbon")
        type = CurveType::Ribbon;
    else if (curveType == "cylinder")
        type = CurveType::Cylinder;
    else {
        ERROR("Unknown curve type \"%s\".  Using \"flat\".", curveType.c_str());
        type = CurveType::Cylinder;
    }

    int nnorm;
    const Normal3f *n = params.findNormal3f("N", &nnorm);
    if (n != nullptr) {
        if (type != CurveType::Ribbon) {
            WARNING("Curve normals are only used with \"ribbon\" type curves.");
            n = nullptr;
        } else if (nnorm != nSegments + 1) {
            ERROR("Invalid number of normals %d: must provide %d normals for ribbon "
                  "curves with %d segments.", nnorm, nSegments + 1, nSegments);
            return {};
        }
    } else if (type == CurveType::Ribbon) {
        ERROR("Must provide normals \"N\" at curve endpoints with ribbon curves.");
        return {};
    }

    int sd = params.findOneInt("splitdepth",
                               int(params.findOneFloat("splitdepth", 3)));

    std::vector<std::shared_ptr<Shape>> curves;
    // Pointer to the first control point for the current segment. This is
    // updated after each loop iteration depending on the current basis.
    const Point3f *cpBase = cp;
    for (int seg = 0; seg < nSegments; ++seg) {
        Point3f segCpBezier[4];

        // First, compute the cubic Bezier control points for the current
        // segment and store them in segCpBezier. (It is admittedly
        // wasteful storage-wise to turn b-splines into Bezier segments and
        // wasteful computationally to turn quadratic curves into cubics,
        // but yolo.)
        if (basis == "bezier") {
            if (degree == 2) {
                // Elevate to degree 3.
                segCpBezier[0] = cpBase[0];
                segCpBezier[1] = lerp(2.f/3.f, cpBase[0], cpBase[1]);
                segCpBezier[2] = lerp(1.f/3.f, cpBase[1], cpBase[2]);
                segCpBezier[3] = cpBase[2];
            } else {
                // Allset.
                for (int i = 0; i < 4; ++i)
                    segCpBezier[i] = cpBase[i];
            }
            cpBase += degree;
        } else {
            // Uniform b-spline.
            if (degree == 2) {
                // First compute equivalent Bezier control points via some
                // blossiming.  We have three control points and a uniform
                // knot vector; we'll label the points p01, p12, and p23.
                // We want the Bezier control points of the equivalent
                // curve, which are p11, p12, and p22.
                Point3f p01 = cpBase[0];
                Point3f p12 = cpBase[1];
                Point3f p23 = cpBase[2];

                // We already have p12.
                Point3f p11 = lerp(0.5, p01, p12);
                Point3f p22 = lerp(0.5, p12, p23);

                // Now elevate to degree 3.
                segCpBezier[0] = p11;
                segCpBezier[1] = lerp(2.f/3.f, p11, p12);
                segCpBezier[2] = lerp(1.f/3.f, p12, p22);
                segCpBezier[3] = p22;
            } else {
                // Otherwise we will blossom from p012, p123, p234, and p345
                // to the Bezier control points p222, p223, p233, and p333.
                // https://people.eecs.berkeley.edu/~sequin/CS284/IMGS/cubicbsplinepoints.gif
                Point3f p012 = cpBase[0];
                Point3f p123 = cpBase[1];
                Point3f p234 = cpBase[2];
                Point3f p345 = cpBase[3];

                Point3f p122 = lerp(2.f/3.f, p012, p123);
                Point3f p223 = lerp(1.f/3.f, p123, p234);
                Point3f p233 = lerp(2.f/3.f, p123, p234);
                Point3f p334 = lerp(1.f/3.f, p234, p345);

                Point3f p222 = lerp(0.5f, p122, p223);
                Point3f p333 = lerp(0.5f, p233, p334);

                segCpBezier[0] = p222;
                segCpBezier[1] = p223;
                segCpBezier[2] = p233;
                segCpBezier[3] = p333;
            }
            ++cpBase;
        }

        auto c = create(o2w, w2o, reverseOrientation, segCpBezier, lerp(float(seg) / float(nSegments), width0, width1),
                        lerp(float(seg + 1) / float(nSegments), width0, width1), type, n ? &n[seg] : nullptr, sd);
        curves.insert(curves.end(), c.begin(), c.end());
    }

    return curves;
}
