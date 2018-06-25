#include "transform.h"
#include "report.h"
#include "core/interaction.h"

Transform Transform::translate(const Vector3f &delta) {
    Matrix4x4 m(1, 0, 0, delta.x, 0, 1, 0, delta.y, 0, 0, 1, delta.z, 0, 0, 0,
                1);
    Matrix4x4 minv(1, 0, 0, -delta.x, 0, 1, 0, -delta.y, 0, 0, 1, -delta.z, 0,
                   0, 0, 1);
    return Transform(m, minv);
}

Transform Transform::scale(Float x, Float y, Float z) {
    Matrix4x4 m(x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1);
    Matrix4x4 minv(1 / x, 0, 0, 0, 0, 1 / y, 0, 0, 0, 0, 1 / z, 0, 0, 0, 0, 1);
    return Transform(m, minv);
}

Transform Transform::rotateX(Float theta) {
    Float sinTheta = sin(radians(theta));
    Float cosTheta = cos(radians(theta));
    Matrix4x4 m(1, 0, 0, 0, 0, cosTheta, -sinTheta, 0, 0, sinTheta, cosTheta, 0,
                0, 0, 0, 1);
    return Transform(m, m.transpose());
}

Transform Transform::rotateY(Float theta) {
    Float sinTheta = sin(radians(theta));
    Float cosTheta = cos(radians(theta));
    Matrix4x4 m(cosTheta, 0, sinTheta, 0, 0, 1, 0, 0, -sinTheta, 0, cosTheta, 0,
                0, 0, 0, 1);
    return Transform(m, m.transpose());
}

Transform Transform::rotateZ(Float theta) {
    Float sinTheta = sin(radians(theta));
    Float cosTheta = cos(radians(theta));
    Matrix4x4 m(cosTheta, -sinTheta, 0, 0, sinTheta, cosTheta, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 1);
    return Transform(m, m.transpose());
}

Transform Transform::rotate(Float theta, const Vector3f &axis) {
    Vector3f a = normalize(axis);
    Float sinTheta = sin(radians(theta));
    Float cosTheta = cos(radians(theta));

    Matrix4x4 m;
    m.m[0][0] = a.x * a.x + (1 - a.x * a.x) * cosTheta;
    m.m[0][1] = a.x * a.y * (1 - cosTheta) - a.z * sinTheta;
    m.m[0][2] = a.x * a.z * (1 - cosTheta) + a.y * sinTheta;
    m.m[1][0] = a.x * a.y * (1 - cosTheta) + a.z * sinTheta;
    m.m[1][1] = a.y * a.y + (1 - a.y * a.y) * cosTheta;
    m.m[1][2] = a.y * a.z * (1 - cosTheta) - a.x * sinTheta;
    m.m[2][0] = a.x * a.z * (1 - cosTheta) - a.y * sinTheta;
    m.m[2][1] = a.y * a.z * (1 - cosTheta) + a.x * sinTheta;
    m.m[2][2] = a.z * a.z + (1 - a.z * a.z) * cosTheta;
    return Transform(m, m.transpose());
}

Transform Transform::lookAt(const Point3f &pos, const Point3f &look, const Vector3f &up) {
    Matrix4x4 cameraToWorld;

    cameraToWorld.m[0][3] = pos.x;
    cameraToWorld.m[1][3] = pos.y;
    cameraToWorld.m[2][3] = pos.z;
    cameraToWorld.m[3][3] = 1;

    Vector3f dir = normalize(look - pos);
    if (cross(normalize(up), dir).length() == 0) {
        ERROR(
            "\"up\" vector (%f, %f, %f) and viewing direction (%f, %f, %f) "
            "passed to LookAt are pointing in the same direction.  Using "
            "the identity transformation.",
            up.x, up.y, up.z, dir.x, dir.y, dir.z);
        return Transform();
    }
    Vector3f right = normalize(cross(normalize(up), dir));
    Vector3f newUp = cross(dir, right);
    cameraToWorld.m[0][0] = right.x;
    cameraToWorld.m[1][0] = right.y;
    cameraToWorld.m[2][0] = right.z;
    cameraToWorld.m[3][0] = 0.;
    cameraToWorld.m[0][1] = newUp.x;
    cameraToWorld.m[1][1] = newUp.y;
    cameraToWorld.m[2][1] = newUp.z;
    cameraToWorld.m[3][1] = 0.;
    cameraToWorld.m[0][2] = dir.x;
    cameraToWorld.m[1][2] = dir.y;
    cameraToWorld.m[2][2] = dir.z;
    cameraToWorld.m[3][2] = 0.;
    return Transform(cameraToWorld.inverse(), cameraToWorld);
}

Bounds3f Transform::operator()(const Bounds3f &b) const {
    const Transform &M = *this;
    Bounds3f ret(M(Point3f(b.pMin.x, b.pMin.y, b.pMin.z)));
    ret = unionOf(ret, M(Point3f(b.pMax.x, b.pMin.y, b.pMin.z)));
    ret = unionOf(ret, M(Point3f(b.pMin.x, b.pMax.y, b.pMin.z)));
    ret = unionOf(ret, M(Point3f(b.pMin.x, b.pMin.y, b.pMax.z)));
    ret = unionOf(ret, M(Point3f(b.pMin.x, b.pMax.y, b.pMax.z)));
    ret = unionOf(ret, M(Point3f(b.pMax.x, b.pMax.y, b.pMin.z)));
    ret = unionOf(ret, M(Point3f(b.pMax.x, b.pMin.y, b.pMax.z)));
    ret = unionOf(ret, M(Point3f(b.pMax.x, b.pMax.y, b.pMax.z)));
    return ret;
}

bool Transform::swapsHandedness() const {
    Float det = m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) -
                m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0]) +
                m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]);
    return det < 0;
}

SurfaceInteraction Transform::operator()(const SurfaceInteraction &si) const {
    SurfaceInteraction ret;
    // Transform _p_ and _pError_ in _SurfaceInteraction_
    ret.p = (*this)(si.p, si.pError, &ret.pError);

    // Transform remaining members of _SurfaceInteraction_
    const Transform &t = *this;
    ret.n = normalize(t(si.n));
    ret.wo = normalize(t(si.wo));
    ret.time = si.time;
    ret.mediumInterface = si.mediumInterface;
    ret.uv = si.uv;
    ret.shape = si.shape;
    ret.dpdu = t(si.dpdu);
    ret.dpdv = t(si.dpdv);
    ret.dndu = t(si.dndu);
    ret.dndv = t(si.dndv);
    ret.shading.n = normalize(t(si.shading.n));
    ret.shading.dpdu = t(si.shading.dpdu);
    ret.shading.dpdv = t(si.shading.dpdv);
    ret.shading.dndu = t(si.shading.dndu);
    ret.shading.dndv = t(si.shading.dndv);
    ret.dudx = si.dudx;
    ret.dvdx = si.dvdx;
    ret.dudy = si.dudy;
    ret.dvdy = si.dvdy;
    ret.dpdx = t(si.dpdx);
    ret.dpdy = t(si.dpdy);
    ret.bsdf = si.bsdf;
    ret.bssrdf = si.bssrdf;
    ret.primitive = si.primitive;
    ret.shading.n = faceforward(ret.shading.n, ret.n);
    // ret.faceIndex = si.faceIndex;
    return ret;
}

Transform Transform::orthographic(Float zNear, Float zFar) {
    return scale(1, 1, 1 / (zFar - zNear)) * translate(Vector3f(0, 0, -zNear));
}

Transform Transform::perspective(Float fov, Float n, Float f) {
    // Perform projective divide for perspective projection
    Matrix4x4 persp(1, 0, 0, 0, 0,
                    1, 0, 0, 0, 0,
                    f / (f - n), -f * n / (f - n),
                    0, 0, 1, 0);
    // Scale canonical perspective view to specified field of view
    Float invTanAng = 1 / tan(radians(fov) / 2);
    return scale(invTanAng, invTanAng, 1) * Transform(persp);
}
