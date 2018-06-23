#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "bounds.h"
#include "matrix.h"
#include "quaternion.h"

class SurfaceInteraction;

class Transform {
public:
    Transform() {}
    Transform(const Float mat[4][4]) {
        m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3], mat[1][0],
                      mat[1][1], mat[1][2], mat[1][3], mat[2][0], mat[2][1],
                      mat[2][2], mat[2][3], mat[3][0], mat[3][1], mat[3][2],
                      mat[3][3]);
        mInv = m.inverse();
    }

    Transform(const Matrix4x4 &m) : m(m), mInv(m.inverse()) {}
    Transform(const Matrix4x4 &m, const Matrix4x4 &mInv) : m(m), mInv(mInv) {}

    static Transform translate(const Vector3f &delta);
    static Transform scale(Float x, Float y, Float z);
    static Transform rotateX(Float theta);
    static Transform rotateY(Float theta);
    static Transform rotateZ(Float theta);
    static Transform rotate(Float theta, const Vector3f &axis);
    static Transform lookAt(const Point3f &pos, const Point3f &look, const Vector3f &up);
    static Transform orthographic(Float znear, Float zfar);
    static Transform perspective(Float fov, Float znear, Float zfar);

    Transform inverse() const { return Transform(mInv, m); }
    Transform transpose() const { return Transform(m.transpose(), mInv.transpose()); }

    Transform operator * (const Transform &t2) const { return Transform(m * t2.m, t2.mInv * mInv); }
    bool operator == (const Transform &t) const { return t.m == m && t.mInv == mInv; }
    bool operator != (const Transform &t) const { return t.m != m || t.mInv != mInv; }
    bool operator < (const Transform &t2) const {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                if (m.m[i][j] < t2.m.m[i][j]) return true;
                if (m.m[i][j] > t2.m.m[i][j]) return false;
            }
        return false;
    }

    template <typename T>
    inline Point3<T> operator () (const Point3<T> &p) const;
    template <typename T>
    inline Vector3<T> operator () (const Vector3<T> &v) const;
    template <typename T>
    inline Normal3<T> operator () (const Normal3<T> &) const;
    inline Ray operator () (const Ray &r) const;
    inline RayDifferential operator () (const RayDifferential &r) const;
    Bounds3f operator () (const Bounds3f &b) const;
    SurfaceInteraction operator () (const SurfaceInteraction &si) const;

    template <typename T>
    inline Point3<T> operator () (const Point3<T> &pt, Vector3<T> *absError) const;
    template <typename T>
    inline Point3<T> operator () (const Point3<T> &p, const Vector3<T> &pError,
                                  Vector3<T> *pTransError) const;
    template <typename T>
    inline Vector3<T> operator () (const Vector3<T> &v, Vector3<T> *vTransError) const;
    template <typename T>
    inline Vector3<T> operator () (const Vector3<T> &v, const Vector3<T> &vError,
                                   Vector3<T> *vTransError) const;
    inline Ray operator () (const Ray &r, Vector3f *oError, Vector3f *dError) const;
    inline Ray operator () (const Ray &r, const Vector3f &oErrorIn, const Vector3f &dErrorIn,
                            Vector3f *oErrorOut, Vector3f *dErrorOut) const;

    friend ostream & operator << (ostream &os, const Transform &t) {
        os << "t=" << t.m << ", inv=" << t.mInv;
        return os;
    }

    void print(FILE *f) const { m.print(f); }

    bool isIdentity() const {
        return (m.m[0][0] == 1.f && m.m[0][1] == 0.f && m.m[0][2] == 0.f &&
                m.m[0][3] == 0.f && m.m[1][0] == 0.f && m.m[1][1] == 1.f &&
                m.m[1][2] == 0.f && m.m[1][3] == 0.f && m.m[2][0] == 0.f &&
                m.m[2][1] == 0.f && m.m[2][2] == 1.f && m.m[2][3] == 0.f &&
                m.m[3][0] == 0.f && m.m[3][1] == 0.f && m.m[3][2] == 0.f &&
                m.m[3][3] == 1.f);
    }

    const Matrix4x4 & getMatrix() const { return m; }
    const Matrix4x4 & getInverseMatrix() const { return mInv; }

    bool hasScale() const {
        Float la2 = (*this)(Vector3f(1, 0, 0)).lengthSq();
        Float lb2 = (*this)(Vector3f(0, 1, 0)).lengthSq();
        Float lc2 = (*this)(Vector3f(0, 0, 1)).lengthSq();
#define NOT_ONE(x) ((x) < .999f || (x) > 1.001f)
        return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
    }

    bool swapsHandedness() const;

private:
    Matrix4x4 m, mInv;
    friend class AnimatedTransform;
    friend struct Quaternion;
};

template <typename T>
inline Point3<T> Transform::operator()(const Point3<T> &p) const {
    T x = p.x, y = p.y, z = p.z;
    T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
    T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
    T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
    T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
    CHECK_NE(wp, 0);
    if (wp == 1)
        return Point3<T>(xp, yp, zp);
    else
        return Point3<T>(xp, yp, zp) / wp;
}

template <typename T>
inline Vector3<T> Transform::operator()(const Vector3<T> &v) const {
    T x = v.x, y = v.y, z = v.z;
    return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                      m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                      m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

template <typename T>
inline Normal3<T> Transform::operator()(const Normal3<T> &n) const {
    T x = n.x, y = n.y, z = n.z;
    return Normal3<T>(mInv.m[0][0] * x + mInv.m[1][0] * y + mInv.m[2][0] * z,
                      mInv.m[0][1] * x + mInv.m[1][1] * y + mInv.m[2][1] * z,
                      mInv.m[0][2] * x + mInv.m[1][2] * y + mInv.m[2][2] * z);
}

inline Ray Transform::operator()(const Ray &r) const {
    Vector3f oError;
    Point3f o = (*this)(r.o, &oError);
    Vector3f d = (*this)(r.d);
    // Offset ray origin to edge of error bounds and compute _tMax_
    Float lengthSquared = d.lengthSq();
    Float tMax = r.tMax;
    if (lengthSquared > 0) {
        Float dt = Math::dot(Math::abs(d), oError) / lengthSquared;
        o += d * dt;
        tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

inline RayDifferential Transform::operator()(const RayDifferential &r) const {
    Ray tr = (*this)(Ray(r));
    RayDifferential ret(tr.o, tr.d, tr.tMax, tr.time, tr.medium);
    ret.hasDifferentials = r.hasDifferentials;
    ret.rxOrigin = (*this)(r.rxOrigin);
    ret.ryOrigin = (*this)(r.ryOrigin);
    ret.rxDirection = (*this)(r.rxDirection);
    ret.ryDirection = (*this)(r.ryDirection);
    return ret;
}

template <typename T>
inline Point3<T> Transform::operator()(const Point3<T> &p, Vector3<T> *pError) const {
    T x = p.x, y = p.y, z = p.z;
    // Compute transformed coordinates from point _pt_
    T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
    T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
    T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
    T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];

    // Compute absolute error for transformed point
    T xAbsSum = (abs(m.m[0][0] * x) + abs(m.m[0][1] * y) +
                 abs(m.m[0][2] * z) + abs(m.m[0][3]));
    T yAbsSum = (abs(m.m[1][0] * x) + abs(m.m[1][1] * y) +
                 abs(m.m[1][2] * z) + abs(m.m[1][3]));
    T zAbsSum = (abs(m.m[2][0] * x) + abs(m.m[2][1] * y) +
                 abs(m.m[2][2] * z) + abs(m.m[2][3]));
    *pError = gamma(3) * Vector3<T>(xAbsSum, yAbsSum, zAbsSum);
    CHECK_NE(wp, 0);
    if (wp == 1)
        return Point3<T>(xp, yp, zp);
    else
        return Point3<T>(xp, yp, zp) / wp;
}

template <typename T>
inline Point3<T> Transform::operator()(const Point3<T> &pt, const Vector3<T> &ptError,
                                       Vector3<T> *absError) const {
    T x = pt.x, y = pt.y, z = pt.z;
    T xp = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z + m.m[0][3];
    T yp = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z + m.m[1][3];
    T zp = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z + m.m[2][3];
    T wp = m.m[3][0] * x + m.m[3][1] * y + m.m[3][2] * z + m.m[3][3];
    absError->x =
        (gamma(3) + T(1)) *
            (abs(m.m[0][0]) * ptError.x + abs(m.m[0][1]) * ptError.y +
             abs(m.m[0][2]) * ptError.z) +
        gamma(3) * (abs(m.m[0][0] * x) + abs(m.m[0][1] * y) +
                    abs(m.m[0][2] * z) + abs(m.m[0][3]));
    absError->y =
        (gamma(3) + T(1)) *
            (abs(m.m[1][0]) * ptError.x + abs(m.m[1][1]) * ptError.y +
             abs(m.m[1][2]) * ptError.z) +
        gamma(3) * (abs(m.m[1][0] * x) + abs(m.m[1][1] * y) +
                    abs(m.m[1][2] * z) + abs(m.m[1][3]));
    absError->z =
        (gamma(3) + T(1)) *
            (abs(m.m[2][0]) * ptError.x + abs(m.m[2][1]) * ptError.y +
             abs(m.m[2][2]) * ptError.z) +
        gamma(3) * (abs(m.m[2][0] * x) + abs(m.m[2][1] * y) +
                    abs(m.m[2][2] * z) + abs(m.m[2][3]));
    CHECK_NE(wp, 0);
    if (wp == 1.)
        return Point3<T>(xp, yp, zp);
    else
        return Point3<T>(xp, yp, zp) / wp;
}

template <typename T>
inline Vector3<T> Transform::operator()(const Vector3<T> &v, Vector3<T> *absError) const {
    T x = v.x, y = v.y, z = v.z;
    absError->x =
        gamma(3) * (abs(m.m[0][0] * v.x) + abs(m.m[0][1] * v.y) +
                    abs(m.m[0][2] * v.z));
    absError->y =
        gamma(3) * (abs(m.m[1][0] * v.x) + abs(m.m[1][1] * v.y) +
                    abs(m.m[1][2] * v.z));
    absError->z =
        gamma(3) * (abs(m.m[2][0] * v.x) + abs(m.m[2][1] * v.y) +
                    abs(m.m[2][2] * v.z));
    return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                      m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                      m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

template <typename T>
inline Vector3<T> Transform::operator()(const Vector3<T> &v, const Vector3<T> &vError,
                                        Vector3<T> *absError) const {
    T x = v.x, y = v.y, z = v.z;
    absError->x =
        (gamma(3) + (T)1) *
            (abs(m.m[0][0]) * vError.x + abs(m.m[0][1]) * vError.y +
             abs(m.m[0][2]) * vError.z) +
        gamma(3) * (abs(m.m[0][0] * v.x) + abs(m.m[0][1] * v.y) +
                    abs(m.m[0][2] * v.z));
    absError->y =
        (gamma(3) + (T)1) *
            (abs(m.m[1][0]) * vError.x + abs(m.m[1][1]) * vError.y +
             abs(m.m[1][2]) * vError.z) +
        gamma(3) * (abs(m.m[1][0] * v.x) + abs(m.m[1][1] * v.y) +
                    abs(m.m[1][2] * v.z));
    absError->z =
        (gamma(3) + (T)1) *
            (abs(m.m[2][0]) * vError.x + abs(m.m[2][1]) * vError.y +
             abs(m.m[2][2]) * vError.z) +
        gamma(3) * (abs(m.m[2][0] * v.x) + abs(m.m[2][1] * v.y) +
                    abs(m.m[2][2] * v.z));
    return Vector3<T>(m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z,
                      m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z,
                      m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z);
}

inline Ray Transform::operator()(const Ray &r, Vector3f *oError, Vector3f *dError) const {
    Point3f o = (*this)(r.o, oError);
    Vector3f d = (*this)(r.d, dError);
    Float tMax = r.tMax;
    Float lengthSquared = d.lengthSq();
    if (lengthSquared > 0) {
        Float dt = dot(abs(d), *oError) / lengthSquared;
        o += d * dt;
        //        tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

inline Ray Transform::operator()(const Ray &r, const Vector3f &oErrorIn, const Vector3f &dErrorIn,
                                 Vector3f *oErrorOut, Vector3f *dErrorOut) const {
    Point3f o = (*this)(r.o, oErrorIn, oErrorOut);
    Vector3f d = (*this)(r.d, dErrorIn, dErrorOut);
    Float tMax = r.tMax;
    Float lengthSquared = d.lengthSq();
    if (lengthSquared > 0) {
        Float dt = dot(abs(d), *oErrorOut) / lengthSquared;
        o += d * dt;
        //        tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

class AnimatedTransform {
public:
    AnimatedTransform(const Transform *startTransform, Float startTime,
                      const Transform *endTransform, Float endTime);

    void interpolate(Float time, Transform *t) const;
    Bounds3f motionBounds(const Bounds3f &b) const;
    Bounds3f boundPointMotion(const Point3f &p) const;

    Ray operator()(const Ray &r) const;
    RayDifferential operator()(const RayDifferential &r) const;
    Point3f operator()(Float time, const Point3f &p) const;
    Vector3f operator()(Float time, const Vector3f &v) const;

    bool hasScale() const {
        return startTransform->hasScale() || endTransform->hasScale();
    }

    static void decompose(const Matrix4x4 &m, Vector3f *T, Quaternion *R, Matrix4x4 *S);

private:
    struct DerivativeTerm {
        DerivativeTerm() {}
        DerivativeTerm(Float c, Float x, Float y, Float z)
            : kc(c), kx(x), ky(y), kz(z) {}
        Float kc, kx, ky, kz;
        Float evaluate(const Point3f &p) const {
            return kc + kx * p.x + ky * p.y + kz * p.z;
        }
    };

    const Transform *startTransform, *endTransform;
    const Float startTime, endTime;
    const bool actuallyAnimated;
    Vector3f T[2];
    Quaternion R[2];
    Matrix4x4 S[2];
    bool hasRotation;
    DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
};

#endif // TRANSFORM_H
