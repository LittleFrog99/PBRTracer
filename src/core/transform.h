#ifndef CORE_TRANSFORM
#define CORE_TRANSFORM

#include "bounds.h"
#include "matrix.h"
#include "quaternion.h"

class SurfaceInteraction;

class Transform {
public:
    Transform() {}
    Transform(const float mat[4][4]) {
        m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3], mat[1][0],
                      mat[1][1], mat[1][2], mat[1][3], mat[2][0], mat[2][1],
                      mat[2][2], mat[2][3], mat[3][0], mat[3][1], mat[3][2],
                      mat[3][3]);
        mInv = m.inverse();
    }

    Transform(const Matrix4x4 &m) : m(m), mInv(m.inverse()) {}
    Transform(const Matrix4x4 &m, const Matrix4x4 &mInv) : m(m), mInv(mInv) {}

    static Transform translate(const Vector3f &delta);
    static Transform scale(float x, float y, float z);
    static Transform rotateX(float theta);
    static Transform rotateY(float theta);
    static Transform rotateZ(float theta);
    static Transform rotate(float theta, const Vector3f &axis);
    static Transform lookAt(const Point3f &pos, const Point3f &look, const Vector3f &up);
    static Transform orthographic(float znear, float zfar);
    static Transform perspective(float fov, float znear, float zfar);

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
        float la2 = (*this)(Vector3f(1, 0, 0)).lengthSq();
        float lb2 = (*this)(Vector3f(0, 1, 0)).lengthSq();
        float lc2 = (*this)(Vector3f(0, 0, 1)).lengthSq();
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
    float lengthSquared = d.lengthSq();
    float tMax = r.tMax;
    if (lengthSquared > 0) {
        float dt = Math::dot(Math::abs(d), oError) / lengthSquared;
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
    float tMax = r.tMax;
    float lengthSquared = d.lengthSq();
    if (lengthSquared > 0) {
        float dt = dot(abs(d), *oError) / lengthSquared;
        o += d * dt;
        //        tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

inline Ray Transform::operator()(const Ray &r, const Vector3f &oErrorIn, const Vector3f &dErrorIn,
                                 Vector3f *oErrorOut, Vector3f *dErrorOut) const {
    Point3f o = (*this)(r.o, oErrorIn, oErrorOut);
    Vector3f d = (*this)(r.d, dErrorIn, dErrorOut);
    float tMax = r.tMax;
    float lengthSquared = d.lengthSq();
    if (lengthSquared > 0) {
        float dt = dot(abs(d), *oErrorOut) / lengthSquared;
        o += d * dt;
        //        tMax -= dt;
    }
    return Ray(o, d, tMax, r.time, r.medium);
}

class Interval {
public:
    Interval(float v) : low(v), high(v) {}
    Interval(float v0, float v1) : low(min(v0, v1)), high(max(v0, v1)) {}

    Interval operator + (const Interval &i) const { return Interval(low + i.low, high + i.high); }
    Interval operator - (const Interval &i) const { return Interval(low - i.high, high - i.low); }
    Interval operator * (const Interval &i) const {
        return Interval(min(min(low * i.low, high * i.low), min(low * i.high, high * i.high)),
                        max(max(low * i.low, high * i.low), max(low * i.high, high * i.high)));
    }

    static void findZeros(float c1, float c2, float c3, float c4, float c5, float theta,
                          Interval tInterval, float *zeros, int *zeroCount, int depth = 8);

    float low, high;
};

namespace Math {
inline Interval sin(const Interval &i) {
    CHECK_GE(i.low, 0);
    CHECK_LE(i.high, 2.0001 * PI);
    float sinLow = std::sin(i.low), sinHigh = std::sin(i.high);
    if (sinLow > sinHigh) swap(sinLow, sinHigh);
    if (i.low < PI / 2 && i.high > PI / 2) sinHigh = 1.;
    if (i.low < (3.f / 2.f) * PI && i.high > (3.f / 2.f) * PI) sinLow = -1.;
    return Interval(sinLow, sinHigh);
}

inline Interval cos(const Interval &i) {
    CHECK_GE(i.low, 0);
    CHECK_LE(i.high, 2.0001 * PI);
    float cosLow = std::cos(i.low), cosHigh = std::cos(i.high);
    if (cosLow > cosHigh) swap(cosLow, cosHigh);
    if (i.low < PI && i.high > PI) cosLow = -1.;
    return Interval(cosLow, cosHigh);
}

}

class AnimatedTransform {
public:
    AnimatedTransform(const Transform *startTransform, float startTime,
                      const Transform *endTransform, float endTime);

    void interpolate(float time, Transform *t) const;
    Bounds3f motionBounds(const Bounds3f &b) const;
    Bounds3f boundPointMotion(const Point3f &p) const;

    Ray operator()(const Ray &r) const;
    RayDifferential operator()(const RayDifferential &r) const;
    Point3f operator()(float time, const Point3f &p) const;
    Vector3f operator()(float time, const Vector3f &v) const;

    bool hasScale() const {
        return startTransform->hasScale() || endTransform->hasScale();
    }

    static void decompose(const Matrix4x4 &m, Vector3f *T, Quaternion *R, Matrix4x4 *S);

private:
    struct DerivativeTerm {
        DerivativeTerm() {}
        DerivativeTerm(float c, float x, float y, float z)
            : kc(c), kx(x), ky(y), kz(z) {}
        float kc, kx, ky, kz;
        float evaluate(const Point3f &p) const {
            return kc + kx * p.x + ky * p.y + kz * p.z;
        }
    };

    const Transform *startTransform, *endTransform;
    const float startTime, endTime;
    const bool actuallyAnimated;
    Vector3f T[2];
    Quaternion R[2];
    Matrix4x4 S[2];
    bool hasRotation;
    DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
};

#endif // CORE_TRANSFORM
