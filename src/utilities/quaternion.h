#ifndef UTILITY_QUATERNION
#define UTILITY_QUATERNION

#include "vector.h"

class Transform;

struct Quaternion {
    Quaternion() : v(0, 0, 0), w(1) {}
    Quaternion(const Transform &t);

    Quaternion & operator += (const Quaternion &q) {
        v += q.v;
        w += q.w;
        return *this;
    }

    Quaternion operator + (const Quaternion &q2) {
        Quaternion ret = *this;
        return ret += q2;
    }

    Quaternion & operator -= (const Quaternion &q) {
        v -= q.v;
        w -= q.w;
        return *this;
    }

    Quaternion operator - () const {
        Quaternion ret;
        ret.v = -v;
        ret.w = -w;
        return ret;
    }

    Quaternion operator - (const Quaternion &q2) {
        Quaternion ret = *this;
        return ret -= q2;
    }

    Quaternion & operator *= (Float f) {
        v *= f;
        w *= f;
        return *this;
    }

    Quaternion operator * (Float f) const {
        Quaternion ret = *this;
        ret.v *= f;
        ret.w *= f;
        return ret;
    }

    Quaternion & operator /= (Float f) {
        v /= f;
        w /= f;
        return *this;
    }

    Quaternion operator / (Float f) const {
        Quaternion ret = *this;
        ret.v /= f;
        ret.w /= f;
        return ret;
    }

    friend ostream & operator << (ostream &os, const Quaternion &q) {
        os << StringPrint::printf("[ %f, %f, %f, %f ]", q.v.x, q.v.y, q.v.z, q.w);
        return os;
    }

    inline Quaternion normalize();

    Transform toTransform() const;

    Vector3f v;
    Float w;
};

inline Quaternion operator * (Float f, const Quaternion &q) { return q * f; }

namespace Math {

Quaternion slerp(Float t, const Quaternion &q1, const Quaternion &q2);

inline Float dot(const Quaternion &q1, const Quaternion &q2) {
    return dot(q1.v, q2.v) + q1.w * q2.w;
}

}

inline Quaternion Quaternion::normalize() {
    return (*this) / sqrt(dot(*this, *this));
}

#endif // UTILITY_QUATERNION
