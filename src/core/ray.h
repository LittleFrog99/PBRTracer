#ifndef CORE_RAY
#define CORE_RAY

#include "vector.h"

class Medium;

class Ray {
public:
    Ray() : tMax(INFINITY), time(0.f), medium(nullptr) {}

    Ray(const Point3f &o, const Vector3f &d, Float tMax = INFINITY,
        Float time = 0.f, const Medium *medium = nullptr)
        : o(o), d(d), tMax(tMax), time(time), medium(medium) {}

    Point3f operator()(Float t) const { return o + d * t; }
    bool hasNaNs() const { return (o.hasNaNs() || d.hasNaNs() || isnan(tMax)); }

    friend ostream & operator << (ostream &os, const Ray &r) {
        os << "[o=" << r.o << ", d=" << r.d << ", tMax=" << r.tMax
           << ", time=" << r.time << "]";
        return os;
    }

    Point3f o;
    Vector3f d;
    mutable Float tMax;
    Float time;
    const Medium *medium;
};

class RayDifferential : public Ray {
public:
    RayDifferential() { hasDifferentials = false; }

    RayDifferential(const Point3f &o, const Vector3f &d, Float tMax = INFINITY,
                    Float time = 0.f, const Medium *medium = nullptr)
        : Ray(o, d, tMax, time, medium) {
        hasDifferentials = false;
    }
    RayDifferential(const Ray &ray) : Ray(ray) { hasDifferentials = false; }

    bool hasNaNs() const {
        return Ray::hasNaNs() || (hasDifferentials && (rxOrigin.hasNaNs() || ryOrigin.hasNaNs() ||
                                                       rxDirection.hasNaNs() || ryDirection.hasNaNs()));
    }

    void scaleDifferentials(Float s) {
        rxOrigin = o + (rxOrigin - o) * s;
        ryOrigin = o + (ryOrigin - o) * s;
        rxDirection = d + (rxDirection - d) * s;
        ryDirection = d + (ryDirection - d) * s;
    }

    friend ostream &operator<<(ostream &os, const RayDifferential &r) {
        os << "[ " << static_cast<const Ray &>(r) << " has differentials: " <<
            (r.hasDifferentials ? "true" : "false") << ", xo = " << r.rxOrigin <<
            ", xd = " << r.rxDirection << ", yo = " << r.ryOrigin << ", yd = " <<
            r.ryDirection;
        return os;
    }

    bool hasDifferentials;
    Point3f rxOrigin, ryOrigin;
    Vector3f rxDirection, ryDirection;
};

#endif // CORE_RAY
