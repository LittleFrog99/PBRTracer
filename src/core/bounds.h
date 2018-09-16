#ifndef CORE_BOUNDS
#define CORE_BOUNDS

#include "vector.h"
#include "ray.h"

template <class T>
class Bounds2 {
public:
    Bounds2() {
        T minNum = numeric_limits<T>::lowest();
        T maxNum = numeric_limits<T>::max();
        pMin = Point2<T>(maxNum, maxNum);
        pMax = Point2<T>(minNum, minNum);
    }

    explicit Bounds2(const Point2<T> &p) : pMin(p), pMax(p) {}
    Bounds2(const Point2<T> &p1, const Point2<T> &p2) {
        pMin = Point2<T>(min(p1.x, p2.x), min(p1.y, p2.y));
        pMax = Point2<T>(max(p1.x, p2.x), max(p1.y, p2.y));
    }

    template <class U>
    explicit operator Bounds2<U>() const {
        return Bounds2<U>(Point2<U>(pMin), Point2<U>(pMax));
    }

    Vector2<T> diagonal() const { return pMax - pMin; }
    T area() const { Vector2<T> d = pMax - pMin; return (d.x * d.y); }
    int maxExtent() const { Vector2<T> diag = diagonal(); return diag.x <= diag.y; }

    const Point2<T> &operator[](int i) const { return (i == 0) ? pMin : pMax;}
    Point2<T> & operator [] (int i) { return (i == 0) ? pMin : pMax; }
    bool operator == (const Bounds2<T> &b) const { return b.pMin == pMin && b.pMax == pMax; }
    bool operator!=(const Bounds2<T> &b) const { return b.pMin != pMin || b.pMax != pMax; }

    Point2<T> lerp(const Point2f &t) const {
        return Point2<T>(Math::lerp(t.x, pMin.x, pMax.x), Math::lerp(t.y, pMin.y, pMax.y));
    }

    Vector2<T> offset(const Point2<T> &p) const {
        Vector2<T> o = p - pMin;
        if (pMax.x > pMin.x) o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
        return o;
    }

    inline void boundingSphere(Point2<T> *c, float *rad) const;

    friend ostream & operator << (ostream &os, const Bounds2<T> &b) {
        os << "[ " << b.pMin << " - " << b.pMax << " ]";
        return os;
    }

    Point2<T> pMin, pMax;
};

template <class T>
class Bounds3 {
  public:
    // Bounds3 Public Methods
    Bounds3() {
        T minNum = numeric_limits<T>::lowest();
        T maxNum = numeric_limits<T>::max();
        pMin = Point3<T>(maxNum, maxNum, maxNum);
        pMax = Point3<T>(minNum, minNum, minNum);
    }

    explicit Bounds3(const Point3<T> &p) : pMin(p), pMax(p) {}

    Bounds3(const Point3<T> &p1, const Point3<T> &p2)
        : pMin(min(p1.x, p2.x), min(p1.y, p2.y),
               min(p1.z, p2.z)),
          pMax(max(p1.x, p2.x), max(p1.y, p2.y),
               max(p1.z, p2.z)) {}

    const Point3<T> & operator [] (int i) const { return (i == 0) ? pMin : pMax; }
    Point3<T> & operator [] (int i) { return (i == 0) ? pMin : pMax; }
    bool operator == (const Bounds3<T> &b) const { return b.pMin == pMin && b.pMax == pMax; }
    bool operator != (const Bounds3<T> &b) const { return b.pMin != pMin || b.pMax != pMax; }

    Point3<T> corner(int corner) const {
        DCHECK(corner >= 0 && corner < 8);
        return Point3<T>((*this)[(corner & 1)].x, (*this)[(corner & 2) ? 1 : 0].y,
                         (*this)[(corner & 4) ? 1 : 0].z);
    }

    Vector3<T> diagonal() const { return pMax - pMin; }

    T surfaceArea() const {
        Vector3<T> d = diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    T volume() const {
        Vector3<T> d = diagonal();
        return d.x * d.y * d.z;
    }

    int maxExtent() const {
        Vector3<T> d = diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    Point3<T> lerp(const Point3f &t) const {
        return Point3<T>(lerp(t.x, pMin.x, pMax.x), lerp(t.y, pMin.y, pMax.y), lerp(t.z, pMin.z, pMax.z));
    }

    Vector3<T> offset(const Point3<T> &p) const {
        Vector3<T> o = p - pMin;
        if (pMax.x > pMin.x) o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z) o.z /= pMax.z - pMin.z;
        return o;
    }

    inline void boundingSphere(Point3<T> *center, float *radius) const;

    template <class U>
    explicit operator Bounds3<U>() const {
        return Bounds3<U>(Point3<U>(pMin), Point3<U>(pMax));
    }

    bool intersectP(const Ray &ray, float *hitt0 = nullptr, float *hitt1 = nullptr) const;
    inline bool intersectP(const Ray &ray, const Vector3f &invDir, const int dirIsNeg[3]) const;

    friend ostream & operator << (ostream &os, const Bounds3<T> &b) {
        os << "[ " << b.pMin << " - " << b.pMax << " ]";
        return os;
    }

    Point3<T> pMin, pMax;
};

typedef Bounds2<float> Bounds2f;
typedef Bounds2<int> Bounds2i;
typedef Bounds3<float> Bounds3f;
typedef Bounds3<int> Bounds3i;

class Bounds2iIterator : public forward_iterator_tag {
public:
    Bounds2iIterator(const Bounds2i &b, const Point2i &pt)
        : p(pt), bounds(&b) {}

    Bounds2iIterator operator ++ () { advance(); return *this; }

    Bounds2iIterator operator ++ (int) {
        Bounds2iIterator old = *this;
        advance();
        return old;
    }

    bool operator == (const Bounds2iIterator &bi) const { return p == bi.p && bounds == bi.bounds; }
    bool operator != (const Bounds2iIterator &bi) const { return p != bi.p || bounds != bi.bounds; }
    Point2i operator * () const { return p; }

private:
    void advance() {
        ++p.x;
        if (p.x == bounds->pMax.x) {
            p.x = bounds->pMin.x;
            ++p.y;
        }
    }

    Point2i p;
    const Bounds2i *bounds;
};

template <class T>
bool Bounds3<T>::intersectP(const Ray &ray, float *hitt0, float *hitt1) const {
    float t0 = 0, t1 = ray.tMax;
    for (int i = 0; i < 3; ++i) {
        // Update interval for i_th bounding box slab
        float invRayDir = 1 / ray.d[i];
        float tNear = (pMin[i] - ray.o[i]) * invRayDir;
        float tFar = (pMax[i] - ray.o[i]) * invRayDir;
        // Update parametric interval from slab intersection t values
        if (tNear > tFar) swap(tNear, tFar);
        // Update _tFar_ to ensure robust ray--bounds intersection
        tFar *= 1 + 2 * Math::gamma(3);
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;
        if (t0 > t1) return false;
    }
    if (hitt0) *hitt0 = t0;
    if (hitt1) *hitt1 = t1;
    return true;
}

template <class T>
inline bool Bounds3<T>::intersectP(const Ray &ray, const Vector3f &invDir,
                                   const int dirIsNeg[3]) const {
    const Bounds3f &bounds = *this;
    // Check for ray intersection against $x$ and $y$ slabs
    float tMin = (bounds[dirIsNeg[0]].x - ray.o.x) * invDir.x;
    float tMax = (bounds[1 - dirIsNeg[0]].x - ray.o.x) * invDir.x;
    float tyMin = (bounds[dirIsNeg[1]].y - ray.o.y) * invDir.y;
    float tyMax = (bounds[1 - dirIsNeg[1]].y - ray.o.y) * invDir.y;

    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * Math::gamma(3);
    tyMax *= 1 + 2 * Math::gamma(3);
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    float tzMin = (bounds[dirIsNeg[2]].z - ray.o.z) * invDir.z;
    float tzMax = (bounds[1 - dirIsNeg[2]].z - ray.o.z) * invDir.z;

    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * Math::gamma(3);
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}

namespace Math {

template <class T>
inline Bounds3<T> unionOf(const Bounds3<T> &b, const Point3<T> &p) {
    Bounds3<T> ret;
    ret.pMin = min(b.pMin, p);
    ret.pMax = max(b.pMax, p);
    return ret;
}

template <class T>
inline Bounds3<T> unionOf(const Bounds3<T> &b1, const Bounds3<T> &b2) {
    Bounds3<T> ret;
    ret.pMin = min(b1.pMin, b2.pMin);
    ret.pMax = max(b1.pMax, b2.pMax);
    return ret;
}

template <class T>
inline Bounds3<T> intersect(const Bounds3<T> &b1, const Bounds3<T> &b2) {
    Bounds3<T> ret;
    ret.pMin = max(b1.pMin, b2.pMin);
    ret.pMax = min(b1.pMax, b2.pMax);
    return ret;
}

template <class T>
inline bool overlaps(const Bounds3<T> &b1, const Bounds3<T> &b2) {
    bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
    bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
    bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
    return (x && y && z);
}

template <class T>
inline bool inside(const Point2<T> &p, const Bounds2<T> &b) {
    return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y);
}

template <class T>
inline bool insideExclusive(const Point2<T> &p, const Bounds2<T> &b) {
    return (p.x >= b.pMin.x && p.x < b.pMax.x && p.y >= b.pMin.y && p.y < b.pMax.y);
}

template <class T>
inline bool inside(const Point3<T> &p, const Bounds3<T> &b) {
    return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
            p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
}

template <class T>
inline bool insideExclusive(const Point3<T> &p, const Bounds3<T> &b) {
    return (p.x >= b.pMin.x && p.x < b.pMax.x && p.y >= b.pMin.y &&
            p.y < b.pMax.y && p.z >= b.pMin.z && p.z < b.pMax.z);
}

template <class T, class U>
inline Bounds3<T> expand(const Bounds3<T> &b, U delta) {
    return Bounds3<T>(b.pMin - Vector3<T>(delta, delta, delta),
                      b.pMax + Vector3<T>(delta, delta, delta));
}

// Minimum squared distance from point to box; returns zero if point is
// inside.
template <class T, class U>
inline float distanceSq(const Point3<T> &p, const Bounds3<U> &b) {
    float dx = std::max({float(0), b.pMin.x - p.x, p.x - b.pMax.x});
    float dy = std::max({float(0), b.pMin.y - p.y, p.y - b.pMax.y});
    float dz = std::max({float(0), b.pMin.z - p.z, p.z - b.pMax.z});
    return dx * dx + dy * dy + dz * dz;
}

template <class T, class U>
inline float distance(const Point3<T> &p, const Bounds3<U> &b) {
    return sqrt(distanceSq(p, b));
}

template <class T>
inline Bounds2<T> unionOf(const Bounds2<T> &b, const Point2<T> &p) {
    Bounds2<T> ret;
    ret.pMin = min(b.pMin, p);
    ret.pMax = max(b.pMax, p);
    return ret;
}

template <class T>
inline Bounds2<T> unionOf(const Bounds2<T> &b, const Bounds2<T> &b2) {
    Bounds2<T> ret;
    ret.pMin = min(b.pMin, b2.pMin);
    ret.pMax = max(b.pMax, b2.pMax);
    return ret;
}

template <class T>
inline Bounds2<T> intersect(const Bounds2<T> &b1, const Bounds2<T> &b2) {
    // Important: assign to pMin/pMax directly and don't run the Bounds2()
    // constructor, since it takes min/max of the points passed to it.  In
    // turn, that breaks returning an invalid bound for the case where we
    // intersect non-overlapping bounds (as we'd like to happen).
    Bounds2<T> ret;
    ret.pMin = max(b1.pMin, b2.pMin);
    ret.pMax = min(b1.pMax, b2.pMax);
    return ret;
}

template <class T>
inline bool overlaps(const Bounds2<T> &ba, const Bounds2<T> &bb) {
    bool x = (ba.pMax.x >= bb.pMin.x) && (ba.pMin.x <= bb.pMax.x);
    bool y = (ba.pMax.y >= bb.pMin.y) && (ba.pMin.y <= bb.pMax.y);
    return (x && y);
}

template <class T, class U>
inline Bounds2<T> expand(const Bounds2<T> &b, U delta) {
    return Bounds2<T>(b.pMin - Vector2<T>(delta, delta), b.pMax + Vector2<T>(delta, delta));
}

}

template<class T>
inline void Bounds2<T>::boundingSphere(Point2<T> *c, float *rad) const {
    *c = (pMin + pMax) / 2;
    *rad = Math::inside(*c, *this) ? Math::distance(*c, pMax) : 0;
}

template<class T>
inline void Bounds3<T>::boundingSphere(Point3<T> *center, float *radius) const {
    *center = (pMin + pMax) / 2;
    *radius = Math::inside(*center, *this) ? Math::distance(*center, pMax) : 0;
}

inline Bounds2iIterator begin(const Bounds2i &b) {
    return Bounds2iIterator(b, b.pMin);
}

inline Bounds2iIterator end(const Bounds2i &b) {
    Point2i pEnd(b.pMin.x, b.pMax.y);
    if (b.pMin.x >= b.pMax.x || b.pMin.y >= b.pMax.y) // bounds are degenerate
        pEnd = b.pMin; // any attempt to iterate over the bounds exit immediately
    return Bounds2iIterator(b, pEnd);
}

#endif // CORE_BOUNDS
