#ifndef UTILITY_VECTOR
#define UTILITY_VECTOR

#include "stringprint.h"

// Forward Declarations
template <class T>
class Vector2;
template <class T>
class Vector3;
template <class T>
class Point3;
template <class T>
class Point2;
template <class T>
class Normal3;

template <class T>
class Vector2 {
public:
    Vector2() { x = y = 0; }
    bool hasNaNs() const { return isnan(x) || isnan(y); }
    Vector2(T x) : x(x), y(x) { DCHECK(!hasNaNs()); }
    Vector2(T xx, T yy) : x(xx), y(yy) { DCHECK(!hasNaNs()); }
    explicit Vector2(const Point2<T> &p);
    explicit Vector2(const Point3<T> &p);

    Vector2(const Vector2<T> &v) { x = v.x; y = v.y; }
    Vector2<T> & operator = (const Vector2<T> &v) { x = v.x; y = v.y; return *this; }
    Vector2<T> operator + (const Vector2<T> &v) const { return Vector2(x + v.x, y + v.y); }
    Vector2<T> & operator += (const Vector2<T> &v) { x += v.x; y += v.y; return *this; }
    Vector2<T> operator - (const Vector2<T> &v) const { return Vector2(x - v.x, y - v.y); }
    Vector2<T> & operator -= (const Vector2<T> &v) { x -= v.x; y -= v.y; return *this; }
    bool operator == (const Vector2<T> &v) const { return x == v.x && y == v.y; }
    bool operator != (const Vector2<T> &v) const { return x != v.x || y != v.y; }

    template <class U>
    Vector2<T> operator * (U f) const { return Vector2<T>(f * x, f * y); }

    template <class U>
    Vector2<T> &operator *= (U f) { x *= f; y *= f; return *this; }

    template <class U>
    Vector2<T> operator / (U f) const {
        float inv = 1.0 / f;
        return Vector2<T>(x * inv, y * inv);
    }

    template <class U>
    Vector2<T> & operator /= (U f) { float inv = 1.0 / f; x *= inv; y *= inv; return *this; }

    Vector2<T> operator - () const { return Vector2<T>(-x, -y); }
    T operator [] (int i) const { if (i == 0) return x; return y; }
    T & operator [] (int i) { if (i == 0) return x; return y; }
    float lengthSq() const { return x * x + y * y; }
    float length() const { return sqrt(lengthSq()); }

    T x, y;
};

template <class T, class U>
Vector2<T> operator * (U f, const Vector2<T> &v) {
    return v * f;
}

template <class T>
ostream & operator << (ostream &os, const Vector2<T> &v) {
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

template <class T>
class Vector3 {
public:
    Vector3() { x = y = z = 0; }
    bool hasNaNs() const { return isnan(x) || isnan(y) || isnan(z); }
    Vector3(T x) : x(x), y(x), z(x) { DCHECK(!hasNaNs()); }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) { DCHECK(!hasNaNs()); }
    Vector3(const Vector3<T> &v) { x = v.x; y = v.y; z = v.z; DCHECK(!hasNaNs()); }
    explicit Vector3(const Point3<T> &p);
    explicit Vector3(const Normal3<T> &n);

    Vector3<T> & operator = (const Vector3<T> &v) { x = v.x; y = v.y; z = v.z; return *this; }
    Vector3<T> operator + (const Vector3<T> &v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3<T> & operator += (const Vector3<T> &v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3<T> operator - (const Vector3<T> &v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3<T> & operator -= (const Vector3<T> &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3<T> operator - () const { return Vector3<T>(-x, -y, -z); }
    bool operator == (const Vector3<T> &v) const { return x == v.x && y == v.y && z == v.z; }
    bool operator != (const Vector3<T> &v) const { return x != v.x || y != v.y || z != v.z; }

    template <class U>
    Vector3<T> operator * (U s) const { return Vector3<T>(s * x, s * y, s * z); }

    template <class U>
    friend Vector3<T> operator * (U s, const Vector3<T> &v) { return Vector3<T>(s * v.x, s * v.y, s * v.z); }

    template <class U>
    Vector3<T> & operator *= (U s) { x *= s; y *= s; z *= s; return *this; }

    template <class U>
    Vector3<T> operator / (U f) const {
        float inv = 1.0 / f;
        return Vector3<T>(x * inv, y * inv, z * inv);
    }

    template <class U>
    Vector3<T> &operator /= (U f) {
        float inv = 1.0 / f;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }

    T operator [] (int i) const { if (i == 0) return x; if (i == 1) return y; return z; }
    T & operator [] (int i) { if (i == 0) return x; if (i == 1) return y; return z; }
    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }

    T x, y, z;
};

template <class T>
ostream & operator << (ostream &os, const Vector3<T> &v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

typedef Vector2<float> Vector2f;
typedef Vector2<int> Vector2i;
typedef Vector3<float> Vector3f;
typedef Vector3<int> Vector3i;

// Point Declarations
template <class T>
class Point2 {
public:
    Point2() { x = y = 0; }
    Point2(T xx, T yy) : x(xx), y(yy) { }
    Point2(const Point2<T> &p) { x = p.x; y = p.y; DCHECK(!hasNaNs()); }
    explicit Point2(const Point3<T> &p) : x(p.x), y(p.y) { DCHECK(!hasNaNs()); }
    bool hasNaNs() const { return isnan(x) || isnan(y); }

    template <class U>
    explicit Point2(const Point2<U> &p) { x = T(p.x); y = T(p.y); }

    template <class U>
    explicit Point2(const Vector2<U> &p) { x = T(p.x); y = T(p.y); }

    template <class U>
    explicit operator Vector2<U>() const { return Vector2<U>(x, y); }

    Point2<T> & operator = (const Point2<T> &p) { x = p.x; y = p.y; return *this; }
    Point2<T> operator + (const Vector2<T> &v) const { return Point2<T>(x + v.x, y + v.y); }
    Point2<T> & operator += (const Vector2<T> &v) { x += v.x; y += v.y; return *this; }
    Vector2<T> operator - (const Point2<T> &p) const { return Vector2<T>(x - p.x, y - p.y); }
    Point2<T> operator - (const Vector2<T> &v) const { return Point2<T>(x - v.x, y - v.y); }
    Point2<T> operator - () const { return Point2<T>(-x, -y); }
    Point2<T> & operator -= (const Vector2<T> &v) {x -= v.x; y -= v.y; return *this; }
    Point2<T> & operator += (const Point2<T> &p) { x += p.x; y += p.y; return *this; }
    Point2<T> operator + (const Point2<T> &p) const { return Point2<T>(x + p.x, y + p.y); }

    template <class U>
    Point2<T> operator * (U f) const { return Point2<T>(f * x, f * y); }

    template <class U>
    Point2<T> & operator *= (U f) { x *= f; y *= f; return *this; }

    template <class U>
    Point2<T> operator / (U f) const { float inv = 1.0 / f; return Point2<T>(inv * x, inv * y); }

    template <class U>
    Point2<T> & operator /= (U f) {
        float inv = 1.0 / f;
        x *= inv; y *= inv;
        return *this;
    }

    T operator [] (int i) const { if (i == 0) return x; return y; }
    T & operator [] (int i) { if (i == 0) return x; return y; }
    bool operator == (const Point2<T> &p) const { return x == p.x && y == p.y; }
    bool operator != (const Point2<T> &p) const { return x != p.x || y != p.y; }

    T x, y;
};

template <class T, class U>
Point2<T> operator*(U f, const Point2<T> &p) {
    return p * f;
}

template <class T>
ostream & operator << (ostream &os, const Point2<T> &v) {
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

template <class T>
class Point3 {
public:
    Point3() { x = y = z = 0; }
    Point3(T x, T y, T z) : x(x), y(y), z(z) { DCHECK(!hasNaNs()); }
    Point3(const Point3<T> &p) { x = p.x; y = p.y; z = p.z; DCHECK(!hasNaNs()); }
    bool hasNaNs() const { return isnan(x) || isnan(y) || isnan(z); }

    template <class U>
    explicit Point3(const Point3<U> &p) : x(p.x), y(p.y), z(p.z) {}

    template <class U>
    explicit operator Vector3<U>() const { return Vector3<U>(x, y, z); }

    Point3<T> & operator = (const Point3<T> &p) { x = p.x; y = p.y; z = p.z; return *this; }
    Point3<T> operator + (const Vector3<T> &v) const { return Point3<T>(x + v.x, y + v.y, z + v.z); }
    Point3<T> & operator += (const Vector3<T> &v) { x += v.x; y += v.y; z += v.z; return *this; }
    Point3<T> operator - () const { return Point3<T>(-x, -y, -z); }
    Vector3<T> operator - (const Point3<T> &p) const { return Vector3<T>(x - p.x, y - p.y, z - p.z); }
    Point3<T> operator - (const Vector3<T> &v) const { return Point3<T>(x - v.x, y - v.y, z - v.z); }
    Point3<T> & operator -= (const Vector3<T> &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Point3<T> & operator += (const Point3<T> &p) { x += p.x; y += p.y; z += p.z; return *this; }
    Point3<T> operator + (const Point3<T> &p) const { return Point3<T>(x + p.x, y + p.y, z + p.z); }

    template <class U>
    Point3<T> operator * (U f) const { return Point3<T>(f * x, f * y, f * z); }

    template <class U>
    Point3<T> & operator *= (U f) { x *= f; y *= f; z *= f; return *this; }

    template <class U>
    Point3<T> operator / (U f) const { float inv = 1.0 / f; return Point3<T>(inv * x, inv * y, inv * z); }

    template <class U>
    Point3<T> & operator /= (U f) {
        float inv = 1.0 / f;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }

    T operator [] (int i) const { if (i == 0) return x; if (i == 1) return y; return z; }
    T & operator [] (int i) { if (i == 0) return x; if (i == 1) return y; return z; }
    bool operator == (const Point3<T> &p) const { return x == p.x && y == p.y && z == p.z; }
    bool operator != (const Point3<T> &p) const { return x != p.x || y != p.y || z != p.z; }

    T x, y, z;
};

template <class T, class U>
Point3<T> operator * (U f, const Point3<T> &p) {
    return p * f;
}

template <class T>
ostream & operator << (ostream &os, const Point3<T> &v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

typedef Point2<float> Point2f;
typedef Point2<int> Point2i;
typedef Point3<float> Point3f;
typedef Point3<int> Point3i;


template <class T>
class Normal3 {
public:
    Normal3() { x = y = z = 0; }
    Normal3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) { DCHECK(!hasNaNs()); }
    Normal3<T>(const Normal3<T> &n) { x = n.x; y = n.y; z = n.z; DCHECK(!hasNaNs()); }
    explicit Normal3<T>(const Vector3<T> &v) : x(v.x), y(v.y), z(v.z) { DCHECK(!hasNaNs()); }
    bool hasNaNs() const { return isnan(x) || isnan(y) || isnan(z); }

    Normal3<T> & operator = (const Normal3<T> &n) { x = n.x; y = n.y; z = n.z; return *this; }
    Normal3<T> operator - () const { return Normal3(-x, -y, -z); }
    Normal3<T> operator + (const Normal3<T> &n) const { return Normal3<T>(x + n.x, y + n.y, z + n.z); }
    Normal3<T> & operator += (const Normal3<T> &n) { x += n.x; y += n.y; z += n.z; return *this; }
    Normal3<T> operator - (const Normal3<T> &n) const { return Normal3<T>(x - n.x, y - n.y, z - n.z); }
    Normal3<T> & operator -= (const Normal3<T> &n) { x -= n.x; y -= n.y; z -= n.z; return *this; }

    template <class U>
    Normal3<T> operator * (U f) const { return Normal3<T>(f * x, f * y, f * z); }

    template <class U>
    Normal3<T> & operator *= (U f) { x *= f; y *= f; z *= f; return *this; }

    template <class U>
    Normal3<T> operator / (U f) const {
        float inv = 1.0 / f;
        return Normal3<T>(x * inv, y * inv, z * inv);
    }

    template <class U>
    Normal3<T> & operator /= (U f) {
        float inv = 1.0 / f;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }

    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }

    bool operator == (const Normal3<T> &n) const { return x == n.x && y == n.y && z == n.z; }
    bool operator != (const Normal3<T> &n) const { return x != n.x || y != n.y || z != n.z; }
    T operator [] (int i) const { if (i == 0) return x; if (i == 1) return y; return z; }
    T & operator [] (int i) { if (i == 0) return x; if (i == 1) return y; return z; }

    T x, y, z;
};

template <class T, class U>
inline Normal3<T> operator * (U f, const Normal3<T> &n) {
    return Normal3<T>(f * n.x, f * n.y, f * n.z);
}

template <class T>
inline ostream & operator << (ostream &os, const Normal3<T> &v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

typedef Normal3<float> Normal3f;

template<class T>
inline Vector2<T>::Vector2(const Point2<T> &p) : x(p.x), y(p.y) {}

template<class T>
inline Vector2<T>::Vector2(const Point3<T> &p) : x(p.x), y(p.y) {}

template <class T>
inline Vector3<T>::Vector3(const Point3<T> &p) : x(p.x), y(p.y), z(p.z) {}

template<class T>
inline Vector3<T>::Vector3(const Normal3<T> &n) : x(n.x), y(n.y), z(n.z) {}

namespace Math {
template <class T>
inline Vector3<T> cross(const Vector3<T> &v1, const Vector3<T> &v2) {
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <class T>
inline Vector3<T> cross(const Vector3<T> &v1, const Normal3<T> &v2) {
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <class T>
inline Vector3<T> cross(const Normal3<T> &v1, const Vector3<T> &v2) {
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <class T>
inline Vector3<T> vecFunc(const Vector3<T> &v, function<T(T)> func) {
    return Vector3<T>(func(v.x), func(v.y), func(v.z));
}

template <class T>
inline Vector3<T> vecFunc(const Vector3<T> &v1, const Vector3<T> &v2, function<T(T, T)> func) {
    return Vector3<T>(func(v1.x, v2.x), func(v1.y, v2.y), func(v1.z, v2.z));
}

template <class T>
inline Vector3<T> abs(const Vector3<T> &v) {
    return Vector3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template <class T>
inline Vector3<T> normalize(const Vector3<T> &v) {
    return v / v.length();
}

template <class T>
inline T minComp(const Vector3<T> &v) {
    return min(v.x, min(v.y, v.z));
}

template <class T>
inline T maxComp(const Vector3<T> &v) {
    return max(v.x, max(v.y, v.z));
}

template <class T>
inline int maxDim(const Vector3<T> &v) {
    return (v.x > v.y) ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2);
}

template <class T>
inline Vector3<T> min(const Vector3<T> &p1, const Vector3<T> &p2) {
    return Vector3<T>(min(p1.x, p2.x), min(p1.y, p2.y), min(p1.z, p2.z));
}

template <class T>
inline Vector3<T> max(const Vector3<T> &p1, const Vector3<T> &p2) {
    return Vector3<T>(max(p1.x, p2.x), max(p1.y, p2.y), max(p1.z, p2.z));
}

template <class T>
inline Vector3<T> permute(const Vector3<T> &v, int x, int y, int z) {
    return Vector3<T>(v[x], v[y], v[z]);
}

template <class T>
inline void coordSystem(const Vector3<T> &v1, Vector3<T> *v2,
                             Vector3<T> *v3) {
    if (std::abs(v1.x) > std::abs(v1.y))
        *v2 = Vector3<T>(-v1.z, 0, v1.x) / sqrt(v1.x * v1.x + v1.z * v1.z);
    else
        *v2 = Vector3<T>(0, v1.z, -v1.y) / sqrt(v1.y * v1.y + v1.z * v1.z);
    *v3 = cross(v1, *v2);
}

template <class T>
inline float dot(const Vector2<T> &v1, const Vector2<T> &v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

template <class T>
inline float absDot(const Vector2<T> &v1, const Vector2<T> &v2) {
    return abs(dot(v1, v2));
}

template <class T>
inline Vector2<T> normalize(const Vector2<T> &v) {
    return v / v.length();
}

template <class T>
inline Vector2<T> abs(const Vector2<T> &v) {
    return Vector2<T>(abs(v.x), abs(v.y));
}

template <class T>
inline float distance(const Point3<T> &p1, const Point3<T> &p2) {
    return (p1 - p2).length();
}

template <class T>
inline float distanceSq(const Point3<T> &p1, const Point3<T> &p2) {
    return (p1 - p2).lengthSq();
}

template <class T>
inline Point3<T> lerp(float t, const Point3<T> &p0, const Point3<T> &p1) {
    return (1 - t) * p0 + t * p1;
}

template <class T>
inline Point3<T> min(const Point3<T> &p1, const Point3<T> &p2) {
    return Point3<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y),
                     std::min(p1.z, p2.z));
}

template <class T>
inline Point3<T> max(const Point3<T> &p1, const Point3<T> &p2) {
    return Point3<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y),
                     std::max(p1.z, p2.z));
}

template <class T>
inline Point3<T> floor(const Point3<T> &p) {
    return Point3<T>(floor(p.x), floor(p.y), floor(p.z));
}

template <class T>
inline Point3<T> ceil(const Point3<T> &p) {
    return Point3<T>(ceil(p.x), ceil(p.y), ceil(p.z));
}

template <class T>
inline Point3<T> abs(const Point3<T> &p) {
    return Point3<T>(abs(p.x), abs(p.y), abs(p.z));
}

template <class T>
inline float distance(const Point2<T> &p1, const Point2<T> &p2) {
    return (p1 - p2).length();
}

template <class T>
inline float distanceSq(const Point2<T> &p1, const Point2<T> &p2) {
    return (p1 - p2).lengthSq();
}

template <class T>
inline Point2<T> floor(const Point2<T> &p) {
    return Point2<T>(std::floor(p.x), std::floor(p.y));
}

template <class T>
inline Point2<T> ceil(const Point2<T> &p) {
    return Point2<T>(std::ceil(p.x), std::ceil(p.y));
}

template <class T>
inline Point2<T> lerp(float t, const Point2<T> &v0, const Point2<T> &v1) {
    return (1 - t) * v0 + t * v1;
}

template <class T>
inline Point2<T> min(const Point2<T> &pa, const Point2<T> &pb) {
    return Point2<T>(std::min(pa.x, pb.x), std::min(pa.y, pb.y));
}

template <class T>
inline Point2<T> max(const Point2<T> &pa, const Point2<T> &pb) {
    return Point2<T>(std::max(pa.x, pb.x), std::max(pa.y, pb.y));
}

template <class T>
inline Point3<T> permute(const Point3<T> &p, int x, int y, int z) {
    return Point3<T>(p[x], p[y], p[z]);
}

template <class T>
inline Normal3<T> normalize(const Normal3<T> &n) {
    return n / n.length();
}

template <class T>
inline T dot(const Normal3<T> &n1, const Vector3<T> &v2) {
    return n1.x * v2.x + n1.y * v2.y + n1.z * v2.z;
}

template <class T>
inline T dot(const Vector3<T> &v1, const Normal3<T> &n2) {
    return v1.x * n2.x + v1.y * n2.y + v1.z * n2.z;
}

template <class T>
inline T dot(const Normal3<T> &n1, const Normal3<T> &n2) {
    return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

template <class T>
inline T dot(const Vector3<T> &v1, const Vector3<T> &v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <class T>
inline T absDot(const Normal3<T> &n1, const Vector3<T> &v2) {
    return std::abs(n1.x * v2.x + n1.y * v2.y + n1.z * v2.z);
}

template <class T>
inline T absDot(const Vector3<T> &v1, const Normal3<T> &n2) {
    return std::abs(v1.x * n2.x + v1.y * n2.y + v1.z * n2.z);
}

template <class T>
inline T absDot(const Normal3<T> &n1, const Normal3<T> &n2) {
    return std::abs(n1.x * n2.x + n1.y * n2.y + n1.z * n2.z);
}

template <class T>
inline Normal3<T> faceforward(const Normal3<T> &n, const Vector3<T> &v) {
    return (dot(n, v) < 0.f) ? -n : n;
}

template <class T>
inline Normal3<T> faceforward(const Normal3<T> &n, const Normal3<T> &n2) {
    return (dot(n, n2) < 0.f) ? -n : n;
}

template <class T>
inline Vector3<T> faceforward(const Vector3<T> &v, const Vector3<T> &v2) {
    return (dot(v, v2) < 0.f) ? -v : v;
}

template <class T>
inline Vector3<T> faceforward(const Vector3<T> &v, const Normal3<T> &n2) {
    return (dot(v, n2) < 0.f) ? -v : v;
}

template <class T>
inline Normal3<T> abs(const Normal3<T> &v) {
    return Normal3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

inline Point3f offsetRayOrigin(const Point3f &p, const Vector3f &pError,
                               const Normal3f &n, const Vector3f &w) {
    float d = dot(abs(n), pError);

    Vector3f offset = d * Vector3f(n);
    if (dot(w, n) < 0) offset = -offset;
    Point3f po = p + offset;
    // Round offset point _po_ away from _p_
    for (int i = 0; i < 3; ++i) {
        if (offset[i] > 0)
            po[i] = nextFloatUp(po[i]);
        else if (offset[i] < 0)
            po[i] = nextFloatDown(po[i]);
    }
    return po;
}

inline Vector3f sphericalDirection(float sinTheta, float cosTheta, float phi) {
    return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

inline Vector3f sphericalDirection(float sinTheta, float cosTheta, float phi,
                                   const Vector3f &x, const Vector3f &y, const Vector3f &z) {
    return sinTheta * cos(phi) * x + sinTheta * sin(phi) * y + cosTheta * z;
}

inline float sphericalTheta(const Vector3f &v) { return acos(clamp(v.z, -1, 1)); }

inline float sphericalPhi(const Vector3f &v) {
    float p = atan2(v.y, v.x);
    return (p < 0) ? (p + 2 * PI) : p;
}

// Trigonometric functions for polar angle and azimuth
inline float cosTheta(const Vector3f &w) { return w.z; }
inline float cos2Theta(const Vector3f &w) { return SQ(w.z); }
inline float absCosTheta(const Vector3f &w) { return std::abs(w.z); }
inline float sin2Thetha(const Vector3f &w) { return std::max(0.0f, 1 - cos2Theta(w)); }
inline float sinTheta(const Vector3f &w) { return sqrt(sin2Thetha(w)); }
inline float tanTheta(const Vector3f &w) { return sinTheta(w) / cosTheta(w); }
inline float tan2Theta(const Vector3f &w) { return sin2Thetha(w) / cos2Theta(w); }

inline float cosPhi(const Vector3f &w) {
    float sin = sinTheta(w);
    return (sin == 0) ? 1 : clamp(w.x / sin, -1, 1);
}

inline float sinPhi(const Vector3f &w) {
    float sin = sinTheta(w);
    return (sin == 0) ? 0 : clamp(w.y / sin, -1, 1);
}

inline float cos2Phi(const Vector3f &w) { return SQ(cosPhi(w)); }
inline float sin2Phi(const Vector3f &w) { return SQ(sinPhi(w)); }

inline float cosDeltaPhi(const Vector3f &wa, const Vector3f &wb) {
    return clamp((wa.x * wb.x + wa.y * wb.y) / sqrt( (SQ(wa.x) + SQ(wa.y)) * (SQ(wb.x) + SQ(wb.y)) ), -1, 1);
}

inline Vector3f reflect(const Vector3f &wo, const Normal3f &n) {
    return -wo + 2 * dot(wo, n) * Vector3f(n);
}

inline bool refract(const Vector3f &wi, const Normal3f &n, float eta, Vector3f *wt) {
    float cosThetaI = dot(n, wi);
    float sin2ThetaI = std::max(0.0f, 1.0f - SQ(cosThetaI));
    float sin2ThetaT = eta * eta * sin2ThetaI;
    if (sin2ThetaT >= 1) return false;
    float cosThetaT = sqrt(1 - sin2ThetaT);
    *wt = -wi * eta + (eta * cosThetaI - cosThetaT) * Vector3f(n);
    return true;
}

}

#endif // UTILITY_VECTOR
