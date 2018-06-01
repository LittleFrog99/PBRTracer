#ifndef VECTOR_H
#define VECTOR_H

#include "stringprint.h"

// Forward Declarations
template <typename T>
class Vector2;
template <typename T>
class Vector3;
template <typename T>
class Point3;
template <typename T>
class Point2;
template <typename T>
class Normal3;

// Vector Declarations
template <typename T>
class Vector2 {
public:
    // Vector2 Public Methods
    inline Vector2() { x = y = 0; }
    inline Vector2(T xx, T yy) : x(xx), y(yy) { }
    inline bool hasNaNs() const { return isNaN(x) || isNaN(y); }
    inline explicit Vector2(const Point2<T> &p);
    inline explicit Vector2(const Point3<T> &p);

    inline Vector2(const Vector2<T> &v) { x = v.x; y = v.y; }
    inline Vector2<T> & operator = (const Vector2<T> &v) { x = v.x; y = v.y; return *this; }
    inline Vector2<T> operator + (const Vector2<T> &v) const { return Vector2(x + v.x, y + v.y); }
    inline Vector2<T> & operator += (const Vector2<T> &v) { x += v.x; y += v.y; return *this; }
    inline Vector2<T> operator - (const Vector2<T> &v) const { return Vector2(x - v.x, y - v.y); }

    inline Vector2<T> & operator -= (const Vector2<T> &v) { x -= v.x; y -= v.y; return *this; }
    inline bool operator == (const Vector2<T> &v) const { return x == v.x && y == v.y; }
    inline bool operator != (const Vector2<T> &v) const { return x != v.x || y != v.y; }

    template <typename U>
    inline Vector2<T> operator * (U f) const { return Vector2<T>(f * x, f * y); }

    template <typename U>
    inline Vector2<T> &operator*=(U f) { x *= f; y *= f; return *this; }

    template <typename U>
    inline Vector2<T> operator / (U f) const {
        Float inv = (Float)1 / f;
        return Vector2<T>(x * inv, y * inv);
    }

    template <typename U>
    inline Vector2<T> & operator /= (U f) { Float inv = 1.0 / f; x *= inv; y *= inv; return *this; }

    inline Vector2<T> operator - () const { return Vector2<T>(-x, -y); }
    inline T operator [] (int i) const { if (i == 0) return x; return y; }
    inline T & operator [] (int i) { if (i == 0) return x; return y; }
    inline Float lengthSq() const { return x * x + y * y; }
    inline Float length() const { return std::sqrt(lengthSq()); }

    // Vector2 Public Data
    T x, y;
};

template <typename T>
inline ostream & operator << (ostream &os, const Vector2<T> &v) {
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

inline ostream & operator << (ostream &os, const Vector2<Float> &v) {
    os << StringPrint::printf("[ %f, %f ]", v.x, v.y);
    return os;
}

template <typename T>
class Vector3 {
public:
    // Vector3 Public Methods
    Vector3() { x = y = z = 0; }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
    bool hasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }
    explicit Vector3(const Point3<T> &p);

    Vector3(const Vector3<T> &v) {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    Vector3<T> &operator=(const Vector3<T> &v) {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }
    Vector3<T> operator+(const Vector3<T> &v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    Vector3<T> &operator+=(const Vector3<T> &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Vector3<T> operator-(const Vector3<T> &v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    Vector3<T> &operator-=(const Vector3<T> &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    bool operator==(const Vector3<T> &v) const {
        return x == v.x && y == v.y && z == v.z;
    }
    bool operator!=(const Vector3<T> &v) const {
        return x != v.x || y != v.y || z != v.z;
    }
    template <typename U>
    Vector3<T> operator*(U s) const {
        return Vector3<T>(s * x, s * y, s * z);
    }
    template <typename U>
    Vector3<T> &operator*=(U s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    template <typename U>
    Vector3<T> operator/(U f) const {
        Float inv = (Float)1 / f;
        return Vector3<T>(x * inv, y * inv, z * inv);
    }

    template <typename U>
    Vector3<T> &operator/=(U f) {
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); }

    T operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
    T &operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
    Float lengthSq() const { return x * x + y * y + z * z; }
    Float length() const { return std::sqrt(lengthSq()); }
    explicit Vector3(const Normal3<T> &n);

    // Vector3 Public Data
    T x, y, z;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Vector3<T> &v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Vector3<Float> &v) {
    os << StringPrint::printf("[ %f, %f, %f ]", v.x, v.y, v.z);
    return os;
}

typedef Vector2<Float> Vector2f;
typedef Vector2<int> Vector2i;
typedef Vector3<Float> Vector3f;
typedef Vector3<int> Vector3i;

// Point Declarations
template <typename T>
class Point2 {
  public:
    // Point2 Public Methods
    explicit Point2(const Point3<T> &p) : x(p.x), y(p.y) { }
    Point2() { x = y = 0; }
    Point2(T xx, T yy) : x(xx), y(yy) { }

    template <typename U>
    explicit Point2(const Point2<U> &p) {
        x = (T)p.x;
        y = (T)p.y;
    }

    template <typename U>
    explicit Point2(const Vector2<U> &p) {
        x = (T)p.x;
        y = (T)p.y;
    }

    template <typename U>
    explicit operator Vector2<U>() const {
        return Vector2<U>(x, y);
    }

#ifndef NDEBUG
    Point2(const Point2<T> &p) {
        x = p.x;
        y = p.y;
    }

    Point2<T> &operator=(const Point2<T> &p) {
        x = p.x;
        y = p.y;
        return *this;
    }
#endif  // !NDEBUG
    Point2<T> operator+(const Vector2<T> &v) const {
        return Point2<T>(x + v.x, y + v.y);
    }

    Point2<T> &operator+=(const Vector2<T> &v) {
        x += v.x;
        y += v.y;
        return *this;
    }
    Vector2<T> operator-(const Point2<T> &p) const {
        return Vector2<T>(x - p.x, y - p.y);
    }

    Point2<T> operator-(const Vector2<T> &v) const {
        return Point2<T>(x - v.x, y - v.y);
    }
    Point2<T> operator-() const { return Point2<T>(-x, -y); }
    Point2<T> &operator-=(const Vector2<T> &v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }
    Point2<T> &operator+=(const Point2<T> &p) {
        x += p.x;
        y += p.y;
        return *this;
    }
    Point2<T> operator+(const Point2<T> &p) const {
        return Point2<T>(x + p.x, y + p.y);
    }
    template <typename U>
    Point2<T> operator*(U f) const {
        return Point2<T>(f * x, f * y);
    }
    template <typename U>
    Point2<T> &operator*=(U f) {
        x *= f;
        y *= f;
        return *this;
    }
    template <typename U>
    Point2<T> operator/(U f) const {
        Float inv = (Float)1 / f;
        return Point2<T>(inv * x, inv * y);
    }
    template <typename U>
    Point2<T> &operator/=(U f) {
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        return *this;
    }
    T operator[](int i) const {
        if (i == 0) return x;
        return y;
    }

    T &operator[](int i) {
        if (i == 0) return x;
        return y;
    }
    bool operator==(const Point2<T> &p) const { return x == p.x && y == p.y; }
    bool operator!=(const Point2<T> &p) const { return x != p.x || y != p.y; }
    bool hasNaNs() const { return isNaN(x) || isNaN(y); }

    // Point2 Public Data
    T x, y;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Point2<T> &v) {
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Point2<Float> &v) {
    os << StringPrint::printf("[ %f, %f ]", v.x, v.y);
    return os;
}

template <typename T>
class Point3 {
  public:
    // Point3 Public Methods
    Point3() { x = y = z = 0; }
    Point3(T x, T y, T z) : x(x), y(y), z(z) { }
    template <typename U>
    explicit Point3(const Point3<U> &p)
        : x((T)p.x), y((T)p.y), z((T)p.z) {
    }
    template <typename U>
    explicit operator Vector3<U>() const {
        return Vector3<U>(x, y, z);
    }
    Point3(const Point3<T> &p) {
        x = p.x;
        y = p.y;
        z = p.z;
    }

    Point3<T> &operator=(const Point3<T> &p) {
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
    Point3<T> operator+(const Vector3<T> &v) const {
        return Point3<T>(x + v.x, y + v.y, z + v.z);
    }
    Point3<T> &operator+=(const Vector3<T> &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Vector3<T> operator-(const Point3<T> &p) const {
        return Vector3<T>(x - p.x, y - p.y, z - p.z);
    }
    Point3<T> operator-(const Vector3<T> &v) const {
        return Point3<T>(x - v.x, y - v.y, z - v.z);
    }
    Point3<T> &operator-=(const Vector3<T> &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    Point3<T> &operator+=(const Point3<T> &p) {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }
    Point3<T> operator+(const Point3<T> &p) const {
        return Point3<T>(x + p.x, y + p.y, z + p.z);
    }
    template <typename U>
    Point3<T> operator*(U f) const {
        return Point3<T>(f * x, f * y, f * z);
    }
    template <typename U>
    Point3<T> &operator*=(U f) {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }
    template <typename U>
    Point3<T> operator/(U f) const {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Point3<T>(inv * x, inv * y, inv * z);
    }
    template <typename U>
    Point3<T> &operator/=(U f) {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    T operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    T &operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    bool operator==(const Point3<T> &p) const {
        return x == p.x && y == p.y && z == p.z;
    }
    bool operator!=(const Point3<T> &p) const {
        return x != p.x || y != p.y || z != p.z;
    }
    bool hasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }
    Point3<T> operator-() const { return Point3<T>(-x, -y, -z); }

    // Point3 Public Data
    T x, y, z;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Point3<T> &v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Point3<Float> &v) {
    os << StringPrint::printf("[ %f, %f, %f ]", v.x, v.y, v.z);
    return os;
}

typedef Point2<Float> Point2f;
typedef Point2<int> Point2i;
typedef Point3<Float> Point3f;
typedef Point3<int> Point3i;

// Normal Declarations
template <typename T>
class Normal3 {
  public:
    // Normal3 Public Methods
    Normal3() { x = y = z = 0; }
    Normal3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) { }
    Normal3<T> operator-() const { return Normal3(-x, -y, -z); }
    Normal3<T> operator+(const Normal3<T> &n) const {
        return Normal3<T>(x + n.x, y + n.y, z + n.z);
    }

    Normal3<T> &operator+=(const Normal3<T> &n) {
        x += n.x;
        y += n.y;
        z += n.z;
        return *this;
    }
    Normal3<T> operator-(const Normal3<T> &n) const {
        return Normal3<T>(x - n.x, y - n.y, z - n.z);
    }

    Normal3<T> &operator-=(const Normal3<T> &n) {
        x -= n.x;
        y -= n.y;
        z -= n.z;
        return *this;
    }
    bool hasNaNs() const { return isNaN(x) || isNaN(y) || isNaN(z); }
    template <typename U>
    Normal3<T> operator*(U f) const {
        return Normal3<T>(f * x, f * y, f * z);
    }

    template <typename U>
    Normal3<T> &operator*=(U f) {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }
    template <typename U>
    Normal3<T> operator/(U f) const {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        return Normal3<T>(x * inv, y * inv, z * inv);
    }

    template <typename U>
    Normal3<T> &operator/=(U f) {
        CHECK_NE(f, 0);
        Float inv = (Float)1 / f;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }
    Float lengthSq() const { return x * x + y * y + z * z; }
    Float length() const { return std::sqrt(lengthSq()); }

    Normal3<T>(const Normal3<T> &n) {
        x = n.x;
        y = n.y;
        z = n.z;
    }

    Normal3<T> &operator=(const Normal3<T> &n) {
        x = n.x;
        y = n.y;
        z = n.z;
        return *this;
    }

    explicit Normal3<T>(const Vector3<T> &v) : x(v.x), y(v.y), z(v.z) {
    }
    bool operator==(const Normal3<T> &n) const {
        return x == n.x && y == n.y && z == n.z;
    }
    bool operator!=(const Normal3<T> &n) const {
        return x != n.x || y != n.y || z != n.z;
    }

    T operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    T &operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    // Normal3 Public Data
    T x, y, z;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &os, const Normal3<T> &v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

template <>
inline std::ostream &operator<<(std::ostream &os, const Normal3<Float> &v) {
    os << StringPrint::printf("[ %f, %f, %f ]", v.x, v.y, v.z);
    return os;
}

typedef Normal3<Float> Normal3f;

#endif // VECTOR_H
