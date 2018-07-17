#ifndef FILTER_H
#define FILTER_H

#include "vector.h"

class Filter {
public:
    Filter(const Vector2f &radius)
        : radius(radius), invRadius(Vector2f(1 / radius.x, 1 / radius.y)) {}
    virtual Float evaluate(const Point2f &p) const = 0;
    virtual ~Filter() {}

    const Vector2f radius, invRadius;
};

class BoxFilter : public Filter {
public:
    BoxFilter(const Vector2f &radius) : Filter(radius) {}
    Float evaluate(const Point2f &p) const { return 1.0f; }
};

class TriangleFilter : public Filter {
public:
    TriangleFilter(const Vector2f &radius) : Filter(radius) {}
    Float evaluate(const Point2f &p) const {
        return min(0.0f, radius.x - abs(p.x)) * min(0.0f, radius.y - abs(p.y));
    }
};


#endif // FILTER_H
