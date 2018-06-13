#ifndef FILTER_H
#define FILTER_H

#include "vector.h"


class Filter {
public:
    virtual ~Filter();
    Filter(const Vector2f &radius)
        : radius(radius), invRadius(Vector2f(1 / radius.x, 1 / radius.y)) {}
    virtual Float evaluate(const Point2f &p) const = 0;

    const Vector2f radius, invRadius;
};


#endif // FILTER_H
