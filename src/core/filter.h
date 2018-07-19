#ifndef FILTER_H
#define FILTER_H

#include "vector.h"
#include "paramset.h"

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

    static BoxFilter * create(const ParamSet &ps) {
        Float xw = ps.findOneFloat("xwidth", 0.5f);
        Float yw = ps.findOneFloat("ywidth", 0.5f);
        return new BoxFilter(Vector2f(xw, yw));
    }

    Float evaluate(const Point2f &p) const { return 1.0f; }
};

class TriangleFilter : public Filter {
public:
    TriangleFilter(const Vector2f &radius) : Filter(radius) {}

    static TriangleFilter * create(const ParamSet &ps) {
        Float xw = ps.findOneFloat("xwidth", 2.f);
        Float yw = ps.findOneFloat("ywidth", 2.f);
        return new TriangleFilter(Vector2f(xw, yw));
    }

    Float evaluate(const Point2f &p) const {
        return min(0.0f, radius.x - abs(p.x)) * min(0.0f, radius.y - abs(p.y));
    }
};

class GaussianFilter : public Filter {
public:
    GaussianFilter(const Vector2f &radius, Float alpha)
        : Filter(radius), alpha(alpha),
          expX(exp(-alpha * SQ(radius.x))), expY(exp(-alpha * SQ(radius.y))) {}

    static GaussianFilter * create(const ParamSet &ps) {
        Float xw = ps.findOneFloat("xwidth", 2.f);
        Float yw = ps.findOneFloat("ywidth", 2.f);
        Float alpha = ps.findOneFloat("alpha", 2.f);
        return new GaussianFilter(Vector2f(xw, yw), alpha);
    }

    Float evaluate(const Point2f &p) const {
        return gaussian(p.x, expX) * gaussian(p.y, expY);
    }

private:
    Float gaussian(Float d, Float expv) const {
        return max(0.0f, Float(exp(-alpha * d * d) - expv));
    }

    const Float alpha;
    const Float expX, expY;
};

class MitchellFilter : public Filter {
public:
    MitchellFilter(const Vector2f &radius, Float B, Float C) : Filter(radius), B(B), C(C) {}

    static MitchellFilter * create(const ParamSet &ps) {
        Float xw = ps.findOneFloat("xwidth", 2.f);
        Float yw = ps.findOneFloat("ywidth", 2.f);
        Float B = ps.findOneFloat("B", 1.f / 3.f);
        Float C = ps.findOneFloat("C", 1.f / 3.f);
        return new MitchellFilter(Vector2f(xw, yw), B, C);
    }

    Float evaluate(const Point2f &p) const {
        return mitchell1D(p.x) * mitchell1D(p.y);
    }

private:
    Float mitchell1D(Float x) const {
        x = abs(2 * x);
        if (x > 1)
            return ((-B - 6 * C) * CUB(x) + (6 * B + 30 * C) * SQ(x) +
                    (-12 * B - 48 * C) * x + (8 * B + 24 * C)) * (1.0f/6.0f);
        else
            return ((12 - 9 * B - 6 * C) * CUB(x) + (-18 + 12 * B + 6 * C) * SQ(x) +
                    (6 - 2 * B)) * (1.0f/6.0f);
    }

    Float B, C;
};

class LanczosSincFilter : public Filter {
public:
    LanczosSincFilter(const Vector2f &radius, Float tau) : Filter(radius), tau(tau) {}

    static LanczosSincFilter * create(const ParamSet &ps) {
        Float xw = ps.findOneFloat("xwidth", 4.);
        Float yw = ps.findOneFloat("ywidth", 4.);
        Float tau = ps.findOneFloat("tau", 3.f);
        return new LanczosSincFilter(Vector2f(xw, yw), tau);
    }

    Float evaluate(const Point2f &p) const {
        return windowedSinc(p.x, radius.x) * windowedSinc(p.y, radius.y);
    }

private:
    static Float sinc(Float x) {
        x = abs(x);
        if (x < 1e-5) return 1;
        return sin(PI * x) / (PI * x);
    }

    Float windowedSinc(Float x, Float radius) const {
        x = abs(x);
        if (x > radius) return 0;
        Float lanczos = sinc(x / tau);
        return sinc(x) * lanczos;
    }

    Float tau;
};


#endif // FILTER_H
