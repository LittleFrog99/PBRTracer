#ifndef FILTER_H
#define FILTER_H

#include "vector.h"
#include "paramset.h"

class Filter {
public:
    Filter(const Vector2f &radius)
        : radius(radius), invRadius(Vector2f(1 / radius.x, 1 / radius.y)) {}
    virtual float evaluate(const Point2f &p) const = 0;
    virtual ~Filter() {}

    const Vector2f radius, invRadius;
};

class BoxFilter : public Filter {
public:
    BoxFilter(const Vector2f &radius) : Filter(radius) {}

    static BoxFilter * create(const ParamSet &ps) {
        float xw = ps.findOneFloat("xwidth", 0.5f);
        float yw = ps.findOneFloat("ywidth", 0.5f);
        return new BoxFilter(Vector2f(xw, yw));
    }

    float evaluate(const Point2f &p) const { return 1.0f; }
};

class TriangleFilter : public Filter {
public:
    TriangleFilter(const Vector2f &radius) : Filter(radius) {}

    static TriangleFilter * create(const ParamSet &ps) {
        float xw = ps.findOneFloat("xwidth", 2.f);
        float yw = ps.findOneFloat("ywidth", 2.f);
        return new TriangleFilter(Vector2f(xw, yw));
    }

    float evaluate(const Point2f &p) const {
        return min(0.0f, radius.x - abs(p.x)) * min(0.0f, radius.y - abs(p.y));
    }
};

class GaussianFilter : public Filter {
public:
    GaussianFilter(const Vector2f &radius, float alpha)
        : Filter(radius), alpha(alpha),
          expX(exp(-alpha * SQ(radius.x))), expY(exp(-alpha * SQ(radius.y))) {}

    static GaussianFilter * create(const ParamSet &ps) {
        float xw = ps.findOneFloat("xwidth", 2.f);
        float yw = ps.findOneFloat("ywidth", 2.f);
        float alpha = ps.findOneFloat("alpha", 2.f);
        return new GaussianFilter(Vector2f(xw, yw), alpha);
    }

    float evaluate(const Point2f &p) const {
        return gaussian(p.x, expX) * gaussian(p.y, expY);
    }

private:
    float gaussian(float d, float expv) const {
        return max(0.0f, float(exp(-alpha * d * d) - expv));
    }

    const float alpha;
    const float expX, expY;
};

class MitchellFilter : public Filter {
public:
    MitchellFilter(const Vector2f &radius, float B, float C) : Filter(radius), B(B), C(C) {}

    static MitchellFilter * create(const ParamSet &ps) {
        float xw = ps.findOneFloat("xwidth", 2.f);
        float yw = ps.findOneFloat("ywidth", 2.f);
        float B = ps.findOneFloat("B", 1.f / 3.f);
        float C = ps.findOneFloat("C", 1.f / 3.f);
        return new MitchellFilter(Vector2f(xw, yw), B, C);
    }

    float evaluate(const Point2f &p) const {
        return mitchell1D(p.x) * mitchell1D(p.y);
    }

private:
    float mitchell1D(float x) const {
        x = abs(2 * x);
        if (x > 1)
            return ((-B - 6 * C) * CUB(x) + (6 * B + 30 * C) * SQ(x) +
                    (-12 * B - 48 * C) * x + (8 * B + 24 * C)) * (1.0f/6.0f);
        else
            return ((12 - 9 * B - 6 * C) * CUB(x) + (-18 + 12 * B + 6 * C) * SQ(x) +
                    (6 - 2 * B)) * (1.0f/6.0f);
    }

    float B, C;
};

class LanczosSincFilter : public Filter {
public:
    LanczosSincFilter(const Vector2f &radius, float tau) : Filter(radius), tau(tau) {}

    static LanczosSincFilter * create(const ParamSet &ps) {
        float xw = ps.findOneFloat("xwidth", 4.);
        float yw = ps.findOneFloat("ywidth", 4.);
        float tau = ps.findOneFloat("tau", 3.f);
        return new LanczosSincFilter(Vector2f(xw, yw), tau);
    }

    float evaluate(const Point2f &p) const {
        return windowedSinc(p.x, radius.x) * windowedSinc(p.y, radius.y);
    }

private:
    static float sinc(float x) {
        x = abs(x);
        if (x < 1e-5) return 1;
        return sin(PI * x) / (PI * x);
    }

    float windowedSinc(float x, float radius) const {
        x = abs(x);
        if (x > radius) return 0;
        float lanczos = sinc(x / tau);
        return sinc(x) * lanczos;
    }

    float tau;
};


#endif // FILTER_H
