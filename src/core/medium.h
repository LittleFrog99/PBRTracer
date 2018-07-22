#ifndef CORE_MEDIUM
#define CORE_MEDIUM

#include "ray.h"
#include "spectrum.h"
#include "memory.h"
#include "stringprint.h"

class MediumInteraction;
class Sampler;

class PhaseFunction {
public:
    virtual ~PhaseFunction();
    virtual Float compute_p(const Vector3f &wo, const Vector3f &wi) const = 0;
    virtual Float sample_p(const Vector3f &wo, Vector3f *wi, const Point2f &u) const = 0;
    virtual string toString() const = 0;
};

inline ostream & operator << (ostream &os, const PhaseFunction &p) {
    os << p.toString();
    return os;
}

class Medium {
public:
    virtual ~Medium() {}
    virtual Spectrum compute_Tr(const Ray &ray, Sampler &sampler) const = 0;
    virtual Spectrum sample(const Ray &ray, Sampler &sampler, MemoryArena &arena,
                            MediumInteraction *mi) const = 0;

    static bool getMediumScatteringProperties(const string &name, Spectrum *sigma_a, Spectrum *sigma_s);
};

class HenyeyGreenstein : public PhaseFunction {
public:
    HenyeyGreenstein(Float g) : g(g) {}
    Float compute_p(const Vector3f &wo, const Vector3f &wi) const;
    Float sample_p(const Vector3f &wo, Vector3f *wi, const Point2f &sample) const;
    string toString() const {
        return STRING_PRINTF("[ HenyeyGreenstein g: %f ]", g);
    }

    inline static Float phase(Float cosTheta, Float g) {
        Float denom = 1 + g * g + 2 * g * cosTheta;
        return INV_PI * (1 - g * g) / (denom * sqrt(denom));
    }

private:
    const Float g;
};

struct MediumInterface {
    MediumInterface() : inside(nullptr), outside(nullptr) {}
    MediumInterface(const Medium *medium) : inside(medium), outside(medium) {}
    MediumInterface(const Medium *inside, const Medium *outside)
        : inside(inside), outside(outside) {}
    bool isMediumTransition() const { return inside != outside; }

    const Medium *inside, *outside;
};

#endif // CORE_MEDIUM
