#ifndef MICROFACET_H
#define MICROFACET_H

#include "core/bsdf.h"

class OrenNayar : public BxDF {
public:
    OrenNayar(const Spectrum &R, float sigma);
    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const;

private:
    const Spectrum R;
    float A, B;
};

#endif // MICROFACET_H
