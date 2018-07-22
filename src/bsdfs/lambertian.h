#ifndef LAMBERTIAN_H
#define LAMBERTIAN_H

#include "core/bsdf.h"

class LambertianReflection : public BxDF {
public:
    LambertianReflection(Spectrum &R)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R) {}

    Spectrum compute_f(const Vector3f &, const Vector3f &) const { return R * INV_PI; }
    Spectrum rho_hd(const Vector3f &, int, const Point2f *) const { return R; }
    Spectrum rho_hh(int, const Point2f *, const Point2f *) const { return R; }

    string toString() const { return "[ LambertianReflection R: " + R.toString() + " ]"; }

private:
    Spectrum R;
};

class LambertianTransmission : public BxDF {
public:
    LambertianTransmission(Spectrum &T)
        : BxDF(BxDFType(BSDF_TRANSMISSION | BSDF_DIFFUSE)), T(T) {}

    Spectrum compute_f(const Vector3f &, const Vector3f &) const { return T * INV_PI; }
    Spectrum rho_hd(const Vector3f &, int, const Point2f *) const { return T; }
    Spectrum rho_hh(int, const Point2f *, const Point2f *) const { return T; }

    string toString() const { return "[ LambertianTransmission T: " + T.toString() + " ]"; }

private:
    Spectrum T;
};

#endif // LAMBERTIAN_H
