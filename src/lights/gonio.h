#ifndef LIGHT_GONIO
#define LIGHT_GONIO

#include "point.h"
#include "mipmap.h"

class GonioPhotometricLight : public PointLight {
public:
    GonioPhotometricLight(const Transform &lightToWorld, const MediumInterface &interface, const Spectrum &I,
                          const string  &texname);

    static shared_ptr<GonioPhotometricLight> create(const Transform &light2world,const Medium *medium,
                                                    const ParamSet &paramSet);

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const {
        Spectrum Li = PointLight::sample_Li(ref, u, wi, pdf, vis);
        return Li * scale(-*wi);
    }

    float pdf_Li(const Interaction &, const Vector3f &) const { return 0; }

    Spectrum power() const {
        return 4 * PI * I * (mipmap ? mipmap->lookup(Point2f(0.5f, 0.5f), 0.5f) : 1.0f);
    }

private:
    Spectrum scale(const Vector3f &w) const;

    unique_ptr<Mipmap<RGBSpectrum>> mipmap;
};

#endif // LIGHT_GONIO
