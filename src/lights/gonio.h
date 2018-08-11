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
                       VisibilityTester *vis) const;
    float pdf_Li(const Interaction &, const Vector3f &) const { return 0; }

    Spectrum power() const;

    Spectrum sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray, Normal3f *nLight,
                       float *pdfPos, float *pdfDir) const;
    void pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const;

private:
    Spectrum scale(const Vector3f &w) const;

    unique_ptr<Mipmap<RGBSpectrum>> mipmap;
};

#endif // LIGHT_GONIO
