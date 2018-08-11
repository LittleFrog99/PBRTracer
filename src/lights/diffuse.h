#ifndef LIGHT_DIFFUSEAREA
#define LIGHT_DIFFUSEAREA

#include "core/light.h"
#include "core/shape.h"

class DiffuseAreaLight : public AreaLight {
public:
    DiffuseAreaLight(const Transform &lightToWorld, const MediumInterface &interface,
                     const Spectrum &Lemit, int nSamples, const shared_ptr<Shape> &shape)
        : AreaLight(lightToWorld, interface, nSamples),
          Lemit(Lemit), shape(shape), area(shape->area()) {}

    static shared_ptr<DiffuseAreaLight> create(const Transform &light2world, const Medium *medium,
                                               const ParamSet &paramSet, const shared_ptr<Shape> &shape);

    Spectrum compute_L(const Interaction &intr, const Vector3f &w) const;

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const;
    float pdf_Li(const Interaction &ref, const Vector3f &wi) const { return shape->pdf(ref, wi); }

    Spectrum power() const { return Lemit * area * PI; }

    Spectrum sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray, Normal3f *nLight,
                       float *pdfPos, float *pdfDir) const;
    void pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const;

protected:
    const Spectrum Lemit;
    shared_ptr<Shape> shape;
    const float area;
};

#endif // LIGHT_DIFFUSEAREA
