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

    Spectrum compute_L(const Interaction &intr, const Vector3f &w) const {
        return dot(intr.n, w) > 0 ? Lemit : 0.0f;
    }

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const;
    float pdf_Li(const Interaction &ref, const Vector3f &wi) const { return shape->pdf(ref, wi); }

    Spectrum power() const { return Lemit * area * PI; }

protected:
    const Spectrum Lemit;
    shared_ptr<Shape> shape;
    const float area;
};

#endif // LIGHT_DIFFUSEAREA
