#ifndef LIGHT_SPOT
#define LIGHT_SPOT

#include "point.h"

class SpotLight : public PointLight {
public:
    SpotLight(const Transform &lightToWorld, const MediumInterface &interface, const Spectrum &I,
              float totalWidth, float falloffStart)
        : PointLight(lightToWorld, interface, I),
          cosTotalWidth(cos(radians(totalWidth))), cosFalloffStart(cos(radians(falloffStart))) {}

    static shared_ptr<SpotLight> create(const Transform &light2world,const Medium *medium,
                                        const ParamSet &paramSet);

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const {
        Spectrum Li = PointLight::sample_Li(ref, u, wi, pdf, vis);
        return Li * falloff(-*wi);
    }

    float pdf_Li(const Interaction &, const Vector3f &) const { return 0; }

    Spectrum power() const { return I * 2 * PI * (1.0f - 0.5f * (cosFalloffStart + cosTotalWidth)); }

private:
    float falloff(const Vector3f &w) const;

    const float cosTotalWidth, cosFalloffStart;
};

#endif // LIGHT_SPOT
