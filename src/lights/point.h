#ifndef LIGHT_POINT
#define LIGHT_POINT

#include "core/light.h"

class PointLight : public Light {
public:
    PointLight(const Transform &lightToWorld, const MediumInterface &interface, const Spectrum &I)
        : Light(int(LightFlags::DeltaPosition), lightToWorld, interface),
          pLight(lightToWorld(Point3f())), I(I) {}

    static shared_ptr<PointLight> create(const Transform &light2world,const Medium *medium,
                                         const ParamSet &paramSet);

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const;

    float pdf_Li(const Interaction &, const Vector3f &) const { return 0; }

    Spectrum power() const { return 4 * PI * I; }

protected:
    const Point3f pLight;
    const Spectrum I; // radiant intensity
};

#endif // POINT_H
