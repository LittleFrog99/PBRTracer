#ifndef LIGHT_PROJECTION
#define LIGHT_PROJECTION

#include "point.h"
#include "mipmap.h"

class ProjectionLight : public PointLight {
public:
    ProjectionLight(const Transform &lightToWorld, const MediumInterface &interface, const Spectrum &I,
                    const string &texname, float fov);

    static shared_ptr<ProjectionLight> create(const Transform &light2world,const Medium *medium,
                                                    const ParamSet &paramSet);

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const {
        Spectrum Li = PointLight::sample_Li(ref, u, wi, pdf, vis);
        return Li * projection(-*wi);
    }

    float pdf_Li(const Interaction &, const Vector3f &) const { return 0; }

    Spectrum power() const {
        return (projMap ? projMap->lookup(Point2f(0.5f, 0.5f), 0.5f) : Spectrum(1.0f)) *
                I * 2 * PI * (1.0f - cosTotalWidth);
    }

private:
    Spectrum projection(const Vector3f &wi) const;

    unique_ptr<Mipmap<RGBSpectrum>> projMap;
    Transform lightProj;
    float near, far;
    Bounds2f screenBounds;
    float cosTotalWidth;
};

#endif // LIGHT_PROJECTION
