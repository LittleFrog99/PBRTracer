#include "point.h"
#include "paramset.h"

Spectrum PointLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                               VisibilityTester *vis) const
{
    *wi = normalize(pLight - ref.p);
    *pdf = 1.0f;
    *vis = VisibilityTester(ref, Interaction(pLight, ref.time, mediumInterface));
    return I / distanceSq(pLight, ref.p);
}

shared_ptr<PointLight> PointLight::create(const Transform &light2world,const Medium *medium,
                                          const ParamSet &paramSet)
{
    Spectrum I = paramSet.findOneSpectrum("I", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    Point3f P = paramSet.findOnePoint3f("from", Point3f(0, 0, 0));
    Transform l2w = Transform::translate(Vector3f(P.x, P.y, P.z)) * light2world;
    return make_shared<PointLight>(l2w, medium, I * sc);
}
