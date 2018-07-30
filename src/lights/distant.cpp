#include "distant.h"
#include "paramset.h"

Spectrum DistantLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                 VisibilityTester *vis) const
{
    *wi = wLight;
    *pdf = 1;
    Point3f pOutside = ref.p + wLight * 2 * worldRadius;
    *vis = VisibilityTester(ref, Interaction(pOutside, ref.time, mediumInterface));
    return L;
}

shared_ptr<DistantLight> DistantLight::create(const Transform &light2world, const ParamSet &paramSet)
{
    Spectrum L = paramSet.findOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    Point3f from = paramSet.findOnePoint3f("from", Point3f(0, 0, 0));
    Point3f to = paramSet.findOnePoint3f("to", Point3f(0, 0, 1));
    Vector3f dir = from - to;
    return make_shared<DistantLight>(light2world, L * sc, dir);
}
