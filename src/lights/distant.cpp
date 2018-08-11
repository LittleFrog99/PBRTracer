#include "distant.h"
#include "paramset.h"
#include "core/sampling.h"

void DistantLight::preprocess(const Scene &scene) {
    scene.getWorldBound().boundingSphere(&worldCenter, &worldRadius);
}

Spectrum DistantLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                 VisibilityTester *vis) const
{
    *wi = wLight;
    *pdf = 1;
    Point3f pOutside = ref.p + wLight * 2 * worldRadius;
    *vis = VisibilityTester(ref, Interaction(pOutside, ref.time, mediumInterface));
    return L;
}

Spectrum DistantLight::sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray, Normal3f *nLight,
                                 float *pdfPos, float *pdfDir) const
{
    // Choose point on disk oriented toward infinite direction
    Vector3f v1, v2;
    coordinateSystem(wLight, &v1, &v2);
    Point2f cd = Sampling::concentricSampleDisk(u1);
    Point3f pDisk = worldCenter + worldRadius * (cd.x * v1 + cd.y * v2);

    *ray = Ray(pDisk + worldRadius * wLight, -wLight, INFINITY, time);
    *nLight = Normal3f(ray->d);
    *pdfPos = 1 / (PI * SQ(worldRadius));
    *pdfDir = 1;
    return L;
}

void DistantLight::pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const
{
    *pdfPos = 1 / (PI * SQ(worldRadius));
    *pdfDir = 0;
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
