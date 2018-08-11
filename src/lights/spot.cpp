#include "spot.h"
#include "paramset.h"
#include "core/sampling.h"

float SpotLight::falloff(const Vector3f &w) const {
    Vector3f wl = normalize(worldToLight(w));
    float cosTheta = wl.z;
    if (cosTheta < cosTotalWidth) return 0;
    if (cosTheta > cosFalloffStart) return 1;
    float delta = (cosTheta - cosTotalWidth) / (cosFalloffStart - cosTotalWidth);
    return QUAD(delta);
}

Spectrum SpotLight::sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray, Normal3f *nLight,
                              float *pdfPos, float *pdfDir) const
{
    Vector3f w = Sampling::uniformSampleCone(u1, cosTotalWidth);
    *ray = Ray(pLight, lightToWorld(w), INFINITY, time, mediumInterface.inside);
    *nLight = Normal3f(ray->d);
    *pdfPos = 1;
    *pdfDir = Sampling::uniformConePdf(cosTotalWidth);
    return I * falloff(ray->d);
}

void SpotLight::pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const
{
    *pdfPos = 0;
    *pdfDir = cosTheta(worldToLight(ray.d)) >= cosTotalWidth ? Sampling::uniformConePdf(cosTotalWidth) : 0;
}

shared_ptr<SpotLight> SpotLight::create(const Transform &l2w,const Medium *medium,
                                        const ParamSet &paramSet)
{
    Spectrum I = paramSet.findOneSpectrum("I", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    float coneangle = paramSet.findOneFloat("coneangle", 30.);
    float conedelta = paramSet.findOneFloat("conedeltaangle", 5.);
    // Compute spotlight world to light transformation
    Point3f from = paramSet.findOnePoint3f("from", Point3f(0, 0, 0));
    Point3f to = paramSet.findOnePoint3f("to", Point3f(0, 0, 1));
    Vector3f dir = normalize(to - from);
    Vector3f du, dv;
    coordinateSystem(dir, &du, &dv);
    Transform dirToZ = Transform(Matrix4x4(du.x, du.y, du.z, 0.,
                                           dv.x, dv.y, dv.z, 0.,
                                           dir.x, dir.y, dir.z, 0.,
                                           0, 0, 0, 1.));
    Transform light2world = l2w * Transform::translate(Vector3f(from.x, from.y, from.z)) * dirToZ.inverse();
    return make_shared<SpotLight>(light2world, medium, I * sc, coneangle, coneangle - conedelta);
}
