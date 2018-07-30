#include "spot.h"
#include "paramset.h"

float SpotLight::falloff(const Vector3f &w) const {
    Vector3f wl = normalize(worldToLight(w));
    float cosTheta = wl.z;
    if (cosTheta < cosTotalWidth) return 0;
    if (cosTheta > cosFalloffStart) return 1;
    float delta = (cosTheta - cosTotalWidth) / (cosFalloffStart - cosTotalWidth);
    return QUAD(delta);
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
