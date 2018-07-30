#include "gonio.h"
#include "imageio.h"
#include "paramset.h"

GonioPhotometricLight::GonioPhotometricLight(const Transform &lightToWorld, const MediumInterface &interface,
                                             const Spectrum &I, const string &texname)
    : PointLight(lightToWorld, interface, I)
{
    Point2i resolution;
    auto texels = ImageIO::readImage(texname, &resolution);
    if (texels)
        mipmap.reset(new Mipmap<RGBSpectrum>(resolution, texels.get()));
}

Spectrum GonioPhotometricLight::scale(const Vector3f &w) const {
    Vector3f wp = normalize(worldToLight(w));
    swap(wp.y, wp.z);
    float theta = sphericalTheta(wp);
    float phi = sphericalPhi(wp);
    Point2f st(phi * INV_TWO_PI, theta * INV_PI);
    return mipmap ? mipmap->lookup(st) : 1.0f;
}

shared_ptr<GonioPhotometricLight>
GonioPhotometricLight::create(const Transform &light2world,const Medium *medium, const ParamSet &paramSet)
{
    Spectrum I = paramSet.findOneSpectrum("I", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    string texname = paramSet.findOneFilename("mapname", "");
    return make_shared<GonioPhotometricLight>(light2world, medium, I * sc, texname);
}
