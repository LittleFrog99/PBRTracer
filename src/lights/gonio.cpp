#include "gonio.h"
#include "imageio.h"
#include "paramset.h"
#include "core/sampling.h"

GonioPhotometricLight::GonioPhotometricLight(const Transform &lightToWorld, const MediumInterface &interface,
                                             const Spectrum &I, const string &texname)
    : PointLight(lightToWorld, interface, I)
{
    Point2i resolution;
    auto texels = ImageIO::readImage(texname, &resolution);
    if (texels)
        mipmap.reset(new Mipmap<RGBSpectrum>(resolution, texels.get()));
}

Spectrum GonioPhotometricLight::power() const {
    return 4 * PI * I * (mipmap ? mipmap->lookup(Point2f(0.5f, 0.5f), 0.5f) : 1.0f);
}

Spectrum GonioPhotometricLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                          VisibilityTester *vis) const
{
    Spectrum Li = PointLight::sample_Li(ref, u, wi, pdf, vis);
    return Li * scale(-*wi);
}

Spectrum GonioPhotometricLight::sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray,
                                          Normal3f *nLight, float *pdfPos, float *pdfDir) const
{
    *ray = Ray(pLight, Sampling::uniformSampleSphere(u1), INFINITY, time, mediumInterface.inside);
    *nLight = Normal3f(ray->d);
    *pdfPos = 1; // delta distribution
    *pdfDir = Sampling::uniformSpherePdf();
    return I * scale(ray->d);
}

void GonioPhotometricLight::pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const
{
    *pdfPos = 0;
    *pdfDir = Sampling::uniformSpherePdf();
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

