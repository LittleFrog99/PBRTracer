#include "diffuse.h"
#include "core/renderer.h"
#include "core/sampling.h"
#include "stats.h"

Spectrum DiffuseAreaLight::compute_L(const Interaction &intr, const Vector3f &w) const {
    return dot(intr.n, w) > 0 ? Lemit : 0.0f;
}

Spectrum DiffuseAreaLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                     VisibilityTester *vis) const {
    Interaction pShape = shape->sample(ref, u, pdf);
    pShape.mediumInterface = mediumInterface;
    *wi = normalize(pShape.p - ref.p);
    *vis = VisibilityTester(ref, pShape);
    return compute_L(pShape, -*wi);
}

Spectrum DiffuseAreaLight::sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray,
                                     Normal3f *nLight, float *pdfPos, float *pdfDir) const
{
    ProfilePhase _(Stage::LightSample);
    // Sample a point on the shape
    Interaction pShape = shape->sample(u1, pdfPos);
    pShape.mediumInterface = this->mediumInterface;
    *nLight = pShape.n;

    // Sample out-going direction
    Vector3f w = Sampling::cosineSampleHemisphere(u2);
    *pdfDir = Sampling::cosineHemispherePdf(cosTheta(w));
    Vector3f v1, v2, n(pShape.n);
    coordinateSystem(n, &v1, &v2);
    w = w.x * v1 + w.y * v2 + w.z * n;

    *ray = pShape.spawnRay(w);
    return compute_L(pShape, w);
}

void DiffuseAreaLight::pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const
{
    ProfilePhase _(Stage::LightPdf);
    Interaction itr(ray.o, nLight, Vector3f(), Vector3f(nLight), ray.time, mediumInterface);
    *pdfPos = shape->pdf(itr);
    *pdfDir = Sampling::cosineHemispherePdf(dot(ray.d, nLight));
}

shared_ptr<DiffuseAreaLight> DiffuseAreaLight::create(const Transform &light2world, const Medium *medium,
                                                      const ParamSet &paramSet, const shared_ptr<Shape> &shape)
{
    Spectrum L = paramSet.findOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.findOneInt("samples", paramSet.findOneInt("nsamples", 1));
    // bool twoSided = paramSet.findOneBool("twosided", false);
    if (Renderer::options.quickRender) nSamples = max(1, nSamples / 4);
    return make_shared<DiffuseAreaLight>(light2world, medium, L * sc, nSamples, shape);
}
