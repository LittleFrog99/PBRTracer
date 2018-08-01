#include "diffuse.h"
#include "core/renderer.h"

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

Spectrum DiffuseAreaLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                     VisibilityTester *vis) const {
    Interaction pShape = shape->sample(ref, u, pdf);
    pShape.mediumInterface = mediumInterface;
    *wi = normalize(pShape.p - ref.p);
    *vis = VisibilityTester(ref, pShape);
    return compute_L(pShape, -*wi);
}
