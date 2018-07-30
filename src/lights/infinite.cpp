#include "infinite.h"
#include "imageio.h"
#include "core/renderer.h"

InfiniteAreaLight::InfiniteAreaLight(const Transform &lightToWorld, const Spectrum &Lemit, int nSamples,
                                     const string &texmap)
    : Light(int(LightFlags::Infinite), lightToWorld, MediumInterface(), nSamples)
{
    // Read data from _texmap_
    Point2i resolution;
    auto texels = ImageIO::readImage(texmap, &resolution);
    Lmap.reset(new Mipmap<RGBSpectrum>(resolution, texels.get()));

    // Initialize sampling PDFs
}

Spectrum InfiniteAreaLight::compute_Le(const RayDifferential &ray) const {
    Vector3f w = normalize(worldToLight(ray.d));
    Point2f st(sphericalPhi(w) * INV_TWO_PI, sphericalTheta(w) * INV_PI);
    return Lmap->lookup(st);
}

shared_ptr<InfiniteAreaLight> InfiniteAreaLight::create(const Transform &light2world, const ParamSet &paramSet)
{
    Spectrum L = paramSet.findOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    std::string texmap = paramSet.findOneFilename("mapname", "");
    int nSamples = paramSet.findOneInt("samples", paramSet.findOneInt("nsamples", 1));
    if (Renderer::options.quickRender) nSamples = max(1, nSamples / 4);
    return make_shared<InfiniteAreaLight>(light2world, L * sc, nSamples, texmap);
}
