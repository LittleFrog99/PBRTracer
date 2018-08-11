#include "infinite.h"
#include "imageio.h"
#include "core/renderer.h"
#include "core/sampling.h"

InfiniteAreaLight::InfiniteAreaLight(const Transform &lightToWorld, const Spectrum &Lemit, int nSamples,
                                     const string &texmap)
    : Light(int(LightFlags::Infinite), lightToWorld, MediumInterface(), nSamples)
{
    // Read data from _texmap_
    Point2i resolution;
    auto texels = ImageIO::readImage(texmap, &resolution);
    Lmap.reset(new Mipmap<RGBSpectrum>(resolution, texels.get()));

    // Initialize sampling PDFs
    // Compute scalar-valued image _img_ from environment map
    int width = 2 * Lmap->width(), height = 2 * Lmap->height();
    unique_ptr<float[]> img(new float[width * height]);
    float fwidth = 0.5f / min(width, height);
    Parallel::forLoop(
        [&](int64_t v) {
            float vp = (v + .5f) / float(height);
            float sinTheta = sin(PI * (v + .5f) / height);
            for (int u = 0; u < width; ++u) {
                float up = (u + .5f) / float(width);
                img[u + v * width] = Lmap->lookup(Point2f(up, vp), fwidth).luminance();
                img[u + v * width] *= sinTheta;
            }
        },
        height, 32);

    // Compute sampling distributions for rows and columns of image
    distrib.reset(new Distribution2D(img.get(), width, height));
}

void InfiniteAreaLight::preprocess(const Scene &scene) {
    scene.getWorldBound().boundingSphere(&worldCenter, &worldRadius);
}

Spectrum InfiniteAreaLight::compute_Le(const RayDifferential &ray) const {
    Vector3f w = normalize(worldToLight(ray.d));
    Point2f st(sphericalPhi(w) * INV_TWO_PI, sphericalTheta(w) * INV_PI);
    return Lmap->lookup(st);
}

Spectrum InfiniteAreaLight::power() const {
    return Lmap->lookup(Point2f(0.5f, 0.5f), 0.5f) * PI * SQ(worldRadius);
}

Spectrum InfiniteAreaLight::sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray,
                                      Normal3f *nLight, float *pdfPos, float *pdfDir) const
{
    // Find (u,v) sample coordinates in infinite light texture
    float mapPdf;
    Point2f uv = distrib->sampleContinuous(u1, &mapPdf);
    if (mapPdf == 0) return Spectrum(0.f);
    float theta = uv[1] * PI, phi = uv[0] * 2.f * PI;
    float cosTheta = cos(theta), sinTheta = sin(theta);
    float sinPhi = sin(phi), cosPhi = cos(phi);
    Vector3f d = -lightToWorld(Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta));
    *nLight = Normal3f(d);

    // Compute origin for infinite light sample ray
    Vector3f v1, v2;
    coordinateSystem(-d, &v1, &v2);
    Point2f cd = Sampling::concentricSampleDisk(u2);
    Point3f pDisk = worldCenter + worldRadius * (cd.x * v1 + cd.y * v2);
    *ray = Ray(pDisk + worldRadius * -d, d, INFINITY, time);

    // Compute _InfiniteAreaLight_ ray PDFs
    *pdfDir = sinTheta == 0 ? 0 : mapPdf / (2 * SQ(PI) * sinTheta);
    *pdfPos = 1 / (PI * worldRadius * worldRadius);
    return Spectrum(Lmap->lookup(uv));
}

void InfiniteAreaLight::pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const
{
    Vector3f d = -worldToLight(ray.d);
    float theta = sphericalTheta(d), phi = sphericalPhi(d);
    Point2f uv(phi * INV_TWO_PI, theta * INV_FOUR_PI);
    float mapPdf = distrib->pdf(uv);
    *pdfDir = mapPdf / (2 * SQ(PI) * sin(theta));
    *pdfPos = 1 / (PI * SQ(worldRadius));
}

Spectrum InfiniteAreaLight::sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                      VisibilityTester *vis) const {
    // Find (u,v) sample coordinates in infinite light texture
    float mapPdf;
    Point2f uv = distrib->sampleContinuous(u, &mapPdf);
    if (mapPdf == 0) return Spectrum(0.f);

    // Convert infinite light sample point to direction
    float theta = uv[1] * PI, phi = uv[0] * 2 * PI;
    float cosTheta = cos(theta), sinTheta = sin(theta);
    float sinPhi = sin(phi), cosPhi = cos(phi);
    *wi = lightToWorld(Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta));

    // Compute PDF for sampled infinite light direction
    *pdf = mapPdf / (2 * PI * PI * sinTheta);
    if (sinTheta == 0) *pdf = 0;

    // Return radiance value for infinite light direction
    *vis = VisibilityTester(ref, Interaction(ref.p + *wi * (2 * worldRadius), ref.time, mediumInterface));
    return Spectrum(Lmap->lookup(uv));
}

float InfiniteAreaLight::pdf_Li(const Interaction &ref, const Vector3f &w) const {
    Vector3f wi = worldToLight(w);
    float theta = sphericalTheta(wi), phi = sphericalPhi(wi);
    float sinTheta = sin(theta);
    if (sinTheta == 0) return 0;
    return distrib->pdf(Point2f(phi * INV_TWO_PI, theta * INV_PI)) / (2 * SQ(PI) * sinTheta);
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
