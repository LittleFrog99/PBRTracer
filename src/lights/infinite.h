#ifndef LIGHT_INFINITEAREA
#define LIGHT_INFINITEAREA

#include "core/light.h"
#include "core/sampling.h"
#include "mipmap.h"

class InfiniteAreaLight : public Light {
public:
    InfiniteAreaLight(const Transform &lightToWorld, const Spectrum &Lemit, int nSamples,
                      const string &texmap);

    static shared_ptr<InfiniteAreaLight> create(const Transform &light2world, const ParamSet &paramSet);

    void preprocess(const Scene &scene);

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const;
    float pdf_Li(const Interaction &ref, const Vector3f &wi) const;

    Spectrum compute_Le(const RayDifferential &r) const;

    Spectrum power() const;

    Spectrum sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray, Normal3f *nLight,
                       float *pdfPos, float *pdfDir) const;
    void pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const;

private:
    unique_ptr<Mipmap<RGBSpectrum>> Lmap;
    Point3f worldCenter;
    float worldRadius;
    unique_ptr<Distribution2D> distrib;
};

#endif // LIGHT_INFINITEAREA
