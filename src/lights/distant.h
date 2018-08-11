#ifndef LIGHT_DISTANT
#define LIGHT_DISTANT

#include "core/light.h"

class DistantLight : public Light {
public:
    DistantLight(const Transform &lightToWorld, const Spectrum &L, const Vector3f &wLight)
        : Light(int(LightFlags::DeltaDirection), lightToWorld, MediumInterface()),
          L(L), wLight(normalize(wLight)) {}

    static shared_ptr<DistantLight> create(const Transform &light2world, const ParamSet &paramSet);

    void preprocess(const Scene &scene);

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const;
    float pdf_Li(const Interaction &, const Vector3f &) const { return 0; }

    Spectrum power() const { return L * PI * SQ(worldRadius); }

    Spectrum sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray, Normal3f *nLight,
                       float *pdfPos, float *pdfDir) const;
    void pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const;

private:
    const Spectrum L;
    const Vector3f wLight;
    Point3f worldCenter;
    float worldRadius;
};

#endif // LIGHT_DISTANT
