#ifndef CORE_LIGHT
#define CORE_LIGHT

#include "spectrum.h"
#include "transform.h"
#include "interaction.h"

class Scene;
class VisibilityTester;
class Sampler;

enum class LightFlags {
    DeltaPosition = 1,
    DeltaDirection = 2,
    Area = 4,
    Infinite = 8
};

class Light {
public:
    Light(int flags, const Transform &LightToWorld, const MediumInterface &mediumInterface,
          int nSamples = 1);
    virtual Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, Float *pdf,
                               VisibilityTester *vis) const = 0;
    virtual Spectrum power() const = 0;
    virtual void preprocess(const Scene &scene) {}
    virtual Spectrum compute_Le(const RayDifferential &r) const;
    virtual Float pdf_Li(const Interaction &ref, const Vector3f &wi) const = 0;
    virtual Spectrum sample_Le(const Point2f &u1, const Point2f &u2, Float time, Ray *ray,
                               Normal3f *nLight, Float *pdfPos, Float *pdfDir) const = 0;
    virtual void pdf_Le(const Ray &ray, const Normal3f &nLight, Float *pdfPos, Float *pdfDir) const = 0;
    virtual ~Light();

    static bool isDeltaLight(int flags) {
        return flags & (int(LightFlags::DeltaPosition) | int(LightFlags::DeltaDirection));
    }

    const int flags;
    const int nSamples;
    const MediumInterface mediumInterface;

protected:
    const Transform lightToWorld, worldToLight;
};

class VisibilityTester {
  public:
    VisibilityTester() {}
    VisibilityTester(const Interaction &p0, const Interaction &p1) : p0(p0), p1(p1) {}
    const Interaction &P0() const { return p0; }
    const Interaction &P1() const { return p1; }
    bool unoccluded(const Scene &scene) const;
    Spectrum compute_Tr(const Scene &scene, Sampler &sampler) const;

private:
    Interaction p0, p1;
};

class AreaLight : public Light {
public:
    AreaLight(const Transform &LightToWorld, const MediumInterface &medium,
              int nSamples);
    virtual Spectrum compute_L(const Interaction &intr, const Vector3f &w) const = 0;
};

#endif // CORE_LIGHT
