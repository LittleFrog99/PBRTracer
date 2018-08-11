#ifndef CORE_LIGHT
#define CORE_LIGHT

#include "spectrum.h"
#include "transform.h"
#include "scene.h"

class Scene;
class VisibilityTester;
class Sampler;

enum class LightFlags : int {
    DeltaPosition = 1,
    DeltaDirection = 2,
    Area = 4,
    Infinite = 8
};

class Light {
public:
    Light(int flags, const Transform &lightToWorld, const MediumInterface &mediumInterface, int nSamples = 1);
    virtual ~Light() {}

    virtual Spectrum power() const = 0;

    virtual void preprocess(const Scene &) {}

    virtual Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                               VisibilityTester *vis) const = 0;
    virtual float pdf_Li(const Interaction &ref, const Vector3f &wi) const = 0;

    virtual Spectrum compute_Le(const RayDifferential &r) const { return Spectrum(0.0f); }
    virtual Spectrum sample_Le(const Point2f &u1, const Point2f &u2, float time, Ray *ray,
                               Normal3f *nLight, float *pdfPos, float *pdfDir) const = 0;
    virtual void pdf_Le(const Ray &ray, const Normal3f &nLight, float *pdfPos, float *pdfDir) const = 0;

    bool isDeltaLight() const {
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

    const Interaction & getP0() const { return p0; }
    const Interaction & getP1() const { return p1; }

    inline bool unoccluded(const Scene &scene) const {
        return !scene.intersectP(p0.spawnRayTo(p1));
    }

    Spectrum compute_Tr(const Scene &scene, Sampler &sampler) const;

private:
    Interaction p0, p1;
};

class AreaLight : public Light {
public:
    AreaLight(const Transform &lightToWorld, const MediumInterface &medium, int nSamples)
        : Light(int(LightFlags::Area), lightToWorld, medium, nSamples) {}

    virtual Spectrum compute_L(const Interaction &intr, const Vector3f &w) const = 0;
};

#endif // CORE_LIGHT
