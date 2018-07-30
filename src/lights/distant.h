#ifndef LIGHT_DISTANT
#define LIGHT_DISTANT

#include "core/light.h"

class DistantLight : public Light {
public:
    DistantLight(const Transform &lightToWorld, const Spectrum &L, const Vector3f &wLight)
        : Light(int(LightFlags::DeltaDirection), lightToWorld, MediumInterface()),
          L(L), wLight(normalize(wLight)) {}

    static shared_ptr<DistantLight> create(const Transform &light2world, const ParamSet &paramSet);

    void preprocess(const Scene &scene) {
        scene.getWorldBound().boundingSphere(&worldCenter, &worldRadius);
    }

    Spectrum sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                       VisibilityTester *vis) const;

    Spectrum power() const {
        return L * PI * SQ(worldRadius);
    }

private:
    const Spectrum L;
    const Vector3f wLight;
    Point3f worldCenter;
    float worldRadius;
};

#endif // LIGHT_DISTANT
