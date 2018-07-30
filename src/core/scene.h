#ifndef CORE_SCENE
#define CORE_SCENE

#include "interaction.h"
#include "bounds.h"

class Light;

class Scene {
public:
    Scene(shared_ptr<Primitive> aggregate, const vector<shared_ptr<Light>> &lights);

    const Bounds3f & getWorldBound() const { return worldBound; }

    bool intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &ray) const;
    bool intersectTr(Ray ray, Sampler &sampler, SurfaceInteraction *isect, Spectrum *transmittance) const;

    vector<shared_ptr<Light>> lights;
    vector<shared_ptr<Light>> infiniteLights;

private:
    shared_ptr<Primitive> aggregate;
    Bounds3f worldBound;
};

#endif // CORE_SCENE
