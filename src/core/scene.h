#ifndef SCENE_H
#define SCENE_H

#include "light.h"
#include "primitive.h"

class Scene {
public:
    Scene(shared_ptr<Primitive> aggregate, const vector<shared_ptr<Light>> &lights)
        : lights(lights), aggregate(aggregate) {
        worldBound = aggregate->worldBound();
        for (const auto &light : lights) {
            light->preprocess(*this);
            if (light->flags & int(LightFlags::Infinite))
                infiniteLights.push_back(light);
        }
    }

    const Bounds3f & getWorldBound() const { return worldBound; }
    bool intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &ray) const;
    bool intersectTr(Ray ray, Sampler &sampler, SurfaceInteraction *isect,
                     Spectrum *transmittance) const;

    vector<shared_ptr<Light>> lights;
    vector<shared_ptr<Light>> infiniteLights;

private:
    shared_ptr<Primitive> aggregate;
    Bounds3f worldBound;
};

#endif // SCENE_H
