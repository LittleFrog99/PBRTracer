#include "scene.h"
#include "stats.h"
#include "core/primitive.h"

STAT_COUNTER("Intersections/Regular ray intersection tests", nIntersectionTests);
STAT_COUNTER("Intersections/Shadow ray intersection tests", nShadowTests);

Scene::Scene(shared_ptr<Primitive> aggregate, const vector<shared_ptr<Light>> &lights)
    : lights(lights), aggregate(aggregate)
{
    worldBound = aggregate->worldBound();
    for (const auto &light : lights) {
        light->preprocess(*this);
        if (light->flags & int(LightFlags::Infinite))
            infiniteLights.push_back(light);
    }
}

bool Scene::intersect(const Ray &ray, SurfaceInteraction *isect) const {
    ++nIntersectionTests;
    DCHECK_NE(ray.d, Vector3f(0,0,0));
    return aggregate->intersect(ray, isect);
}

bool Scene::intersectP(const Ray &ray) const {
    ++nShadowTests;
    DCHECK_NE(ray.d, Vector3f(0,0,0));
    return aggregate->intersectP(ray);
}

bool Scene::intersectTr(Ray ray, Sampler &sampler, SurfaceInteraction *isect, Spectrum *Tr) const {
    *Tr = Spectrum(1.f);
    while (true) {
        bool hitSurface = intersect(ray, isect);
        // Accumulate beam transmittance for ray segment
        if (ray.medium) *Tr *= ray.medium->compute_Tr(ray, sampler);
        // Terminate transmittance computation
        if (!hitSurface) return false;
        if (isect->primitive->getMaterial()) return true;
        // Intialize next ray
        ray = isect->spawnRay(ray.d);
    }
}
