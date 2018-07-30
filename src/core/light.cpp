#include "light.h"
#include "core/scene.h"
#include "core/primitive.h"

Light::Light(int flags, const Transform &lightToWorld, const MediumInterface &mediumInterface, int nSamples)
    : flags(flags), nSamples(max(1, nSamples)), mediumInterface(mediumInterface),
      lightToWorld(lightToWorld), worldToLight(lightToWorld.inverse())
{
    if (lightToWorld.hasScale())
        LOG(ERROR) << "Light to world transform has scale factors.";
}

Spectrum VisibilityTester::compute_Tr(const Scene &scene, Sampler &sampler) const {
    Ray ray(p0.spawnRayTo(p1));
    Spectrum Tr(1.0f);
    while (true) {
        SurfaceInteraction isect;
        bool hitSurface = scene.intersect(ray, &isect);
        if (hitSurface && isect.primitive->getMaterial()) // not media bound
            return Spectrum(0.0f); // occluded by opaque surface
        if (ray.medium)
            Tr *= ray.medium->compute_Tr(ray, sampler); // accumulate beam transmittance
        if (!hitSurface) break;
        ray = isect.spawnRayTo(p1);
    }
    return Tr;
}
