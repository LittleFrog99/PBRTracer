#include "whitted.h"
#include "core/scene.h"
#include "core/bsdf.h"

Spectrum Whitted::compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                                MemoryArena &arena, int depth) const
{
    Spectrum L;
    SurfaceInteraction isect;
    if (!scene.intersect(ray, &isect)) { // no intersection was found
        for (auto const &light : scene.lights)
            L += light->compute_Le(ray);
        return L;
    }

    Normal3f n = isect.shading.n;
    Vector3f wo = isect.wo;
    isect.computeScatteringFuncs(ray, arena);
    L += isect.compute_Le(wo); // intersect emissive surface

    // Add contribution of each light source
    for (const auto &light : scene.lights) {
        Vector3f wi;
        Float pdf;
        VisibilityTester visib;
        Spectrum Li = light->sample_Li(isect, sampler.get2D(), &wi, &pdf, &visib);
        if (Li.isBlack() || pdf == 0) continue;
        Spectrum f = isect.bsdf->compute_f(wo, wi);
        if (!f.isBlack() && visib.unoccluded(scene))
            L += f * Li * absDot(wi, n) / pdf;
    }

    if (depth + 1 < maxDepth) {
        // Trace rays for specular reflection and refraction
        L += specularReflect(ray, isect, scene, sampler, arena, depth);
        L += specularTransmit(ray, isect, scene, sampler, arena, depth);
    }

    return L;
}
