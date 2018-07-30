#include "whitted.h"
#include "core/scene.h"
#include "core/bsdf.h"
#include "paramset.h"

Spectrum WhittedIntegrator::compute_Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
                                MemoryArena &arena, int depth) const
{
    Spectrum L;
    SurfaceInteraction isect;
    if (!scene.intersect(ray, &isect)) {
        for (auto const &light : scene.lights)
            L += light->compute_Le(ray); // account for infinite light
        return L;
    }

    Normal3f n = isect.shading.n;
    Vector3f wo = isect.wo;
    isect.computeScatteringFunctions(ray, arena);
    L += isect.compute_Le(wo); // intersect emissive surface

    // Add contribution of each light source
    for (const auto &light : scene.lights) {
        Vector3f wi;
        float pdf;
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

WhittedIntegrator * WhittedIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                              shared_ptr<const Camera> camera)
{
    int maxDepth = params.findOneInt("maxdepth", 5);
    int np;
    const int *pb = params.findInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->getSampleBounds();
    if (pb) {
        if (np != 4)
            ERROR("Expected four values for \"pixelbounds\" parameter. Got %d.", np);
        else {
            pixelBounds = intersect(pixelBounds, Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.area() == 0)
                ERROR("Degenerate \"pixelbounds\" specified.");
        }
    }
    return new WhittedIntegrator(maxDepth, camera, sampler, pixelBounds);
}
