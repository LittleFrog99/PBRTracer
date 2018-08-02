#include "directlighting.h"
#include "stats.h"
#include "paramset.h"

void DirectLightingIntegrator::preprocess(const Scene &scene, Sampler &sampler) {
    if (strategy == LightStrategy::UniformSampleAll) {
        for (const auto &light : scene.lights)
            nLightSamples.push_back(sampler.roundCount(light->nSamples));
        for (int i = 0; i < maxDepth; i++)
            for (size_t j = 0; j < scene.lights.size(); j++) {
                sampler.request2DArray(nLightSamples[j]); // for position on the light source
                sampler.request2DArray(nLightSamples[j]); // for BSDF scattering direction
            }
    }
}

Spectrum DirectLightingIntegrator::compute_Li(const RayDifferential &ray, const Scene &scene,
                                              Sampler &sampler, MemoryArena &arena, int depth) const
{
    ProfilePhase p(Stage::SamplerIntegratorLi);
    Spectrum L(0.f);
    // Find closest ray intersection or return background radiance
    SurfaceInteraction isect;
    if (!scene.intersect(ray, &isect)) {
        for (const auto &light : scene.lights)
            L += light->compute_Le(ray);
        return L;
    }

    // Compute scattering functions for surface interaction
    isect.computeScatteringFunctions(ray, arena);
    if (!isect.bsdf)
        return compute_Li(isect.spawnRay(ray.d), scene, sampler, arena, depth);
    Vector3f wo = isect.wo;

    L += isect.compute_Le(wo); // hit emission surface
    if (scene.lights.size() > 0) {
        if (strategy == LightStrategy::UniformSampleAll)
            L += uniformSampleAllLights(isect, scene, arena, sampler, nLightSamples);
        else
            L += uniformSampleOneLight(isect, scene, arena, sampler);
    }
    if (depth + 1 < maxDepth) {
        L += specularReflect(ray, isect, scene, sampler, arena, depth);
        L += specularTransmit(ray, isect, scene, sampler, arena, depth);
    }

    return L;
}

DirectLightingIntegrator *
DirectLightingIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                 shared_ptr<const Camera> camera)
{
    int maxDepth = params.findOneInt("maxdepth", 5);
    LightStrategy strategy;
    string st = params.findOneString("strategy", "all");
    if (st == "one")
        strategy = LightStrategy::UniformSampleOne;
    else if (st == "all")
        strategy = LightStrategy::UniformSampleAll;
    else {
        WARNING("Strategy \"%s\" for direct lighting unknown. Using \"all\".", st.c_str());
        strategy = LightStrategy::UniformSampleAll;
    }
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
    return new DirectLightingIntegrator(strategy, maxDepth, camera, sampler, pixelBounds);
}
