#include "volpath.h"
#include "core/bsdf.h"
#include "core/bssrdf.h"
#include "stats.h"

Spectrum VolPathIntegrator::compute_Li(const RayDifferential &r, const Scene &scene, Sampler &sampler,
                                       MemoryArena &arena, int depth) const
{
    ProfilePhase p(Stage::SamplerIntegratorLi);
    Spectrum L;
    Spectrum beta(1.0f); // path throughput weight
    RayDifferential ray(r);
    float etaScale = 1;
    bool specularBounce = false;

    for (int bounce = 0; ; bounce++) {
        // Intersect ray with scene and store in isect
        SurfaceInteraction isect;
        bool foundIsect = scene.intersect(ray, &isect);

        // Sample the participating medium
        MediumInteraction mi;
        if (ray.medium)
            beta *= ray.medium->sample(ray, sampler, arena, &mi);
        if (beta.isBlack())
            break;

        // Handle an interaction with a medium or a surface
        if (mi.isValid()) {
            if (bounce > maxDepth) break;
            L += beta * uniformSampleOneLight(mi, scene, arena, sampler, true);
            Vector3f wo = -ray.d, wi;
            mi.phase->sample_p(wo, &wi, sampler.get2D());
            ray = mi.spawnRay(wi);
            specularBounce = false;
        } else {
            // Add light emission at intersection
            if (bounce == 0 || specularBounce) {
                if (foundIsect)
                    L += beta * isect.compute_Le(-ray.d); // emission surface
                else
                    for (const auto &light : scene.lights)
                        L += beta * light->compute_Le(ray); // infinite area light
            }

            // Terminate path if ray escaped or maxDepth was reached
            if (!foundIsect || bounce >= maxDepth) break;

            // Compute scattering functions
            isect.computeScatteringFunctions(ray, arena, true);
            if (!isect.bsdf) { // skip medium boundaries
                ray = isect.spawnRay(ray.d);
                bounce--;
                continue;
            }

            // Sample illumination from lights to find path contribution
            L += beta * uniformSampleOneLight(isect, scene, arena, sampler, true);

            // Sample BSDF to get new path direction
            Vector3f wo = -ray.d, wi;
            float pdf;
            BxDFType flags;
            Spectrum f = isect.bsdf->sample_f(wo, &wi, sampler.get2D(), &pdf, BSDF_ALL, &flags);
            if (f.isBlack() || pdf == 0) break;
            beta *= f * absDot(wi, isect.shading.n) / pdf;
            specularBounce = (flags & BSDF_SPECULAR) != 0;
            if ((flags & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION)) {
                float eta = isect.bsdf->eta;
                etaScale *= dot(wo, isect.n) > 0 ? SQ(eta) : 1 / SQ(eta);
            }
            ray = isect.spawnRay(wi);

            // Account for BSSRDF
            if (isect.bssrdf && (flags & BSDF_TRANSMISSION)) {
                // Importance sample the BSSRDF
                SurfaceInteraction pi;
                Spectrum S = isect.bssrdf->sample_S(scene, sampler.get1D(), sampler.get2D(), arena, &pi, &pdf);
                if (S.isBlack() || pdf == 0) break;
                beta *= S / pdf; // the spatial component

                // Account for direct lighting
                L += beta * uniformSampleOneLight(pi, scene, arena, sampler, true);

                // Indirect lighting
                Spectrum f = pi.bsdf->sample_f(pi.wo, &wi, sampler.get2D(), &pdf, BSDF_ALL, &flags);
                if (f.isBlack() || pdf == 0) break;
                beta *= f * absDot(wi, pi.shading.n) / pdf; // the direction component
                specularBounce = (flags & BSDF_SPECULAR) != 0;
                ray = pi.spawnRay(wi);
            }

        }

        // Possibly terminate the path with Russian roulette
        Spectrum rrBeta = beta * etaScale;
        if (bounce > 3 && rrBeta.maxComp() < 0.05f) {
            float q = max(0.05f, 1 - rrBeta.maxComp());
            if (sampler.get1D() < q) break;
            beta /= 1 - q;
        }
    }
    return L;
}

VolPathIntegrator *VolPathIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
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

    return new VolPathIntegrator(maxDepth, camera, sampler, pixelBounds);
}
