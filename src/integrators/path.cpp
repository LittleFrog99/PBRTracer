#include "path.h"
#include "paramset.h"
#include "core/bsdf.h"
#include "core/bssrdf.h"
#include "stats.h"

Spectrum PathIntegrator::compute_Li(const RayDifferential &r, const Scene &scene, Sampler &sampler,
                                    MemoryArena &arena, int depth) const
{
    Spectrum L;
    Spectrum beta(1.0f); // path throughput
    RayDifferential ray(r);
    bool specularBounce = false;
    for (int bounce = 0; ; bounce++) {
        // Intersect ray with scene and store in isect
        SurfaceInteraction isect;
        bool foundIsect = scene.intersect(ray, &isect);

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
        L += beta * uniformSampleOneLight(isect, scene, arena, sampler);

        // Sample BSDF to get new path direction
        Vector3f wo = -ray.d, wi;
        float pdf;
        BxDFType flags;
        Spectrum f = isect.bsdf->sample_f(wo, &wi, sampler.get2D(), &pdf, BSDF_ALL, &flags);
        if (f.isBlack() || pdf == 0) break;
        beta *= f * absDot(wi, isect.shading.n) / pdf;
        specularBounce = (flags & BSDF_SPECULAR) != 0;
        ray = isect.spawnRay(wi);

        // Account for BSSRDF
        if (isect.bssrdf && (flags & BSDF_TRANSMISSION)) {
            // Importance sample the BSSRDF
            SurfaceInteraction pi;
            Spectrum S = isect.bssrdf->sample_S(scene, sampler.get1D(), sampler.get2D(), arena, &pi, &pdf);
            if (S.isBlack() || pdf == 0) break;
            beta *= S / pdf; // the spatial component

            // Account for direct lighting
            L += beta * uniformSampleOneLight(pi, scene, arena, sampler);

            // Indirect lighting
            Spectrum f = pi.bsdf->sample_f(pi.wo, &wi, sampler.get2D(), &pdf, BSDF_ALL, &flags);
            if (f.isBlack() || pdf == 0) break;
            beta *= f * absDot(wi, pi.shading.n) / pdf; // the direction component
            specularBounce = (flags & BSDF_SPECULAR) != 0;
            ray = pi.spawnRay(wi);
        }

        // Possibly terminate the path with Russian roulette
        if (bounce > 3) {
            float q = max(0.05f, 1 - beta.luminance());
            if (sampler.get1D() < q) break;
            beta /= 1 - q;
        }
    }
    return L;
}

PathIntegrator * PathIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
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
    return new PathIntegrator(maxDepth, camera, sampler, pixelBounds);
}

