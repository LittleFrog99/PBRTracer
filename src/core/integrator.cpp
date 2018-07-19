#include "integrator.h"
#include "parallel.h"
#include "sampling.h"
#include "stats.h"
#include "bsdf.h"

STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

void SamplerIntegrator::render(const Scene &scene) {
    preprocess(scene, *sampler);

    // Compute number of tiles, nTiles, to use for parallel rendering
    Bounds2i sampleBounds = camera->film->getSampleBounds();
    Vector2i sampleExtent = sampleBounds.diagonal();
    const int tileSize = 16;
    Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize, // at least one, no completely unused
                   (sampleExtent.y + tileSize - 1) / tileSize);

    ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
    Parallel::forLoop2D(
                [&](Point2i tile) {
        MemoryArena arena;
        int seed = tile.y * nTiles.x + tile.x;
        unique_ptr<Sampler> tileSampler = sampler->clone(seed);

        // Compute sample bounds for tile
        int x0 = sampleBounds.pMin.x + tile.x * tileSize;
        int x1 = min(x0 + tileSize, sampleBounds.pMax.x);
        int y0 = sampleBounds.pMin.y + tile.y * tileSize;
        int y1 = min(y0 + tileSize, sampleBounds.pMax.y);
        Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));

        unique_ptr<FilmTile> filmTile = camera->film->getFilmTile(tileBounds);
        for (Point2i pixel : tileBounds) {
            ProfilePhase pp(Profiler::Stage::StartPixel);
            tileSampler->startPixel(pixel);

            do {
                CameraSample camSample = tileSampler->getCameraSample(pixel);
                RayDifferential ray;
                Float rayWeight = camera->generateRayDifferential(camSample, &ray);
                ray.scaleDifferentials(1.0 / sqrt(tileSampler->samplesPerPixel));
                ++nCameraRays;

                Spectrum L;
                if (rayWeight > 0)
                    L = compute_Li(ray, scene, *tileSampler, arena);

                // Issue warning if unexpected radiance value returned
                if (L.hasNaNs()) {
                    LOG(ERROR) << STRING_PRINTF(
                        "Not-a-number radiance value returned "
                        "for pixel (%d, %d), sample %d. Setting to black.",
                        pixel.x, pixel.y, tileSampler->currentSampleNumber());
                    L = Spectrum(0.f);
                } else if (L.luminance() < -1e-5) {
                    LOG(ERROR) << STRING_PRINTF(
                        "Negative luminance value, %f, returned "
                        "for pixel (%d, %d), sample %d. Setting to black.",
                        L.luminance(), pixel.x, pixel.y, tileSampler->currentSampleNumber());
                    L = Spectrum(0.f);
                } else if (isinf(L.luminance())) {
                      LOG(ERROR) << STRING_PRINTF(
                        "Infinite luminance value returned "
                        "for pixel (%d, %d), sample %d. Setting to black.",
                        pixel.x, pixel.y, tileSampler->currentSampleNumber());
                    L = Spectrum(0.f);
                }

                filmTile->addSample(camSample.pFilm, L, rayWeight);
                arena.reset();

            } while (tileSampler->startNextSample());
        }

        LOG(INFO) << "Finished image tile " << tileBounds;
        camera->film->mergeFilmTile(move(filmTile));
        reporter.update();

    }, nTiles);

    reporter.done();
    camera->film->writeImage();
    LOG(INFO) << "Rendering finished";
}

Spectrum SamplerIntegrator::specularReflect(const RayDifferential &ray, const SurfaceInteraction &isect,
                                            const Scene &scene, Sampler &sampler,
                                            MemoryArena &arena, int depth) const
{
    // Compute specular reflection direction _wi_ and BSDF value
    Vector3f wo = isect.wo, wi;
    Float pdf;
    BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
    Spectrum f = isect.bsdf->sample_f(wo, &wi, sampler.get2D(), &pdf, type);

    // Return contribution of specular reflection
    const Normal3f &ns = isect.shading.n;
    if (pdf > 0.f && !f.isBlack() && absDot(wi, ns) != 0.f) {
        RayDifferential rd = isect.spawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = isect.p + isect.dpdx;
            rd.ryOrigin = isect.p + isect.dpdy;

            Normal3f dndx = isect.shading.dndu * isect.dudx + isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy + isect.shading.dndv * isect.dvdy;
            Vector3f dwodx = -ray.rxDirection - wo, dwody = -ray.ryDirection - wo;
            Float dDNdx = dot(dwodx, ns) + dot(wo, dndx);
            Float dDNdy = dot(dwody, ns) + dot(wo, dndy);
            rd.rxDirection = wi - dwodx + 2.f * Vector3f(dot(wo, ns) * dndx + dDNdx * ns);
            rd.ryDirection = wi - dwody + 2.f * Vector3f(dot(wo, ns) * dndy + dDNdy * ns);
        }
        return f * compute_Li(rd, scene, sampler, arena, depth + 1) * absDot(wi, ns) / pdf;
    } else
        return Spectrum(0.0);
}

Spectrum SamplerIntegrator::specularTransmit(const RayDifferential &ray, const SurfaceInteraction &isect,
                                             const Scene &scene, Sampler &sampler,
                                             MemoryArena &arena, int depth) const
{
    Vector3f wo = isect.wo, wi;
    Float pdf;
    const Point3f &p = isect.p;
    const Normal3f &ns = isect.shading.n;
    const BSDF &bsdf = *isect.bsdf;
    Spectrum f = bsdf.sample_f(wo, &wi, sampler.get2D(), &pdf, BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
    Spectrum L;
    if (pdf > 0.f && !f.isBlack() && absDot(wi, ns) != 0.f) {
        RayDifferential rd = isect.spawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = p + isect.dpdx;
            rd.ryOrigin = p + isect.dpdy;

            Float eta = bsdf.eta;
            Vector3f w = -wo;
            if (dot(wo, ns) < 0) eta = 1.f / eta;

            Normal3f dndx = isect.shading.dndu * isect.dudx + isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy + isect.shading.dndv * isect.dvdy;

            Vector3f dwodx = -ray.rxDirection - wo, dwody = -ray.ryDirection - wo;
            Float dDNdx = dot(dwodx, ns) + dot(wo, dndx);
            Float dDNdy = dot(dwody, ns) + dot(wo, dndy);

            Float mu = eta * dot(w, ns) - dot(wi, ns);
            Float dmudx = (eta - (eta * eta * dot(w, ns)) / dot(wi, ns)) * dDNdx;
            Float dmudy = (eta - (eta * eta * dot(w, ns)) / dot(wi, ns)) * dDNdy;

            rd.rxDirection = wi + eta * dwodx - Vector3f(mu * dndx + dmudx * ns);
            rd.ryDirection = wi + eta * dwody - Vector3f(mu * dndy + dmudy * ns);
        }
        L = f * compute_Li(rd, scene, sampler, arena, depth + 1) * absDot(wi, ns) / pdf;
    }
    return L;
}
