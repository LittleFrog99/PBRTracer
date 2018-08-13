#include "integrator.h"
#include "parallel.h"
#include "sampling.h"
#include "stats.h"
#include "bsdf.h"
#include "primitive.h"

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
    Parallel::forLoop2D( [&] (Point2i tile) {
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
            ProfilePhase pp(Stage::StartPixel);
            tileSampler->startPixel(pixel);

            do {
                CameraSample camSample = tileSampler->getCameraSample(pixel);
                RayDifferential ray;
                float rayWeight = camera->generateRayDifferential(camSample, &ray);
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
    float pdf;
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
            float dDNdx = dot(dwodx, ns) + dot(wo, dndx);
            float dDNdy = dot(dwody, ns) + dot(wo, dndy);
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
    float pdf;
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

            float eta = bsdf.eta;
            Vector3f w = -wo;
            if (dot(wo, ns) < 0) eta = 1.f / eta;

            Normal3f dndx = isect.shading.dndu * isect.dudx + isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy + isect.shading.dndv * isect.dvdy;

            Vector3f dwodx = -ray.rxDirection - wo, dwody = -ray.ryDirection - wo;
            float dDNdx = dot(dwodx, ns) + dot(wo, dndx);
            float dDNdy = dot(dwody, ns) + dot(wo, dndy);

            float mu = eta * dot(w, ns) - dot(wi, ns);
            float dmudx = (eta - (eta * eta * dot(w, ns)) / dot(wi, ns)) * dDNdx;
            float dmudy = (eta - (eta * eta * dot(w, ns)) / dot(wi, ns)) * dDNdy;

            rd.rxDirection = wi + eta * dwodx - Vector3f(mu * dndx + dmudx * ns);
            rd.ryDirection = wi + eta * dwody - Vector3f(mu * dndy + dmudy * ns);
        }
        L = f * compute_Li(rd, scene, sampler, arena, depth + 1) * absDot(wi, ns) / pdf;
    }
    return L;
}

Spectrum Integrator::uniformSampleAllLights(const Interaction &it, const Scene &scene,
                                            MemoryArena &arena, Sampler &sampler,
                                            const vector<int> &nLightSamples, bool handleMedia)
{
    Spectrum L;
    for (size_t j = 0; j < scene.lights.size(); j++) {
        const auto &light = scene.lights[j];
        const int nSamples = nLightSamples[j];
        const Point2f *uLightArray = sampler.get2DArray(nSamples);
        const Point2f *uScatteringArray = sampler.get2DArray(nSamples);
        if (!(uLightArray && uScatteringArray)) { // requested arrays have all been consumed
            // Use a single sample for illumination
            Point2f uLight = sampler.get2D();
            Point2f uScattering = sampler.get2D();
            L += estimateDirect(it, uScattering, *light, uLight, scene, sampler, handleMedia);
        } else {
            // Estimate direct lighting using sample arrays
            Spectrum Ld;
            for (int k = 0; k < light->nSamples; k++)
                Ld += estimateDirect(it, uScatteringArray[k], *light, uLightArray[k], scene, sampler,
                                     handleMedia);
            L += Ld / nSamples;
        }
    }
    return L;
}

Spectrum Integrator::uniformSampleOneLight(const Interaction &it, const Scene &scene,
                                           MemoryArena &arena, Sampler &sampler, bool handleMedia)
{
    ProfilePhase _(Stage::DirectLighting);
    size_t nLights = scene.lights.size();
    if (nLights == 0) return 0;
    size_t lightNum = min(size_t(sampler.get1D() * nLights), nLights - 1);
    const auto &light = scene.lights[lightNum];
    Point2f uLight = sampler.get2D();
    Point2f uScattering = sampler.get2D();
    return float(nLights) * estimateDirect(it, uScattering, *light, uLight, scene, sampler, handleMedia);
}

Spectrum Integrator::estimateDirect(const Interaction &it, const Point2f &uScattering,
                                    const Light &light, const Point2f &uLight, const Scene &scene,
                                    Sampler &sampler, bool handleMedia, bool specular)
{
    BxDFType flags = BxDFType(specular ? BSDF_ALL : (BSDF_ALL & ~BSDF_SPECULAR));
    Spectrum Ld;

    // Sample light source
    Vector3f wi;
    float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester vis;
    Spectrum Li = light.sample_Li(it, uLight, &wi, &lightPdf, &vis);

    if (lightPdf > 0 && !Li.isBlack()) {
        // Compute BSDF or phase function for light sample
        Spectrum f;
        if (it.isSurfaceInteraction()) {
            const auto &isect = static_cast<const SurfaceInteraction &>(it);
            f = isect.bsdf->compute_f(isect.wo, wi, flags) * absDot(wi, isect.shading.n);
            scatteringPdf = isect.bsdf->pdf(isect.wo, wi, flags);
        } else {
            const auto &mi = static_cast<const MediumInteraction &>(it);
            float p = mi.phase->compute_p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        if (!f.isBlack()) {
            // Compute effect of visibility
            if (handleMedia)
                Li *= vis.compute_Tr(scene, sampler);
            else if (!vis.unoccluded(scene))
                Li = 0;

            // Add light's contribution to reflected radiance
            if (!Li.isBlack()) {
                if (light.isDeltaLight())
                    Ld += f * Li / lightPdf; // can't use MIS in a delta distribution
                else {
                    float weight = Sampling::powerHeuristic(1, lightPdf, 1, scatteringPdf);
                    Ld += f * Li * weight / lightPdf;
                } // end light.isDeltaLight()
            } // end !Li.isBlack()

        } // end !f.isBlack()
    } // end lightPdf > 0 && !Li.isBlack()

    // Sample BSDF
    if (!light.isDeltaLight()) {
        Spectrum f;
        bool sampledSpecular = false;
        if (it.isSurfaceInteraction()) {
            BxDFType sampledType;
            const auto &isect = static_cast<const SurfaceInteraction &>(it);
            f = isect.bsdf->sample_f(isect.wo, &wi, uScattering, &scatteringPdf, flags, &sampledType);
            f *= absDot(isect.shading.n, wi);
            sampledSpecular = sampledType & BSDF_SPECULAR;
        } else {
            const auto &mi = static_cast<const MediumInteraction &>(it);
            float p = mi.phase->sample_p(mi.wo, &wi, uScattering);
            f = Spectrum(p);
            scatteringPdf = p;
        }

        // Account for light contributions along sampled direction
        if (!f.isBlack() && scatteringPdf > 0) {
            float weight = 1;
            if (!sampledSpecular) {
                lightPdf = light.pdf_Li(it, wi);
                if (lightPdf == 0) return Ld;
                weight = Sampling::powerHeuristic(1, scatteringPdf, 1, lightPdf);
            }

            // Find intersection and compute transmittance
            SurfaceInteraction lightIsect;
            Ray ray = it.spawnRay(wi);
            Spectrum Tr(1.0f);
            bool foundSI = handleMedia ? scene.intersectTr(ray, sampler, &lightIsect, &Tr)
                                       : scene.intersect(ray, &lightIsect);

            // Add light contribution from material sampling
            Spectrum Li;
            if (foundSI) {
                if (lightIsect.primitive->getAreaLight() == &light)
                    Li = lightIsect.compute_Le(wi);
            }
            else
                Li = light.compute_Le(ray); // for infinite area light
            if (!Li.isBlack())
                Ld += f * Li * Tr * weight / scatteringPdf;

        } // end !f.isBlack() && scatteringPdf > 0
    } // end !light.isDeltaLight()

    return Ld;
}

unique_ptr<Distribution1D> Integrator::computeLightPowerDistribution(const vector<shared_ptr<Light>> lights) {
    if (lights.empty()) return nullptr;
    vector<float> lightPower;
    for (const auto &light : lights)
        lightPower.push_back(light->power().luminance());
    return make_unique<Distribution1D>(&lightPower[0], lightPower.size());
}
