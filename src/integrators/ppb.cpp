#include "ppb.h"
#include "samplers/halton.h"
#include "stats.h"
#include "lowdiscrep.h"
#include "core/bsdf.h"

using namespace LowDiscrepancy;

void PPBIntegrator::render(const Scene &scene) {
    ProfilePhase __(Stage::IntegratorRender);
    // Initialize _pixelBounds_ and pixel array
    Bounds2i pixelBounds = camera->film->croppedPixelBounds;
    int nPixels = pixelBounds.area();
    const float invSqrtSPP = 1.0f / sqrtf(nIterations);

    // Compute light distribution for sampling lights proportional to light power
    auto lightDistrib = computeLightPowerDistribution(scene.lights);

    // Initialize sampler
    HaltonSampler globalSampler(nIterations, pixelBounds);

    // Compute number of tiles to use for camera pass
    Vector2i pixelExtent = pixelBounds.diagonal();
    constexpr int tileSize = 16;
    Point2i nTiles((pixelExtent.x + tileSize - 1) / tileSize, (pixelExtent.y + tileSize - 1) / tileSize);

    // Perform integration
    for (int ite = 0; ite < nIterations; ite++) {
        vector<MemoryArena> perThreadArenas;
        {
            ProfilePhase _(Stage::SPPMPhotonPass);
            Parallel::forLoop([&] (int photonIndex) {
                auto &arena = perThreadArenas[Parallel::getThreadIndex()];
                int64_t haltonIndex = int64_t(ite) * int64_t(photonsPerIteration) + int64_t(photonIndex);
                auto photonSampler = globalSampler.clone(0);
                photonSampler->setSampleNumber(haltonIndex);

                // Choose light to shoot photons from
                float lightPdf;
                float lightSample = photonSampler->get1D();
                int lightNum = lightDistrib->sampleDiscrete(lightSample, &lightPdf);
                const auto &light = scene.lights[lightNum];

                // Generate photon ray from light source
                RayDifferential photonRay;
                Normal3f nLight;
                float pdfPos, pdfDir;
                Spectrum Le = light->sample_Le(photonSampler->get2D(), photonSampler->get2D(),
                                               photonSampler->get1D(), &photonRay, &nLight, &pdfPos, &pdfDir);
                if (Le.isBlack() || pdfPos == 0 || pdfDir == 0) return;
                Spectrum beta = absDot(nLight, photonRay.d) * Le / (lightPdf * pdfPos * pdfDir);
                if (beta.isBlack()) return;

                // Follow photon path throught the scene and record interaction
                for (int depth = 0; ; depth++) {
                    // Intersect ray with scene and store in isect
                    SurfaceInteraction isect;
                    bool foundIsect = scene.intersect(photonRay, &isect);

                    // Store current photon beam

                    // Sample the participating medium
                    MediumInteraction mi;
                    if (photonRay.medium)
                        beta *= photonRay.medium->sample(photonRay, *photonSampler, arena, &mi);
                    if (beta.isBlack())
                        break;

                    // Handle an interaction with a medium or a surface
                    if (mi.isValid()) {
                        if (depth > maxDepth) break;
                        Vector3f wo = -photonRay.d, wi;
                        mi.phase->sample_p(wo, &wi, photonSampler->get2D());
                        photonRay = mi.spawnRay(wi);
                    } else {
                        // Terminate path if photon escaped or maxDepth was reached
                        if (!foundIsect || depth >= maxDepth) break;

                        // Compute scattering functions
                        isect.computeScatteringFunctions(photonRay, arena, true, TransportMode::Importance);
                        if (!isect.bsdf) { // skip medium boundaries
                            photonRay = isect.spawnRay(photonRay.d);
                            depth--;
                            continue;
                        }

                        // Sample BSDF to get new path direction
                        Vector3f wo = -photonRay.d, wi;
                        float pdf;
                        BxDFType flags;
                        Spectrum f = isect.bsdf->sample_f(wo, &wi, photonSampler->get2D(), &pdf, BSDF_ALL, &flags);
                        if (f.isBlack() || pdf == 0) break;
                        beta *= f * absDot(wi, isect.shading.n) / pdf;
                        photonRay = isect.spawnRay(wi);
                    }

                    // Possibly terminate the path with Russian roulette
                    if (beta.luminance() < 0.05f) {
                        float q = max(0.99f, 1 - beta.luminance());
                        if (photonSampler->get1D() < q) break;
                        beta /= 1 - q;
                    }
                } // end photon path
            }, photonsPerIteration, 8192);
        }
    }

}
