#include "sppm.h"
#include "samplers/halton.h"
#include "core/bsdf.h"
#include "core/renderer.h"
#include "lowdiscrep.h"
#include "stats.h"
#include "paramset.h"

STAT_RATIO("SPPM/Visible points checked per photon intersection", visiblePhotonChecked,
           totalPhotonSurfaceInteraction);
STAT_COUNTER("SPPM/Photon path followed", photonPaths);
STAT_INT_DISTRIB("SPPM/Grid cells per visible point", gridCellsPerVisiblePoint);
STAT_MEMORY_COUNTER("Memory/SPPM Pixels", pixelMemoryBytes);
STAT_FLOAT_DISTRIB("Memory/SPPM BSDF and Grid Memory", memoryArenaMB);

struct SPPMIntegrator::SPPMPixel {
    bool hasVisiblePoint() const { return !vp.beta.isBlack(); }

    struct VisiblePoint {
        VisiblePoint() {}
        VisiblePoint(Point3f &p, Vector3f &wo, const BSDF *bsdf, Spectrum &beta)
            : p(p), wo(wo), bsdf(bsdf), beta(beta) {}
        Point3f p;
        Vector3f wo;
        const BSDF *bsdf = nullptr;
        Spectrum beta = 0;
    } vp;
    float radius = 0;
    Spectrum Ld;
    AtomicFloat Phi[Spectrum::nSamples]; // Spectrum is not thread-safe
    atomic<int> M = 0;
    float N = 0;
    Spectrum tau;
};

struct SPPMIntegrator::SPPMPixelListNode {
    SPPMPixel *pixel;
    SPPMPixelListNode *next;
};

class SPPMIntegrator::VisiblePointGrid {
public:
    VisiblePointGrid(SPPMPixel *pixels, int nPixels, vector<MemoryArena> &perThreadArenas)
        : hashSize(nPixels), grid(nPixels)
    {
        ProfilePhase _(Stage::SPPMGridConstruction);
        // Compute grid bounds
        float maxRadius = 0;
        for (int i = 0; i < nPixels; i++) {
            const SPPMPixel &pixel = pixels[i];
            if (!pixel.hasVisiblePoint()) continue;
            Bounds3f vpBound = expand(Bounds3f(pixel.vp.p), pixel.radius);
            bounds = unionOf(bounds, vpBound);
            maxRadius = max(maxRadius, pixel.radius);
        }

        // Compute resolution of grid in each dimension
        Vector3f diag = bounds.diagonal();
        float maxDiag = maxComp(diag);
        int baseGridRes = int(maxDiag / maxRadius);
        for (int i = 0; i < 3; i++)
            resolution[i] = max(int(baseGridRes * diag[i] / maxDiag), 1);

        // Add visible points to grid
        Parallel::forLoop([&] (int pixelIndex) {
            auto &arena = perThreadArenas[Parallel::getThreadIndex()];
            SPPMPixel &pixel = pixels[pixelIndex];
            if (pixel.hasVisiblePoint()) {
                float radius = pixel.radius;
                Point3i pMin, pMax;
                toGridIndex(pixel.vp.p - Vector3f(radius), &pMin);
                toGridIndex(pixel.vp.p + Vector3f(radius), &pMax);

                for (int z = pMin.z; z <= pMax.z; z++)
                    for (int y = pMin.y; y <= pMax.y; y++)
                        for (int x = pMin.x; x <= pMax.x; x++) {
                            unsigned h = hash(Point3i(x, y, z));
                            auto *node = arena.alloc<SPPMPixelListNode>();
                            node->pixel = &pixel;
                            node->next = grid[h];
                            while (!grid[h].compare_exchange_weak(node->next, node));
                        }

            }
        }, nPixels, 4096);
    }

    bool toGridIndex(const Point3f &p, Point3i *pi) const {
        bool inBounds = true;
        Vector3f pg = bounds.offset(p);
        for (int i = 0; i < 3; i++) {
            (*pi)[i] = int(resolution[i] * pg[i]);
            inBounds &= ((*pi)[i] >= 0 && (*pi)[i] < resolution[i]);
            (*pi)[i] = clamp((*pi)[i], 0, resolution[i] - 1);
        }
        return inBounds;
    }

    atomic<SPPMPixelListNode *> & operator [] (const Point3i &index) { return grid[hash(index)]; }

private:
    unsigned hash(const Point3i &p) const {
        unsigned hash = (p.x * 73856093) ^ (p.y * 19349663) ^ (p.z * 83492791);
        return hash % unsigned(hashSize);
    }

    const int hashSize;
    vector<atomic<SPPMPixelListNode *>> grid;
    Bounds3f bounds;
    int resolution[3];
};

void SPPMIntegrator::render(const Scene &scene) {
    ProfilePhase __(Stage::IntegratorRender);
    // Initialize _pixelBounds_ and pixel array
    Bounds2i pixelBounds = camera->film->croppedPixelBounds;
    int nPixels = pixelBounds.area();
    unique_ptr<SPPMPixel[]> pixels(new SPPMPixel[nPixels]);
    for (int i = 0; i < nPixels; i++)
        pixels[i].radius = initialSearchRadius;
    const float invSqrtSPP = 1.0f / sqrt(nIterations);
    pixelMemoryBytes += nPixels * sizeof(SPPMPixel);

    // Compute light distribution for sampling lights proportional to light power
    auto lightDistrib = computeLightPowerDistribution(scene.lights);

    // Perform integration
    HaltonSampler sampler(nIterations, pixelBounds);

    // Compute number of tiles to use for camera pass
    Vector2i pixelExtent = pixelBounds.diagonal();
    constexpr int tileSize = 16;
    Point2i nTiles((pixelExtent.x + tileSize - 1) / tileSize, (pixelExtent.y + tileSize - 1) / tileSize);

    ProgressReporter progress(2 * nIterations, "Rendering");
    for (int ite = 0; ite < nIterations; ite++) {
        // Generate visible points
        vector<MemoryArena> perThreadArenas(Parallel::maxThreadIndex());
        {
            ProfilePhase _(Stage::SPPMCameraPass);
            Parallel::forLoop2D([&] (Point2i tile) {
                auto &arena = perThreadArenas[Parallel::getThreadIndex()];

                // Follow camera paths for _tile_ in image
                int tileIndex = tile.y * nTiles.x + tile.x;
                auto tileSampler = sampler.clone(tileIndex);

                // Compute tile bounds
                int x0 = pixelBounds.pMin.x + tile.x * tileSize;
                int x1 = min(x0 + tileSize, pixelBounds.pMax.x);
                int y0 = pixelBounds.pMin.y + tile.y * tileSize;
                int y1 = min(y0 + tileSize, pixelBounds.pMax.y);
                Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));

                for (auto pPixel : tileBounds) {
                    // Prepare _tileSampler_ for _pPixel_
                    tileSampler->startPixel(pPixel);
                    tileSampler->setSampleNumber(ite); // for each sample number, for each pixel

                    // Generate camera rays for pixel
                    auto cameraSample = tileSampler->getCameraSample(pPixel);
                    RayDifferential ray;
                    Spectrum beta = camera->generateRayDifferential(cameraSample, &ray);
                    if (beta.isBlack()) continue;
                    ray.scaleDifferentials(invSqrtSPP);

                    // Follow camera ray path until a visible point is created
                    // Get _SPPMPixel_ for _pPixel
                    Point2i pPixelO = Point2i(pPixel - pixelBounds.pMin);
                    int pixelOffset = pPixelO.x + pPixelO.y * (pixelBounds.pMax.x - pixelBounds.pMin.x);
                    SPPMPixel &pixel = pixels[pixelOffset];

                    bool specularBounce = false;
                    for (int depth = 0; depth < maxDepth; depth++) {
                        SurfaceInteraction isect;
                        totalPhotonSurfaceInteraction++;
                        if (!scene.intersect(ray, &isect)) {
                            for (const auto &light : scene.lights)
                                pixel.Ld += beta * light->compute_Le(ray);
                            break;
                        }

                        // Process camera ray intersection
                        // Compute BSDF
                        isect.computeScatteringFunctions(ray, arena, true);
                        if (!isect.bsdf) {
                            ray = isect.spawnRay(ray.d);
                            --depth;
                            continue;
                        }

                        // Accumulate direct illumination
                        Vector3f wo = -ray.d;
                        if (depth == 0 || specularBounce)
                            pixel.Ld += beta * isect.compute_Le(wo);
                        pixel.Ld += beta * uniformSampleOneLight(isect, scene, arena, *tileSampler);

                        // Possibly create visible points and end camera path
                        bool isDiffuse = isect.bsdf->numComponents(BxDFType(BSDF_DIFFUSE | BSDF_REFLECTION
                                                                            | BSDF_TRANSMISSION)) > 0;
                        bool isGlossy = isect.bsdf->numComponents(BxDFType(BSDF_GLOSSY | BSDF_REFLECTION
                                                                           | BSDF_TRANSMISSION)) > 0;
                        if (isDiffuse || (isGlossy && depth == maxDepth - 1)) {
                            pixel.vp = SPPMPixel::VisiblePoint(isect.p, wo, isect.bsdf, beta);
                            break;
                        }

                        // Spawn ray from camera path vertex
                        if (depth < maxDepth - 1) {
                            Vector3f wi;
                            float pdf;
                            BxDFType flags;
                            Spectrum f = isect.bsdf->sample_f(wo, &wi, tileSampler->get2D(), &pdf, BSDF_ALL,
                                                              &flags);
                            if (f.isBlack() || pdf == 0) break;
                            beta *= f * absDot(wi, isect.shading.n) / pdf;
                            specularBounce = (flags & BSDF_SPECULAR) != 0;

                            if (beta.luminance() < 0.25) {
                                float continueProb = min(1.0f, beta.luminance());
                                if (tileSampler->get1D() > continueProb) break;
                                beta /= continueProb;
                            }

                            ray = RayDifferential(isect.spawnRay(wi));
                        } // end spawn ray

                    } // end following camera path
                } // end tile bounds traversal
            }, nTiles);

        } // end camera pass
        progress.update();

        // Create grid of all visible points
        auto grid = VisiblePointGrid(pixels.get(), nPixels, perThreadArenas);

        // Trace photons and accumulate contributions
        {
            ProfilePhase _(Stage::SPPMPhotonPass);
            using namespace LowDiscrepancy;
            vector<MemoryArena> photonArenas(Parallel::maxThreadIndex());
            Parallel::forLoop([&] (int photonIndex) {
                auto &arena = photonArenas[Parallel::getThreadIndex()];
                uint64_t haltonIndex = uint64_t(ite) * uint64_t(photonsPerIteration) + uint64_t(photonIndex);
                int haltonDim = 0;

                // Choose light to shoot photons from
                float lightPdf;
                float lightSample = radicalInverse(haltonDim++, haltonIndex);
                int lightNum = lightDistrib->sampleDiscrete(lightSample, &lightPdf);
                const auto &light = scene.lights[lightNum];

                // Compute sample values for photon ray leaving light source
                Point2f uLight0(radicalInverse(haltonDim, haltonIndex),
                                radicalInverse(haltonDim + 1, haltonIndex));
                Point2f uLight1(radicalInverse(haltonDim + 2, haltonIndex),
                                radicalInverse(haltonDim + 3, haltonIndex));
                float uLightTime = lerp(radicalInverse(haltonDim + 4, haltonIndex),
                                        camera->shutterOpen, camera->shutterClose);
                haltonDim += 5;

                // Generate photon ray from light source
                RayDifferential photonRay;
                Normal3f nLight;
                float pdfPos, pdfDir;
                Spectrum Le = light->sample_Le(uLight0, uLight1, uLightTime, &photonRay, &nLight, &pdfPos, &pdfDir);
                if (Le.isBlack() || pdfPos == 0 || pdfDir == 0) return;
                Spectrum beta = absDot(nLight, photonRay.d) * Le / (lightPdf * pdfPos * pdfDir);
                if (beta.isBlack()) return;

                // Follow photon path throught the scene and record intersections
                SurfaceInteraction isect;
                for (int depth = 0; depth < maxDepth; depth++) {
                    if (!scene.intersect(photonRay, &isect)) break;
                    totalPhotonSurfaceInteraction++;
                    if (depth > 0) { // only account for indirect illumination
                        Point3i index;
                        if (grid.toGridIndex(isect.p, &index)) {
                            for (auto *node = grid[index].load(memory_order_relaxed); node; node = node->next) {
                                visiblePhotonChecked++;
                                auto &pixel = *node->pixel;
                                float radius = pixel.radius;
                                if (distanceSq(pixel.vp.p, isect.p) > SQ(radius)) continue;

                                // Update pixel Φ and M for nearby photon
                                Vector3f wi = -photonRay.d;
                                Spectrum Phi = beta * pixel.vp.bsdf->compute_f(pixel.vp.wo, wi);
                                for (int i = 0; i < Spectrum::nSamples; i++)
                                    pixel.Phi[i].add(Phi[i]);
                                pixel.M++;
                            }
                        }
                    }

                    // Sample new photon ray direction
                    // Compute BSDF at photon intersection point
                    isect.computeScatteringFunctions(photonRay, arena, true, TransportMode::Importance);
                    if (!isect.bsdf) {
                        --depth;
                        photonRay = isect.spawnRay(photonRay.d);
                        continue;
                    }

                    // Sample BSDF for reflected photon
                    Vector3f wi, wo = -photonRay.d;
                    float pdf;
                    BxDFType flags;
                    Point2f bsdfSample(radicalInverse(haltonDim, haltonIndex),
                                       radicalInverse(haltonDim + 1, haltonIndex));
                    haltonDim += 2;
                    Spectrum f = isect.bsdf->sample_f(wo, &wi, bsdfSample, &pdf, BSDF_ALL, &flags);
                    if (f.isBlack() || pdf == 0) break;
                    Spectrum bnew = beta * f * absDot(wi, isect.shading.n) / pdf;

                    // Possibly terminate photon path using Russian Roulette
                    float q = max(0.0f, 1 - bnew.luminance() / beta.luminance());
                    if (radicalInverse(haltonDim++, haltonIndex) < q) break;
                    beta = bnew / (1 - q);
                    photonRay = RayDifferential(isect.spawnRay(wi));
                }
                arena.reset();
            }, photonsPerIteration, 8192);
            photonPaths += photonsPerIteration;
            progress.update();
        }

        // Update pixel values from this pass's photons
        {
            ProfilePhase _(Stage::SPPMStatsUpdate);
            Parallel::forLoop([&] (int i) {
                SPPMPixel &pixel = pixels[i];
                if (pixel.M > 0) {
                    // Update photon count, search radius and τ from photons
                    constexpr float gamma = 2.0f / 3.0f;
                    float Nnew = pixel.N + gamma * pixel.M;
                    float rNew = pixel.radius * sqrt(Nnew / (pixel.N + pixel.M));
                    Spectrum Phi;
                    for (int j = 0; j < Spectrum::nSamples; j++)
                        Phi[j] = pixel.Phi[j];
                    pixel.tau = (pixel.tau + pixel.vp.beta * Phi) * SQ(rNew) / SQ(pixel.radius);
                    pixel.N = Nnew;
                    pixel.radius = rNew;
                    pixel.M = 0;
                    for (int j = 0; j < Spectrum::nSamples; j++)
                        pixel.Phi[j] = 0.0f;
                }

                // Reset visible point in pixel
                pixel.vp.beta = 0;
                pixel.vp.bsdf = nullptr;
            }, nPixels, 4096);
        }

        // Periodically store image in film and write image
        if (ite + 1 == nIterations || ((ite + 1) % writeFrequency) == 0) {
            int x0 = pixelBounds.pMin.x;
            int x1 = pixelBounds.pMax.x;
            uint64_t Np = uint64_t(ite + 1) * uint64_t(photonsPerIteration);
            unique_ptr<Spectrum[]> image(new Spectrum[pixelBounds.area()]);
            int offset = 0;
            for (int y = pixelBounds.pMin.y; y < pixelBounds.pMax.y; ++y) {
                for (int x = x0; x < x1; ++x) {
                    // Compute radiance for SPPM pixel _pixel_
                    const SPPMPixel &pixel = pixels[(y - pixelBounds.pMin.y) * (x1 - x0) + (x - x0)];
                    Spectrum L = pixel.Ld / (ite + 1);
                    L += pixel.tau / (Np * PI * pixel.radius * pixel.radius);
                    image[offset++] = L;
                }
            }
            camera->film->setImage(image.get());
            camera->film->writeImage();
        }// end write image
    } // end render iteration

    progress.done();
}

Integrator * SPPMIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                    shared_ptr<const Camera> camera)
{
    int nIterations = params.findOneInt("iterations", params.findOneInt("numiterations", 64));
    int maxDepth = params.findOneInt("maxdepth", 5);
    int photonsPerIter = params.findOneInt("photonsperiteration", 500000);
    int writeFreq = params.findOneInt("imagewritefrequency", 1 << 5);
    float radius = params.findOneFloat("radius", 1.f);
    if (Renderer::options.quickRender) nIterations = max(1, nIterations / 16);
    return new SPPMIntegrator(camera, nIterations, photonsPerIter, maxDepth, radius, writeFreq);
}

