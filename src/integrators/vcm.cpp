#include "vcm.h"
#include "stats.h"
#include "samplers/sobol.h"
#include "stats.h"

STAT_RATIO("VCM/Light vertices per iteration", storedLightVertices, nIterations)

using LightIndexMap = BDPTIntegrator::LightIndexMap;

class VCMIntegrator::LightVerticesGrid {
public:
    struct VertexNode {
        Vertex *vertex;
        VertexNode *next;
    };

    LightVerticesGrid(Vertex **vertices, vector<int> &pathLengths, float searchRadius, int maxDepth,
                      const Bounds3f &worldBound, vector<MemoryArena> &arenas)
        : hashSize(pathLengths.size()), grid(pathLengths.size()), searchRadius(searchRadius),
          bounds(worldBound)
    {
        // Compute resolution of grid in each dimension
        Vector3f diag = bounds.diagonal();
        float maxDiag = maxComp(diag);
        int baseGridRes = int(maxDiag / searchRadius);
        for (int i = 0; i < 3; i++)
            resolution[i] = max(int(baseGridRes * diag[i] / maxDiag), 1);

         // Add light vertices to grid
        Parallel::forLoop([&] (int pixelIndex) {
            auto &arena = arenas[Parallel::getThreadIndex()];
            Vertex *path = vertices[pixelIndex];
            int pathLength = pathLengths[pixelIndex];

            for (int i = 1; i < pathLength; i++) {
                Vertex *vertex = path + i;
                if (vertex->beta.isBlack()) continue;
                if (vertex->isOnSurface() && vertex->si.bsdf) {
                    bool isDiffuse = vertex->si.bsdf->numComponents(BxDFType(BSDF_DIFFUSE | BSDF_REFLECTION
                                                                        | BSDF_TRANSMISSION)) > 0;
                    bool isGlossy = vertex->si.bsdf->numComponents(BxDFType(BSDF_GLOSSY | BSDF_REFLECTION
                                                                       | BSDF_TRANSMISSION)) > 0;
                    if (!isDiffuse && !(isGlossy && i == (maxDepth - 1) )) continue;
                }

                Point3i pGrid;
                toGridIndex(vertex->p(), &pGrid);
                unsigned h = hash(pGrid);
                auto node = arena.alloc<VertexNode>();
                node->vertex = vertex;
                node->next = grid[h];
                while (!grid[h].compare_exchange_weak(node->next, node));

                /* Point3i pMin, pMax;
                toGridIndex(vertex->p() - Vector3f(searchRadius), &pMin);
                toGridIndex(vertex->p() + Vector3f(searchRadius), &pMax);

                for (int z = pMin.z; z <= pMax.z; z++)
                    for (int y = pMin.y; y <= pMax.y; y++)
                        for (int x = pMin.x; x <= pMax.x; x++) {
                            unsigned h = hash(Point3i(x, y, z));
                            auto *node = arena.alloc<VertexNode>();
                            node->vertex = vertex;
                            node->next = grid[h];
                            while (!grid[h].compare_exchange_weak(node->next, node));
                        }*/

                storedLightVertices++;
            }

        }, hashSize, 4096);
    }

    void query(const Vertex *pt, vector<Vertex *> &lightVertices) const {
        Point3i pMin, pMax;
        toGridIndex(pt->p() - Vector3f(searchRadius), &pMin);
        toGridIndex(pt->p() + Vector3f(searchRadius), &pMax);

        Spectrum L;
        for (int px = pMin.x; px <= pMax.x; px++)
            for (int py = pMin.y; py <= pMax.y; py++)
                for (int pz = pMin.z; pz <= pMax.z; pz++) {
                    Point3i index(px, py, pz);
                    for (auto *node = grid[hash(index)].load(memory_order_relaxed); node; node = node->next) {
                        auto *vertex = node->vertex;
                        if (distanceSq(pt->p(), vertex->p()) > SQ(searchRadius)) continue;
                        lightVertices.push_back(vertex);
                    }
                }
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

    const atomic<VertexNode *> & operator [] (const Point3i &index) const { return grid[hash(index)]; }

private:
    inline unsigned hash(const Point3i &p) const {
        unsigned hash = (p.x * 73856093) ^ (p.y * 19349663) ^ (p.z * 83492791);
        return hash % unsigned(hashSize);
    }

    const int hashSize;
    vector<atomic<VertexNode *>> grid;
    float searchRadius;
    Bounds3f bounds;
    int resolution[3];
};

template <class T>
class VCMIntegrator::ScopedAssignment {
public:
    ScopedAssignment(T *target = nullptr, T value = T()) : target(target) {
        if (target) {
            backup = *target;
            *target = value;
        }
    }

    ~ScopedAssignment() { if (target) *target = backup; }

private:
    T *target;
    T backup;
};

#define SPHERE_VOLUME(r) ((4.0f / 3.0f) * PI * CUB(r))
#define REMAP0(f) ((f) == 0 ? 1 : (f))

Spectrum VCMIntegrator::mergeVertices(const LightVerticesGrid &grid, Vertex **lightPaths, Vertex *cameraVertices,
                                      int t, int nLightPaths, float searchRadius, float *weight) const
{
    Vertex *pt = &cameraVertices[t - 1];
    if (pt->type == VertexType::Light) return 0;
    if (pt->isOnSurface()) {
        bool isDiffuse = pt->si.bsdf->numComponents(BxDFType(BSDF_DIFFUSE | BSDF_REFLECTION
                                                            | BSDF_TRANSMISSION)) > 0;
        bool isGlossy = pt->si.bsdf->numComponents(BxDFType(BSDF_GLOSSY | BSDF_REFLECTION
                                                           | BSDF_TRANSMISSION)) > 0;
        if (!isDiffuse && !(isGlossy && t == (maxDepth - 1) )) return 0;
    }

    // Precompute camera weight
    float camPdf = 1;
    for (int i = 1; i < t; i++)
        camPdf *= REMAP0(cameraVertices[i].pdfFwd);

    Spectrum L;
    vector<Vertex *> lightVertices;
    grid.query(pt, lightVertices);

    for (auto vertex : lightVertices) {
        Vertex *path = lightPaths[vertex->pathIndex];
        Spectrum Lpath;
        float pathWeight = 0, pdfVC = camPdf;
        if (vertex == &path[0]) continue; // ignore light vertex

        for (int i = 0; &path[i] != vertex; i++)
            pdfVC *= REMAP0(path[i].pdfFwd);

        if (pt->type == VertexType::Surface) // Surface estimate
        {
            Spectrum phi = pt->beta * pt->si.bsdf->compute_f(pt->wo(), vertex->wo()) * vertex->beta;
            Lpath = phi / (nLightPaths * PI * SQ(searchRadius));
            pathWeight = PI * SQ(searchRadius) * REMAP0(vertex->pdfFwd) * pdfVC;
        }
        else if (pt->type == VertexType::Medium) // Volume estimate
        {
            Spectrum phi = pt->beta * pt->mi.phase->compute_p(pt->wo(), vertex->wo()) * vertex->beta;
            Spectrum sigma_s = vertex->mi.getMedium(Vector3f(vertex->ns()))->get_sigma_s(&vertex->mi);
            Lpath = phi / (nLightPaths * sigma_s * SPHERE_VOLUME(searchRadius));
            pathWeight = sigma_s.luminance() * SPHERE_VOLUME(searchRadius) * REMAP0(vertex->pdfFwd) * pdfVC;
        }
        else continue;

        *weight += pathWeight;
        L += Lpath;
    }

    return L;
}

#define REMAP0(f) ((f) == 0 ? 1 : (f))

void VCMIntegrator::render(const Scene &scene) {
    ProfilePhase __(Stage::IntegratorRender);
    Stats::nIterations = nIterations;

    // Initialize pixel bounds
    Film *film = camera->film;
    const Bounds2i sampleBounds = film->getSampleBounds();
    const Vector2i sampleExtent = sampleBounds.diagonal();
    int nPixels = sampleBounds.area();
    constexpr int tileSize = 16;
    const int nXTiles = (sampleExtent.x + tileSize - 1) / tileSize;
    const int nYTiles = (sampleExtent.y + tileSize - 1) / tileSize;

    // Initialize sampler
    SobolSampler globalSampler(nIterations, sampleBounds);

    // Initialize light distribution
    if (scene.lights.empty())
        WARNING("No lights in the scene. Rendering black image...");
    auto lightDistrib = computeLightPowerDistribution(scene.lights);
    LightIndexMap lightToIndex;
    for (size_t i = 0; i < scene.lights.size(); i++)
        lightToIndex[scene.lights[i].get()] = i;

    // Main render loop
    ProgressReporter reporter(nIterations * 2, "Rendering ");
    for (int ite = 0; ite < nIterations; ite++) {
        vector<MemoryArena> lightArenas(Parallel::maxThreadIndex());

        // Sample light path
        unique_ptr<Vertex *[]> lightPaths(new Vertex *[nPixels]);
        vector<int> lightPathLengths(nPixels);
        constexpr int chunkSize = 256;
        int nChunk = nPixels / chunkSize + 1;

        Parallel::forLoop([&] (int chunkIndex) {
            auto &arena = lightArenas[Parallel::getThreadIndex()];
            auto sampler = globalSampler.clone(chunkIndex);
            for (int i = 0; i < chunkSize; i++) {
                int pathIndex = chunkIndex * chunkSize + i;
                if (pathIndex >= nPixels) return;
                sampler->startPixel(Point2i(chunkIndex, i));
                sampler->setSampleNumber(ite);
                lightPaths[pathIndex] = arena.alloc<Vertex>(maxDepth + 1);
                lightPathLengths[pathIndex] =
                        BDPTIntegrator::generateLightSubpath(scene, *sampler, arena, maxDepth, sampler->get1D(),
                                                             *lightDistrib, lightToIndex, lightPaths[pathIndex]);
                for (int k = 0; k < lightPathLengths[pathIndex]; k++)
                    lightPaths[pathIndex][k].pathIndex = pathIndex; // append pathIndex to vertices for VM
            }
        }, nChunk);

        // Build range search struct
        float radius = initialSearchRadius * pow(float(ite + 1), 0.5f * (alpha - 1));
        LightVerticesGrid grid(lightPaths.get(), lightPathLengths, radius, maxDepth, scene.getWorldBound(),
                               lightArenas);
        reporter.update();

        Parallel::forLoop2D([&] (const Point2i tile) {
            MemoryArena arena;
            int seed = tile.y * nXTiles + tile.x;
            auto tileSampler = globalSampler.clone(seed);

            // Get film tile
            int x0 = sampleBounds.pMin.x + tile.x * tileSize;
            int x1 = min(x0 + tileSize, sampleBounds.pMax.x);
            int y0 = sampleBounds.pMin.y + tile.y * tileSize;
            int y1 = min(y0 + tileSize, sampleBounds.pMax.y);
            Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
            auto filmTile = camera->film->getFilmTile(tileBounds);

            for (Point2i pPixel : tileBounds) {
                tileSampler->startPixel(pPixel);
                tileSampler->setSampleNumber(ite);
                // Compute pixel offset
                Point2i pPixelO = Point2i(pPixel - sampleBounds.pMin);
                int pixelOffset = pPixelO.x + pPixelO.y * (sampleBounds.pMax.x - sampleBounds.pMin.x);

                // Sample camera path
                Point2f pFilm = Point2f(pPixel) + tileSampler->get2D();
                Vertex *cameraVertices = arena.alloc<Vertex>(maxDepth + 2);
                int nCamera = BDPTIntegrator::generateCameraSubpath(scene, *tileSampler, arena, maxDepth, *camera,
                                                                    pFilm, cameraVertices);
                Vertex *lightVertices = lightPaths[pixelOffset];
                int nLight = lightPathLengths[pixelOffset];

                Spectrum L;
                float weightSum = 0;
                for (int t = 1; t <= nCamera; t++) {
                    // Vertex connection
                    /* for (int s = 0; s <= nLight; s++) {
                        int depth = t + s - 2;
                        if ((s == 1 && t == 1) || depth < 0 || depth > maxDepth) continue;

                        // Vertex connection
                        Point2f pFilmNew = pFilm;
                        float vcMISWeight = 0;
                        Spectrum Lvc =
                                BDPTIntegrator::connectVertices(scene, lightVertices, cameraVertices, s, t,
                                                                *lightDistrib, lightToIndex, *camera,
                                                                *tileSampler, &pFilmNew, &vcMISWeight);
                        if (t == 1) // connect directly to camera
                            film->addSplat(pFilmNew, Lvc * vcMISWeight);
                        else
                            L += vcMISWeight * Lvc;
                    }*/

                    // Vertex merging (for t >= 2)
                    if (t >= 2) {
                        Spectrum Lvm = mergeVertices(grid, lightPaths.get(), cameraVertices, t, nPixels, radius,
                                                     &weightSum);
                        L += Lvm;
                    }
                }
                // if (weightSum != 0) L /= weightSum;
                filmTile->addSample(pFilm, L);
            } // end each pixel in tile bounds

            film->mergeFilmTile(move(filmTile));
        }, Point2i(nXTiles, nYTiles));

        reporter.update();
    } // end iteration

    film->writeImage(1.0f / nIterations);
    reporter.done();
}

VCMIntegrator *VCMIntegrator::create(const ParamSet &params, shared_ptr<Sampler> sampler,
                                     shared_ptr<const Camera> camera)
{
    int maxDepth = params.findOneInt("maxdepth", 7);
    int nIterations = params.findOneInt("iterations", params.findOneInt("numiterations", 10));
    float radius = params.findOneFloat("radius", 0.01f);
    float alpha = params.findOneFloat("alpha", 0.75f);
    return new VCMIntegrator(camera, nIterations, maxDepth, radius, alpha);
}
