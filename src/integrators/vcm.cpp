#include "vcm.h"
#include "stats.h"
#include "samplers/sobol.h"

class VCMIntegrator::RangeQuery {
public:
    RangeQuery(const VCMIntegrator &vcm, const Vertex &cameraVertex, float radius)
        : vcm(vcm), cameraVertex(cameraVertex), radius(radius) {}

    const Spectrum & getContribution() const { return contrib; }

    const Point3f & getPosition() const { return cameraVertex.p(); }

    void process(const Vertex &lightVertex) {
        // Reject if full path length not in range
        int depth = lightVertex.pathLength + cameraVertex.pathLength;
        if (depth > vcm.getMaxDepth() || depth < 2) return;

        // Compute forward and reverse pdf w.r.t. solid angle
        Vector3f dirCam = cameraVertex.wo(), dirLight = lightVertex.wo();
        Spectrum f = cameraVertex.compute_f(dirLight, TransportMode::Radiance);
        if (f.isBlack()) return;
        float pdfLightRevW = lightVertex.pdfW(dirCam, dirLight); // equation 38
        float pdfCamRevW = cameraVertex.pdfW(dirLight, dirCam); // equation 39

        // Compute MIS weight
        float wVC = vcm.getVCWeightFactor();
        float wLight = lightVertex.dVCM * wVC + lightVertex.dVM * pdfLightRevW;
        float wCam = cameraVertex.dVCM * wVC + cameraVertex.dVM * pdfCamRevW;
        float weight = vcm.doPPM() ? 1.0f : 1.0f / (wLight + 1.0f + wCam); // equation 37

        // Compute normalization factor
        Spectrum norm;
        int nLightSubpaths = vcm.getLightSubpathNumber();
        if (lightVertex.type == VertexType::Surface)
            norm = 1.0f / (nLightSubpaths * PI * SQ(radius));
        else {
            Spectrum sigma_s = lightVertex.mi.getMedium()->get_sigma_s(&lightVertex.mi);
            norm = Spectrum(1.0f) / (nLightSubpaths * 4.0f / 3.0f * PI * CUB(radius) * sigma_s);
        }

        contrib += weight * norm * cameraVertex.beta * f * lightVertex.beta;
    }

private:
    const VCMIntegrator &vcm;
    const Vertex &cameraVertex;
    const float radius;
    Spectrum contrib;
};

class VCMIntegrator::LightVerticesGrid {
public:
    LightVerticesGrid(Vertex **vertices, vector<int> &pathLengths, float searchRadius, const Bounds3f &worldBound,
                      vector<MemoryArena> &arenas)
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

            for (int i = 0; i < pathLength; i++) {
                Vertex *vertex = path + i;
                if (vertex->beta.isBlack()) continue;

                Point3i pGrid;
                toGridIndex(vertex->p(), &pGrid);
                unsigned h = hash(pGrid);
                auto node = arena.alloc<VertexNode>();
                node->vertex = vertex;
                node->next = grid[h];
                while (!grid[h].compare_exchange_weak(node->next, node));
            }

        }, hashSize, 4096);
    }

    void process(RangeQuery &query) const {
        Point3f center = query.getPosition();
        Point3i pMin, pMax;
        toGridIndex(center - Vector3f(searchRadius), &pMin);
        toGridIndex(center + Vector3f(searchRadius), &pMax);

        Spectrum L;
        for (int px = pMin.x; px <= pMax.x; px++)
            for (int py = pMin.y; py <= pMax.y; py++)
                for (int pz = pMin.z; pz <= pMax.z; pz++) {
                    Point3i index(px, py, pz);
                    for (auto *node = grid[hash(index)].load(memory_order_relaxed); node; node = node->next) {
                        auto *vertex = node->vertex;
                        if (distanceSq(center, vertex->p()) > SQ(searchRadius)) continue;
                        query.process(*vertex);
                    }
                }
    }

private:
    struct VertexNode {
        Vertex *vertex;
        VertexNode *next;
    };

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

    uint hash(const Point3i &p) const {
        uint hash = (p.x * 73856093) ^ (p.y * 19349663) ^ (p.z * 83492791);
        return hash % uint(hashSize);
    }

    const int hashSize;
    vector<atomic<VertexNode *>> grid;
    float searchRadius;
    Bounds3f bounds;
    int resolution[3];
};

VCMIntegrator::VCMIntegrator(shared_ptr<const Camera> &camera, AlgorithmType algorithm, int nIterations,
                             int maxDepth, float initialRadius, float alpha)
    : camera(camera), nIterations(nIterations), maxDepth(maxDepth), initialRadius(initialRadius), alpha(alpha)
{
    switch (algorithm) {
    case AlgorithmType::LightTrace:
        lightTraceOnly = true;
        break;
    case AlgorithmType::PPM:
        usePPM = true;
        useVM = true;
        break;
    case AlgorithmType::BPM:
        useVM = true;
        break;
    case AlgorithmType::BDPT:
        useVC = true;
        break;
    case AlgorithmType::VCM:
        useVC = true;
        useVM = true;
        break;
    }
}

void VCMIntegrator::lightPDFs(const Light *light, float pdfChoice, float pdfPos, float pdfDir, float *pdfTrace,
                              float *pdfConnect) const
{
    switch (LightFlags(light->flags)) {
    case LightFlags::Area:
        *pdfTrace = pdfChoice * pdfPos * pdfDir;
        *pdfConnect = pdfChoice * pdfPos;
        break;
    case LightFlags::DeltaDirection:
        *pdfTrace = pdfChoice * pdfPos;
        *pdfConnect = pdfChoice;
        break;
    case LightFlags::DeltaPosition:
        *pdfTrace = pdfChoice * pdfDir;
        *pdfConnect = pdfChoice;
        break;
    case LightFlags::Infinite:
        *pdfTrace = pdfChoice * pdfPos * pdfDir;
        *pdfConnect = pdfChoice * pdfDir;
    }
}

VCMIntegrator::SubpathState
VCMIntegrator::generateLightSample(const Scene &scene, Sampler &sampler) const
{
    SubpathState lightState;

    // Sample initial ray for light subpath
    float pdfChoice;
    int lightNum = lightDistrib->sampleDiscrete(sampler.get1D(), &pdfChoice);
    const auto &light = scene.lights[lightNum];
    RayDifferential ray;
    Normal3f nLight;
    float pdfPos, pdfDir;
    Spectrum Le = light->sample_Le(sampler.get2D(), sampler.get2D(), sampler.get1D(), &ray, &nLight,
                                   &pdfPos, &pdfDir);
    if (Le.isBlack() || pdfPos == 0 || pdfDir == 0) return lightState; // beta == 0 indicates invalid sample

    // Compute tracing and connection pdfs
    float pdfTrace, pdfConnect;
    lightPDFs(light.get(), pdfChoice, pdfPos, pdfDir, &pdfTrace, &pdfConnect);

    // Initialize light subpath state
    float cosAtLight = absDot(nLight, ray.d);
    lightState.ray = ray;
    lightState.beta = Le * cosAtLight / pdfTrace;
    lightState.pathLength = 1;
    lightState.isInfiniteLight = light->isInfiniteLight();
    lightState.isSpecularPath = true;

    // Compute MIS quantities
    lightState.dVCM = pdfConnect / pdfTrace;
    if (light->isDeltaLight())
        lightState.dVC = 0.0f;
    else {
        float usedCosLight = light->isInfiniteLight() ? 1.0f : cosAtLight;
        lightState.dVC = usedCosLight / pdfTrace;
    }
    lightState.dVM = lightState.dVC * wVC;

    return lightState;
}

int VCMIntegrator::lightRandomWalk(const Scene &scene, Sampler &sampler, MemoryArena &arena,
                                   SubpathState &lightState, Vertex *path, Film *film) const
{
    RayDifferential &ray = lightState.ray;
    Spectrum &beta = lightState.beta;
    int &length = lightState.pathLength;
    int stored = 0;

    while (true) {
        // Trace a ray and sample the medium, if any
        MediumInteraction mi;
        SurfaceInteraction isect;
        bool foundIsect = scene.intersect(ray, &isect);
        if (ray.medium)
            beta *= ray.medium->sample(ray, sampler, arena, &mi);
        if (beta.isBlack()) break;
        bool isMediumInteraction = mi.isValid();

        // Compute scattering functions for mode and skip over medium boundaries
        if (!isMediumInteraction) {
            if (!foundIsect) break;
            isect.computeScatteringFunctions(ray, arena, true, TransportMode::Importance);
            if (!isect.bsdf) {
                ray = isect.spawnRay(ray.d);
                continue;
            }
        }

        // Complete MIS quantities evaluation
        if (lightState.pathLength > 1 || !lightState.isInfiniteLight) {
            float distSq = SQ(ray.tMax) / ray.d.lengthSq();
            lightState.dVCM *= distSq;
        }
        if (!isMediumInteraction) { // cosine fix only applies to surface interaction
            float cosTheta = dot(isect.wo, isect.shading.n);
            lightState.dVCM /= cosTheta;
            lightState.dVC /= cosTheta;
            lightState.dVM /= cosTheta;
        }

        // Store light vertices
        if (isMediumInteraction)
            path[stored++] = Vertex(lightState, mi);
        else if (!lightState.isDelta() && (useVC || useVM))
            path[stored++] = Vertex(lightState, isect);

        // Connect to camera
        if (!lightState.isDelta() && (useVC || lightTraceOnly))
            connectToCamera(scene, sampler, path[stored - 1], film);

        // Sample scattering functions and continue random walk
        if (++length >= maxDepth) break;
        if (!sampleScattering(sampler, path[stored - 1], lightState, TransportMode::Importance)) break;
    }

    return stored;
}

void VCMIntegrator::connectToCamera(const Scene &scene, Sampler &sampler, const Vertex &lightVertex,
                                    Film *film) const
{
    VisibilityTester visib;
    Vector3f wi;
    Point2f pRaster;
    float camPdfA;
    Spectrum L;

    // Sample camera importance
    Spectrum Wi = camera->sample_Wi(lightVertex.getInteraction(), sampler.get2D(), &wi, &camPdfA, &pRaster,
                                    &visib);
    if (camPdfA == 0.0f || Wi.isBlack()) return;
    Spectrum beta = Wi / camPdfA;
    L = beta * lightVertex.compute_f(wi, TransportMode::Importance) * lightVertex.beta;
    if (L.isBlack()) return;
    if (!visib.unoccluded(scene)) return;
    L *= visib.compute_Tr(scene, sampler);
    if (lightVertex.isOnSurface())
        L *= absDot(wi, lightVertex.ns());

    // Compute MIS weight
    float bsdfRevPdfW = lightVertex.pdfW(wi, lightVertex.wo());
    float wLight = (camPdfA / nLightSubpaths) * (wVM + lightVertex.dVCM + lightVertex.dVC * bsdfRevPdfW);
    float weight = lightTraceOnly ? 1.0f : (1.0f / (wLight + 1.0f));
    Spectrum contrib = weight * L * camPdfA / nLightSubpaths;

    // Add splat to film
    film->addSplat(pRaster, contrib);
}

bool VCMIntegrator::sampleScattering(Sampler &sampler, const Vertex &vertex, SubpathState &state,
                                     TransportMode mode) const
{
    RayDifferential &ray = state.ray;
    Spectrum &beta = state.beta;
    float pdfFwdW = 0, pdfRevW = 0;
    Vector3f wi;

    if (vertex.type == VertexType::Medium) {
        // Sample phase function
        auto mi = vertex.mi;
        pdfFwdW = pdfRevW = vertex.mi.phase->sample_p(vertex.wo(), &wi, sampler.get2D());
        if (pdfFwdW == 0) return false;
        ray = mi.spawnRay(wi);

        // Compute MIS quantities
        state.dVC = (state.dVC * pdfRevW + state.dVCM + wVM) / pdfFwdW;
        state.dVM = (state.dVM * pdfRevW + state.dVCM * wVC + 1.0f) / pdfFwdW;
        state.dVCM = 1.0f / pdfFwdW;
        state.isSpecularPath &= 0;
    } else {
        // Sample BSDF
        auto &isect = vertex.si;
        auto wo = isect.wo;
        Spectrum f = isect.bsdf->sample_f(wo, &wi, sampler.get2D(), &pdfFwdW, BSDF_ALL, &state.flags);
        if (f.isBlack() || pdfFwdW == 0.0f) return false;

        // Compute MIS quantities
        float cosTheta = absDot(wi, isect.shading.n);
        if (state.isDelta()) {
            pdfRevW = pdfFwdW;
            state.dVCM = 0;
            state.dVC *= cosTheta;
            state.dVM *= cosTheta;
            state.isSpecularPath &= 1;
        } else {
            pdfRevW = vertex.pdfW(wi, wo);
            state.dVC = (cosTheta / pdfFwdW) * (state.dVC * pdfRevW + state.dVCM + wVM);
            state.dVM = (cosTheta / pdfFwdW) * (state.dVM * pdfRevW + state.dVCM * wVC + 1.0f);
            state.dVCM = 1.0f / pdfFwdW;
            state.isSpecularPath &= 0;
        }

        // Update path throughput and generate new ray
        beta *= (f * cosTheta / pdfFwdW);
        beta *= BDPTIntegrator::correctShadingNormal(isect, wo, wi, mode);
        ray = isect.spawnRay(wi);
    }

    return true;
}

VCMIntegrator::SubpathState VCMIntegrator::generateCameraSample(Sampler &sampler, const Point2f &pFilm) const
{
    // Sample initial ray for camera subpath
    CameraSample camSample;
    camSample.pFilm = pFilm;
    camSample.pLens = sampler.get2D();
    camSample.time = sampler.get1D();
    RayDifferential ray;
    Spectrum beta = camera->generateRayDifferential(camSample, &ray);
    ray.scaleDifferentials(1.0f / sqrtf(sampler.samplesPerPixel));
    float pdfPos, pdfDirW;
    camera->pdf_We(ray, &pdfPos, &pdfDirW);

    // Initialize camera subpath state
    SubpathState camState;
    camState.ray = ray;
    camState.beta = beta;
    camState.pathLength = 1;
    camState.isSpecularPath = 1;
    camState.dVCM = nLightSubpaths / pdfDirW;
    camState.dVC = camState.dVM = 0;

    return camState;
}

Spectrum VCMIntegrator::getLightRadiance(const Light *light, SubpathState &camState, SurfaceInteraction *isect) const
{
    Spectrum Le;
    float pdfChoice = lightDistrib->discretePDF(lightToIndex.find(light)->second);
    float pdfPos, pdfDir;
    Ray lightRay(camState.ray.o, -camState.ray.d, INFINITY, camState.ray.time);

    // Compute radiance carried by ray
    if (light->flags & int(LightFlags::Infinite)) {  // infinite area light
        Le = light->compute_Le(camState.ray);
        light->pdf_Le(lightRay, Normal3f(), &pdfPos, &pdfDir);
    }
    else { // area light
        Le = isect->compute_Le(-camState.ray.d);
        light->pdf_Le(lightRay, isect->n, &pdfPos, &pdfDir);
    }

    // Handle special cases
    if (Le.isBlack()) return 0;
    if (camState.pathLength == 1) return Le; // light source directly seen from camera
    if (useVM && !useVC) // using only VM
        return camState.isSpecularPath ? Le : 0; // purely specular paths give radiance

    // Compute tracing and connection pdfs
    float pdfTrace, pdfConnect;
    lightPDFs(light, pdfChoice, pdfPos, pdfDir, &pdfTrace, &pdfConnect);

    // Compute MIS quantities
    float wCam = pdfConnect * camState.dVCM + pdfTrace * camState.dVC;
    float weight = 1.0f / (1.0f + wCam);

    Le *= (weight * camState.beta);
    return Le;
}

Spectrum VCMIntegrator::directIllumination(const Scene &scene, Sampler &sampler, const Vertex &camVertex) const
{
    // Sample lights for radiance
    float pdfChoice;
    int lightIndex = lightDistrib->sampleDiscrete(sampler.get1D(), &pdfChoice);
    const auto &light = scene.lights[lightIndex];
    Vector3f wi;
    float pdfConnectW, pdfPos, pdfDir;
    VisibilityTester visib;
    Spectrum Li = light->sample_Li(camVertex.getInteraction(), sampler.get2D(), &wi, &pdfConnectW, &visib);
    if (!visib.unoccluded(scene)) return 0;
    if (Li.isBlack()) return 0;

    // Compute light and surface pdfs
    Interaction pShape = visib.getP1();
    light->pdf_Le(Ray(pShape.p, -wi, INFINITY, pShape.time), pShape.n, &pdfPos, &pdfDir);
    float pdfFwdW = camVertex.pdfW(wi);
    float pdfRevW = camVertex.pdfW(wi, camVertex.wo());
    if (pdfFwdW == 0 || pdfRevW == 0) return 0;

    // Compute MIS quantities
    float pdfTraceW = pdfPos * pdfDir;
    float cosToLight = dot(camVertex.ns(), wi), cosAtLight = dot(pShape.n, -wi);
    float wLight = pdfFwdW / (pdfChoice * pdfConnectW);
    float wCam = (pdfTraceW * cosToLight) / (pdfConnectW * cosAtLight) *
                 (wVM + camVertex.dVCM + camVertex.dVC * pdfRevW);
    float weight = 1.0f / (wLight + 1.0f + wCam);

    // Return contribution
    Spectrum Tr = visib.compute_Tr(scene, sampler);
    Spectrum contrib = weight * camVertex.beta * Li * Tr * cosToLight / (pdfChoice * pdfConnectW);
    return contrib;
}

Spectrum VCMIntegrator::connectVertices(const Scene &scene, Sampler &sampler, const Vertex &lightVertex,
                                        const Vertex &camVertex) const
{
    // Get the connection
    Vector3f dir = lightVertex.p() - camVertex.p();
    float distSq = dir.lengthSq();
    float dist = sqrt(distSq);
    dir /= dist;

    // Evaluate BSDF at camera and light vertex
    Spectrum f_cam = camVertex.compute_f(dir, TransportMode::Radiance);
    if (f_cam.isBlack()) return 0;
    float cosCam = absDot(camVertex.ns(), dir);
    float pdfCamFwdW = camVertex.pdfW(camVertex.wo(), dir);
    float pdfCamRevW = camVertex.pdfW(dir, camVertex.wo());

    Spectrum f_light = lightVertex.compute_f(-dir, TransportMode::Importance);
    if (f_light.isBlack()) return 0;
    float cosLight = absDot(lightVertex.ns(), -dir);
    float pdfLightFwdW = lightVertex.pdfW(lightVertex.wo(), -dir);
    float pdfLightRevW = lightVertex.pdfW(-dir, lightVertex.wo());

    // Compute geometry and throughput term
    float G = cosLight * cosCam / distSq;
    if (G <= 0) return 0;
    VisibilityTester visib(lightVertex.getInteraction(), camVertex.getInteraction());
    if (!visib.unoccluded(scene)) return 0;
    Spectrum Tr = visib.compute_Tr(scene, sampler);

    // Compute MIS quantities
    float pdfCamFwdA = pdfCamFwdW * distSq / cosLight;
    float pdfLightFwdA = pdfLightFwdW * distSq / cosCam;
    float wLight = pdfCamFwdA * (wVM + lightVertex.dVCM + lightVertex.dVC * pdfLightRevW);
    float wCam = pdfLightFwdA * (wVM + camVertex.dVCM + camVertex.dVC * pdfCamRevW);
    float weight = 1.0f / (wLight + 1.0f + wCam);

    // Return contribution
    Spectrum contrib = weight * lightVertex.beta * f_light * G * Tr * f_cam * camVertex.beta;
    return contrib;
}

void VCMIntegrator::render(const Scene &scene) {
    ProfilePhase __(Stage::IntegratorRender);

    // Initialize pixel bounds
    Film *film = camera->film;
    const Bounds2i sampleBounds = film->getSampleBounds();
    const Vector2i sampleExtent = sampleBounds.diagonal();
    nLightSubpaths = nPixels = sampleBounds.area();
    constexpr int tileSize = 16;
    const int nXTiles = (sampleExtent.x + tileSize - 1) / tileSize;
    const int nYTiles = (sampleExtent.y + tileSize - 1) / tileSize;

    // Initialize sampler
    SobolSampler globalSampler(nIterations, sampleBounds);

    // Initialize light distribution
    if (scene.lights.empty())
        WARNING("No lights in the scene. Rendering black image...");
    lightDistrib = computeLightPowerDistribution(scene.lights);
    for (size_t i = 0; i < scene.lights.size(); i++)
        lightToIndex[scene.lights[i].get()] = i;

    // Main render loop
    ProgressReporter reporter(nIterations * 2, "Rendering ");
    for (int ite = 0; ite < nIterations; ite++) {
        // Compute VCM quantities
        float radius = initialRadius * pow(float(ite + 1), 0.5f * (alpha - 1));
        float etaVCM = nLightSubpaths * PI * SQ(radius);
        wVM = useVM ? etaVCM : 0;
        wVC = useVC ? (1.0f / etaVCM) : 0;

        // Prepare memory arenas for each thread
        vector<MemoryArena> lightArenas(Parallel::maxThreadIndex());

        // Sample light path
        unique_ptr<Vertex *[]> lightPaths(new Vertex *[nLightSubpaths]);
        vector<int> lightPathLengths(nLightSubpaths);
        constexpr int chunkSize = 1024;
        int nChunk = nLightSubpaths / chunkSize + 1;

        Parallel::forLoop([&] (int chunkIndex) {
            auto &arena = lightArenas[Parallel::getThreadIndex()];
            auto sampler = globalSampler.clone(chunkIndex);
            for (int i = 0; i < chunkSize; i++) {
                // Preparation
                int pathIndex = chunkIndex * chunkSize + i;
                if (pathIndex >= nPixels) return;
                sampler->startPixel(Point2i(chunkIndex, i));
                sampler->setSampleNumber(ite);
                lightPaths[pathIndex] = arena.alloc<Vertex>(maxDepth);

                // Generate light sample and start random walk
                SubpathState lightState = generateLightSample(scene, *sampler);
                lightPathLengths[pathIndex] = lightRandomWalk(scene, *sampler, arena, lightState,
                                                              lightPaths[pathIndex], film);
            }
        }, nChunk);

        // Build range search struct
        LightVerticesGrid grid(lightPaths.get(), lightPathLengths, radius, scene.getWorldBound(), lightArenas);
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
                if (lightTraceOnly) break;
                Point2f pFilm = Point2f(pPixel) + tileSampler->get2D();
                SubpathState camState = generateCameraSample(*tileSampler, pFilm);

                // Start camera random walk and accumulate radiance
                Spectrum L;
                RayDifferential &ray = camState.ray;
                Spectrum &beta = camState.beta;
                int &length = camState.pathLength;

                while (true) {
                    // Trace a ray and sample the medium, if any
                    MediumInteraction mi;
                    SurfaceInteraction isect;
                    bool foundIsect = scene.intersect(ray, &isect);
                    if (ray.medium)
                        beta *= ray.medium->sample(ray, *tileSampler, arena, &mi);
                    if (beta.isBlack()) break;
                    bool isMediumInteraction = mi.isValid();

                    if (!isMediumInteraction) {
                        if (!foundIsect) {
                            // Get radiance from enviroment for rays without medium or surface interaction
                            for (const auto &light : scene.lights)
                                if (light->flags & int(LightFlags::Infinite))
                                    L += getLightRadiance(light.get(), camState);
                            break;
                        } else {
                            // Compute scattering functions for camera subpath and skip over medium boundaries
                            isect.computeScatteringFunctions(ray, arena, true, TransportMode::Radiance);
                            if (!isect.bsdf) {
                                ray = isect.spawnRay(ray.d);
                                continue;
                            }
                        }
                    }

                    // Complete MIS quantities evaluation
                    if (camState.pathLength > 1 || !camState.isInfiniteLight) {
                        float distSq = SQ(ray.tMax);
                        camState.dVCM *= distSq;
                        if (!isMediumInteraction) { // cosine fix only applies to surface interaction
                            float cosTheta = dot(isect.wo, isect.shading.n);
                            camState.dVCM /= cosTheta;
                            camState.dVC /= cosTheta;
                            camState.dVM /= cosTheta;
                        }
                    }

                    // Handle intersection with area light source
                    if (!isMediumInteraction) {
                        auto area = isect.primitive->getAreaLight();
                        if (area) {
                            L += getLightRadiance(area, camState, &isect);
                            break;
                        }
                    }

                    if (length >= maxDepth) break;

                    // Vertex connection
                    Vertex camVertex = isMediumInteraction ? Vertex(camState, mi) : Vertex(camState, isect);
                    if (useVC && !camState.isDelta()) {
                        // Connect to a light source
                        L += directIllumination(scene, *tileSampler, camVertex);

                        // Connect to light vertices
                        for (int s = 0; s < lightPathLengths[pixelOffset]; s++) {
                            auto &lightVertex = lightPaths[pixelOffset][s];
                            if (lightVertex.pathLength + camVertex.pathLength >= maxDepth) break;
                            L += connectVertices(scene, *tileSampler, lightVertex, camVertex);
                        }
                    }

                    // Vertex merging
                    if (useVM && (isMediumInteraction || !camState.isDelta())) {
                        RangeQuery query(*this, camVertex, radius);
                        grid.process(query);
                        L += query.getContribution();
                        if (usePPM) break;
                    }

                    // Sample next event
                    if (!sampleScattering(*tileSampler, camVertex, camState, TransportMode::Radiance)) break;

                    length++;
                }

                filmTile->addSample(pFilm, L);
            } // end each pixel in tile bounds

            film->mergeFilmTile(move(filmTile));
        }, Point2i(nXTiles, nYTiles));

        reporter.update();
    } // end iteration

    reporter.done();
    film->writeImage(1.0f / nIterations);
}

VCMIntegrator * VCMIntegrator::create(const ParamSet &params, shared_ptr<Sampler>,
                                     shared_ptr<const Camera> camera)
{
    int maxDepth = params.findOneInt("maxdepth", 7);
    int nIterations = params.findOneInt("iterations", params.findOneInt("numiterations", 10));
    float radius = params.findOneFloat("radius", 0.01f);
    float alpha = params.findOneFloat("alpha", 0.75f);
    return new VCMIntegrator(camera, AlgorithmType::VCM, nIterations, maxDepth, radius, alpha);
}
