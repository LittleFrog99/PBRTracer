#include "mlt.h"
#include "paramset.h"
#include "core/renderer.h"
#include "stats.h"

STAT_PERCENT("Integrator/Acceptance rate", acceptedMutations, totalMutations);

using Vertex = BDPTIntegrator::Vertex;

Spectrum MLTIntegrator::compute_L(const Scene &scene, MemoryArena &arena,
                                  const unique_ptr<Distribution1D> &lightDistrib, const LightIndexMap &lightToIndex,
                                  PSSSampler &sampler, int depth, Point2f *pRaster)
{
    sampler.startStream(CAMERA_STREAM_INDEX);

    // Determine the number of available strategies and pick a specific one
    int s, t, nStrategies;
    if (depth == 0) {
        nStrategies = 1;
        s = 0;
        t = 2;
    } else {
        nStrategies = depth + 2;
        s = min(int(sampler.get1D() * nStrategies), nStrategies - 1); // at least one camera vertice
        t = nStrategies - s;
    }

    // Generate a camera subpath with exactly _t_ vertices
    auto cameraVertices = arena.alloc<Vertex>(t);
    Bounds2f sampleBounds(camera->film->getSampleBounds());
    *pRaster = sampleBounds.lerp(sampler.get2D());
    if (BDPTIntegrator::generateCameraSubpath(scene, sampler, arena, t, *camera, *pRaster, cameraVertices) != t)
        return 0;

    // Generate a light subpath with exactly _s_ vertices
    sampler.startStream(LIGHT_STREAM_INDEX);
    auto lightVertices = arena.alloc<Vertex>(s);
    if (BDPTIntegrator::generateLightSubpath(scene, sampler, arena, s, cameraVertices[0].time(), *lightDistrib,
                                             lightToIndex, lightVertices) != s)
        return 0;

    // Execute connection strategies and return the radiance estimate
    sampler.startStream(CONNECTION_STREAM_INDEX);
    float weight = 0;
    Spectrum Lpath = BDPTIntegrator::connectVertices(scene, lightVertices, cameraVertices, s, t, *lightDistrib,
                                                     lightToIndex, *camera, sampler, pRaster, &weight);
    return weight * Lpath * nStrategies;
}

void MLTIntegrator::render(const Scene &scene) {
    // Compute light distribution and index map
    auto lightDistrib = computeLightPowerDistribution(scene.lights);
    LightIndexMap lightToIndex;
    for (size_t i = 0; i < scene.lights.size(); i++)
        lightToIndex[scene.lights[i].get()] = i;

    // Generate bootstrap samples and compute normalization constant b
    const int nBootstrapSamples = nBootstrap * (maxDepth + 1);
    vector<float> bootstrapWeights(nBootstrapSamples, 0);
    vector<MemoryArena> bootstrapArenas(Parallel::maxThreadIndex());
    {
        ProgressReporter progress(nBootstrap / 256, "Generating bootstrap paths");
        Parallel::forLoop([&] (int i) {
            MemoryArena &arena = bootstrapArenas[Parallel::getThreadIndex()];
            for (uint depth = 0; depth <= maxDepth; depth++) {
                int rngIndex = i * (maxDepth + 1) + depth;
                PSSSampler sampler(mutPerPixel, rngIndex, sigma, largeStepProb, NUM_SAMPLE_STREAMS);
                Point2f pRaster;
                bootstrapWeights[rngIndex] = compute_L(scene, arena, lightDistrib, lightToIndex, sampler, depth,
                                                       &pRaster).luminance();
                arena.reset();
            }
            if ((i + 1) % 256 == 0) progress.update();
        }, nBootstrap, 4096);
        progress.done();
    }

    const Distribution1D bootstrap(&bootstrapWeights[0], nBootstrapSamples);
    const float b = bootstrap.funcInt * (maxDepth + 1);

    // Run _nChains_ Markov chain in parallel
    auto &film = *camera->film;
    const uint64_t nTotalMut = uint64_t(mutPerPixel) * uint64_t(film.getSampleBounds().area());
    const uint64_t nMutPerChain = nTotalMut / nChains;
    constexpr int progressFrequency = 32768;
    ProgressReporter progress(nTotalMut / progressFrequency, "Rendering");

    Parallel::forLoop([&] (uint i) {
        uint64_t nChainMut = min((i + 1) * nMutPerChain, nTotalMut) - i * nMutPerChain;
        MemoryArena arena;

        // Select initial state from the set of bootstrap samples
        Random rng(i);
        uint bootstrapIndex = bootstrap.sampleDiscrete(rng.uniformFloat());
        uint depth = bootstrapIndex % (maxDepth + 1); // deduce path depth from sampled index

        // Initialize local variables for selected state
        PSSSampler sampler(mutPerPixel, bootstrapIndex, sigma, largeStepProb, NUM_SAMPLE_STREAMS);
        Point2f pCurrent;
        Spectrum LCurrent = compute_L(scene, arena, lightDistrib, lightToIndex, sampler, depth, &pCurrent);

        // Run on the Markov chain for _nChainMut_ steps
        for (uint64_t j = 0; j < nChainMut; j++) {
            sampler.startIteration();
            Point2f pProposed;
            Spectrum LProposed = compute_L(scene, arena, lightDistrib, lightToIndex, sampler, depth, &pProposed);

            // Compute acceptance probability for proposed sample
            if (LCurrent.luminance() == 0) {
                totalMutations++;
                continue;
            }
            float accept = min(1.0f, LProposed.luminance() / LCurrent.luminance());

            // Splat both current and proposed samples to film
            if (accept > 0)
                film.addSplat(pProposed, LProposed * accept / LProposed.luminance());
            film.addSplat(pCurrent, LCurrent * (1 - accept) / LCurrent.luminance());

            // Accept or reject the proposal
            if (rng.uniformFloat() < accept) {
                pCurrent = pProposed;
                LCurrent = LProposed;
                sampler.accept();
                acceptedMutations++;
            } else
                sampler.reject();
            totalMutations++;

            if ((i * nMutPerChain + j) % progressFrequency == 0) progress.update();
        }

    }, nChains);

    progress.done();

    // Store final image
    camera->film->writeImage(b / mutPerPixel);
}

MLTIntegrator * MLTIntegrator::create(const ParamSet &params, shared_ptr<Sampler>, shared_ptr<const Camera> camera) {
    int maxDepth = params.findOneInt("maxdepth", 5);
    int nBootstrap = params.findOneInt("bootstrapsamples", 100000);
    int nChains = params.findOneInt("chains", 1000);
    int mutPerPixel = params.findOneInt("mutationsperpixel", 100);
    float largeStepProb = params.findOneFloat("largestepprobability", 0.3f);
    float sigma = params.findOneFloat("sigma", .01f);
    if (Renderer::options.quickRender) {
        mutPerPixel = max(1, mutPerPixel / 16);
        nBootstrap = max(1, nBootstrap / 16);
    }
    return new MLTIntegrator(camera, maxDepth, nBootstrap, nChains, mutPerPixel, sigma, largeStepProb);
}
