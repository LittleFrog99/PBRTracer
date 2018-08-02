#include "stratified.h"
#include "core/renderer.h"
#include "stats.h"

using namespace Sampling;

void StratifiedSampler::startPixel(const Point2i &p) {
    ProfilePhase _(Stage::StartPixel);
    // Generate single stratified samples for the pixel
    for (size_t i = 0; i < samples1D.size(); ++i) {
        sample1D(&samples1D[i][0], xPixelSamples * yPixelSamples, rng, jitterSamples);
        shuffle(&samples1D[i][0], xPixelSamples * yPixelSamples, 1, rng);
    }
    for (size_t i = 0; i < samples2D.size(); ++i) {
        sample2D(&samples2D[i][0], xPixelSamples, yPixelSamples, rng, jitterSamples);
        shuffle(&samples2D[i][0], xPixelSamples * yPixelSamples, 1, rng);
    }

    // Generate arrays of stratified samples for the pixel
    for (size_t i = 0; i < samples1DArraySizes.size(); ++i)
        for (int64_t j = 0; j < samplesPerPixel; ++j) {
            int count = samples1DArraySizes[i];
            sample1D(&sampleArray1D[i][j * count], count, rng, jitterSamples);
            shuffle(&sampleArray1D[i][j * count], count, 1, rng);
        }
    for (size_t i = 0; i < samples2DArraySizes.size(); ++i)
        for (int64_t j = 0; j < samplesPerPixel; ++j) {
            int count = samples2DArraySizes[i];
            latinHypercube(&sampleArray2D[i][j * count].x, count, 2, rng);
        }

    PixelSampler::startPixel(p);
}

unique_ptr<Sampler> StratifiedSampler::clone(int seed) {
    auto ss = new StratifiedSampler(*this);
    ss->rng.setSequence(seed);
    return unique_ptr<Sampler>(ss);
}

void StratifiedSampler::sample1D(float *samples, int nSamples, Random &rng, bool jitter) {
    float invNSamples = 1.0f / nSamples;
    for (int i = 0; i < nSamples; ++i) {
        float delta = jitter ? rng.uniformFloat() : 0.5f;
        samples[i] = min((i + delta) * invNSamples, Random::ONE_MINUS_EPSILON);
    }
}

void StratifiedSampler::sample2D(Point2f *samples, int nx, int ny, Random &rng, bool jitter) {
    float dx = 1.0f / nx, dy = 1.0f / ny;
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
            float jx = jitter ? rng.uniformFloat() : 0.5f;
            float jy = jitter ? rng.uniformFloat() : 0.5f;
            samples->x = min((x + jx) * dx, Random::ONE_MINUS_EPSILON);
            samples->y = min((y + jy) * dy, Random::ONE_MINUS_EPSILON);
            ++samples;
        }
}

void StratifiedSampler::latinHypercube(float *samples, int nSamples, int nDim, Random &rng) {
    // Generate LHS samples along diagonal
    float invNSamples = 1.0f / nSamples;
    for (int i = 0; i < nSamples; ++i)
        for (int j = 0; j < nDim; ++j) {
            float sj = (i + (rng.uniformFloat())) * invNSamples;
            samples[nDim * i + j] = min(sj, Random::ONE_MINUS_EPSILON);
        }

    // Permute LHS samples in each dimension
    for (int i = 0; i < nDim; ++i) {
        for (int j = 0; j < nSamples; ++j) {
            int other = j + rng.uniformUInt32(nSamples - j);
            swap(samples[nDim * j + i], samples[nDim * other + i]);
        }
    }
}

StratifiedSampler * StratifiedSampler::create(const ParamSet &params) {
    bool jitter = params.findOneBool("jitter", true);
    int xsamp = params.findOneInt("xsamples", 4);
    int ysamp = params.findOneInt("ysamples", 4);
    int sd = params.findOneInt("dimensions", 4);
    if (Renderer::options.quickRender) xsamp = ysamp = 1;
    return new StratifiedSampler(xsamp, ysamp, jitter, sd);
}
