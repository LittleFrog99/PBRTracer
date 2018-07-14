#include "sampling.h"
#include "core/camera.h"

using namespace Sampling;

CameraSample Sampler::getCameraSample(const Point2i &pRaster) {
    CameraSample cs;
    cs.pFilm = Point2f(pRaster) + get2D();
    cs.time = get1D();
    cs.pLens = get2D();
    return cs;
}

void Sampler::startPixel(const Point2i &p) {
    curPixel = p;
    curPixelSampleIndex = 0;
    array1DOffset = array2DOffset = 0;
}

bool Sampler::startNextSample() {
    array1DOffset = array2DOffset = 0;
    return ++curPixelSampleIndex < samplesPerPixel;
}

bool Sampler::setSampleNumber(int64_t sampleNum) {
    array1DOffset = array2DOffset = 0;
    curPixelSampleIndex = sampleNum;
    return curPixelSampleIndex < samplesPerPixel;
}

PixelSampler::PixelSampler(int64_t samplesPerPixel, int nSampledDimensions)
    : Sampler(samplesPerPixel) {
    for (int i = 0; i < nSampledDimensions; i++) {
        samples1D.push_back(vector<Float>(samplesPerPixel));
        samples2D.push_back(vector<Point2f>(samplesPerPixel));
    }
}

bool PixelSampler::startNextSample() {
    cur1DDim = cur2DDim = 0;
    return Sampler::startNextSample();
}

bool PixelSampler::setSampleNumber(int64_t sampleNum) {
    cur1DDim = cur2DDim = 0;
    return Sampler::setSampleNumber(sampleNum);
}

Float PixelSampler::get1D() {
    if (cur1DDim < samples1D.size())
        return samples1D[cur1DDim][curPixelSampleIndex++];
    else
        return rng.uniformFloat();
}

Point2f PixelSampler::get2D() {
    if (cur2DDim < samples2D.size())
        return samples2D[cur2DDim][curPixelSampleIndex++];
    else
        return Point2f(rng.uniformFloat(), rng.uniformFloat());
}

void GlobalSampler::startPixel(const Point2i &p) {
    Sampler::startPixel(p);
    dimension = 0;
    intervalSampleIndex = getIndexForSample(0);
    arrayEndDim = arrayStartDim + sampleArray1D.size() + 2 * sampleArray2D.size();

    for (size_t i = 0; i < samples1DArraySizes.size(); i++) {
        int nSamples = samples1DArraySizes[i] * samplesPerPixel;
        for (int j = 0; j < nSamples; j++) {
            int64_t index = getIndexForSample(j);
            sampleArray1D[i][j] = sampleDimension(index, arrayStartDim + i);
        }
    }

    int dim = arrayStartDim + samples1DArraySizes.size();
    for (size_t i = 0; i < samples2DArraySizes.size(); i++) {
        int nSamples = samples2DArraySizes[i] * samplesPerPixel;
        for (int j = 0; j < nSamples; j++) {
            int64_t index = getIndexForSample(j);
            sampleArray2D[i][j] = Point2f(sampleDimension(index, dim), sampleDimension(index, dim + 1));
            dim += 2;
        }
    }
}

bool GlobalSampler::startNextSample() {
    dimension = 0;
    intervalSampleIndex = getIndexForSample(curPixelSampleIndex + 1);
    return Sampler::startNextSample();
}

bool GlobalSampler::setSampleNumber(int64_t sampleNum) {
    dimension = 0;
    intervalSampleIndex = getIndexForSample(sampleNum);
    return Sampler::setSampleNumber(sampleNum);
}

Float GlobalSampler::get1D() {
    if (dimension >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
    return sampleDimension(intervalSampleIndex, dimension++);
}

Point2f GlobalSampler::get2D() {
    if (dimension >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
    Point2f p(sampleDimension(intervalSampleIndex, dimension),
              sampleDimension(intervalSampleIndex, dimension + 1));
    dimension += 2;
    return p;
}
