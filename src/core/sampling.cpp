#include "sampling.h"
#include "core/camera.h"
#include "stats.h"

using namespace Sampling;

Sampler::Sampler(int64_t samplesPerPixel) : samplesPerPixel(samplesPerPixel) {}

CameraSample Sampler::getCameraSample(const Point2i &pRaster) {
    CameraSample cs;
    cs.pFilm = (Point2f)pRaster + get2D();
    cs.time = get1D();
    cs.pLens = get2D();
    return cs;
}

void Sampler::startPixel(const Point2i &p) {
    currentPixel = p;
    currentPixelSampleIndex = 0;
    array1DOffset = array2DOffset = 0;
}

bool Sampler::startNextSample() {
    array1DOffset = array2DOffset = 0;
    return ++currentPixelSampleIndex < samplesPerPixel;
}

bool Sampler::setSampleNumber(int64_t sampleNum) {
    array1DOffset = array2DOffset = 0;
    currentPixelSampleIndex = sampleNum;
    return currentPixelSampleIndex < samplesPerPixel;
}

void Sampler::request1DArray(int n) {
    CHECK_EQ(roundCount(n), n);
    samples1DArraySizes.push_back(n);
    sampleArray1D.push_back(vector<float>(n * samplesPerPixel));
}

void Sampler::request2DArray(int n) {
    CHECK_EQ(roundCount(n), n);
    samples2DArraySizes.push_back(n);
    sampleArray2D.push_back(vector<Point2f>(n * samplesPerPixel));
}

const float *Sampler::get1DArray(int n) {
    if (array1DOffset == sampleArray1D.size()) return nullptr;
    CHECK_EQ(samples1DArraySizes[array1DOffset], n);
    CHECK_LT(currentPixelSampleIndex, samplesPerPixel);
    return &sampleArray1D[array1DOffset++][currentPixelSampleIndex * n];
}

const Point2f *Sampler::get2DArray(int n) {
    if (array2DOffset == sampleArray2D.size()) return nullptr;
    CHECK_EQ(samples2DArraySizes[array2DOffset], n);
    CHECK_LT(currentPixelSampleIndex, samplesPerPixel);
    return &sampleArray2D[array2DOffset++][currentPixelSampleIndex * n];
}

PixelSampler::PixelSampler(int64_t samplesPerPixel, int nSampledDimensions)
    : Sampler(samplesPerPixel) {
    for (int i = 0; i < nSampledDimensions; ++i) {
        samples1D.push_back(vector<float>(samplesPerPixel));
        samples2D.push_back(vector<Point2f>(samplesPerPixel));
    }
}

bool PixelSampler::startNextSample() {
    current1DDimension = current2DDimension = 0;
    return Sampler::startNextSample();
}

bool PixelSampler::setSampleNumber(int64_t sampleNum) {
    current1DDimension = current2DDimension = 0;
    return Sampler::setSampleNumber(sampleNum);
}

float PixelSampler::get1D() {
    ProfilePhase _(Stage::GetSample);
    CHECK_LT(currentPixelSampleIndex, samplesPerPixel);
    if (current1DDimension < samples1D.size())
        return samples1D[current1DDimension++][currentPixelSampleIndex];
    else
        return rng.uniformFloat();
}

Point2f PixelSampler::get2D() {
    ProfilePhase _(Stage::GetSample);
    CHECK_LT(currentPixelSampleIndex, samplesPerPixel);
    if (current2DDimension < samples2D.size())
        return samples2D[current2DDimension++][currentPixelSampleIndex];
    else
        return Point2f(rng.uniformFloat(), rng.uniformFloat());
}

void GlobalSampler::startPixel(const Point2i &p) {
    ProfilePhase _(Stage::StartPixel);
    Sampler::startPixel(p);
    dimension = 0;
    intervalSampleIndex = getIndexForSample(0);
    arrayEndDim = arrayStartDim + sampleArray1D.size() + 2 * sampleArray2D.size();

    // Compute 1D array samples for _GlobalSampler_
    for (size_t i = 0; i < samples1DArraySizes.size(); ++i) {
        int nSamples = samples1DArraySizes[i] * samplesPerPixel;
        for (int j = 0; j < nSamples; ++j) {
            int64_t index = getIndexForSample(j);
            sampleArray1D[i][j] = sampleDimension(index, arrayStartDim + i);
        }
    }

    // Compute 2D array samples for _GlobalSampler_
    int dim = arrayStartDim + samples1DArraySizes.size();
    for (size_t i = 0; i < samples2DArraySizes.size(); ++i) {
        int nSamples = samples2DArraySizes[i] * samplesPerPixel;
        for (int j = 0; j < nSamples; ++j) {
            int64_t idx = getIndexForSample(j);
            sampleArray2D[i][j].x = sampleDimension(idx, dim);
            sampleArray2D[i][j].y = sampleDimension(idx, dim + 1);
        }
        dim += 2;
    }
    CHECK_EQ(arrayEndDim, dim);
}

bool GlobalSampler::startNextSample() {
    dimension = 0;
    intervalSampleIndex = getIndexForSample(currentPixelSampleIndex + 1);
    return Sampler::startNextSample();
}

bool GlobalSampler::setSampleNumber(int64_t sampleNum) {
    dimension = 0;
    intervalSampleIndex = getIndexForSample(sampleNum);
    return Sampler::setSampleNumber(sampleNum);
}

float GlobalSampler::get1D() {
    ProfilePhase _(Stage::GetSample);
    if (dimension >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
    return sampleDimension(intervalSampleIndex, dimension++);
}

Point2f GlobalSampler::get2D() {
    ProfilePhase _(Stage::GetSample);
    if (dimension + 1 >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
    Point2f p(sampleDimension(intervalSampleIndex, dimension),
              sampleDimension(intervalSampleIndex, dimension + 1));
    dimension += 2;
    return p;
}
Distribution1D::Distribution1D(const float *f, int n)
    : func(f, f + n), cdf(n + 1)
{
    // Compute step function integral
    cdf[0] = 0;
    for (int i = 1; i < n + 1; ++i)
        cdf[i] = cdf[i - 1] + func[i - 1] / n;

    // Transform step function integral into CDF
    funcInt = cdf[n];
    if (funcInt == 0) // Handle degeneracy
        for (int i = 1; i < n + 1; ++i)
            cdf[i] = float(i) / float(n); // uniform distribution
    else
        for (int i = 1; i < n + 1; ++i)
            cdf[i] /= funcInt; // normalize CDF

}

float Distribution1D::sampleContinuous(float u, float *pdf, int *off) const {
    // Find surrounding CDF segments and _offset_
    int offset = findInterval(int(cdf.size()), [&](int index) { return cdf[index] <= u; });
    if (off) *off = offset;

    // Compute offset along CDF segment
    float du = u - cdf[offset];
    if ((cdf[offset + 1] - cdf[offset]) > 0) {
        CHECK_GT(cdf[offset + 1], cdf[offset]);
        du /= (cdf[offset + 1] - cdf[offset]);
    }
    DCHECK(!isnan(du));

    if (pdf) *pdf = (funcInt > 0) ? func[offset] / funcInt : 0;
    return (offset + du) / count();
}

int Distribution1D::sampleDiscrete(float u, float *pdf, float *uRemapped) const {
    // Find surrounding CDF segments and _offset_
    int offset = findInterval(int(cdf.size()), [&] (int index) { return cdf[index] <= u; });
    if (pdf) *pdf = (funcInt > 0) ? func[offset] / (funcInt * count()) : 0;
    if (uRemapped)
        *uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
    if (uRemapped) CHECK(*uRemapped >= 0.f && *uRemapped <= 1.f);
    return offset;
}

Distribution2D::Distribution2D(const float *func, int nu, int nv) {
    pConditionalV.reserve(nv);
    for (int v = 0; v < nv; ++v)
        pConditionalV.emplace_back(new Distribution1D(&func[v * nu], nu));

    vector<float> marginalFunc;
    marginalFunc.reserve(nv);
    for (int v = 0; v < nv; ++v)
        marginalFunc.push_back(pConditionalV[v]->funcInt);
    pMarginal.reset(new Distribution1D(&marginalFunc[0], nv));
}
