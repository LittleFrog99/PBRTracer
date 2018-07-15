#ifndef CORE_SAMPLER
#define CORE_SAMPLER

#include "vector.h"
#include "random.h"

class CameraSample;

class Sampler {
public:
    Sampler(int64_t samplesPerPixel) : samplesPerPixel(samplesPerPixel) {}
    virtual ~Sampler() {}

    virtual Float get1D() = 0;
    virtual Point2f get2D() = 0;
    virtual unique_ptr<Sampler> clone(int seed) = 0;

    virtual void startPixel(const Point2i &p);
    virtual bool startNextSample();
    virtual bool setSampleNumber(int64_t sampleNum);
    virtual int roundCount(int n) const { return n; }

    CameraSample getCameraSample(const Point2i &pRaster);

    void request1DArray(int n) {
        samples1DArraySizes.push_back(n);
        sampleArray1D.push_back(vector<Float>(n * samplesPerPixel));
    }

    void request2DArray(int n) {
        samples2DArraySizes.push_back(n);
        sampleArray2D.push_back(vector<Point2f>(n * samplesPerPixel));
    }

    const Float * get1DArray(int n) {
        if (array1DOffset == sampleArray1D.size()) return nullptr;
        return &sampleArray1D[array1DOffset++][curPixelSampleIndex * n];
    }

    const Point2f * get2DArray(int n) {
        if (array2DOffset == sampleArray2D.size()) return nullptr;
        return &sampleArray2D[array2DOffset++][curPixelSampleIndex * n];
    }

    string stateString() const {
      return STRING_PRINTF("(%d,%d), sample %" PRId64, curPixel.x, curPixel.y,
                           curPixelSampleIndex);
    }

    int64_t currentSampleNumber() const { return curPixelSampleIndex; }

    const int64_t samplesPerPixel;

protected:
    Point2i curPixel;
    int64_t curPixelSampleIndex;
    vector<int> samples1DArraySizes, samples2DArraySizes;
    vector<vector<Float>> sampleArray1D;
    vector<vector<Point2f>> sampleArray2D;

private:
    size_t array1DOffset, array2DOffset;
};

class PixelSampler : public Sampler {
public:
    PixelSampler(int64_t samplesPerPixel, int nSampledDimensions);
    bool startNextSample();
    bool setSampleNumber(int64_t);
    Float get1D();
    Point2f get2D();

protected:
    vector<vector<Float>> samples1D;
    vector<vector<Point2f>> samples2D;
    int cur1DDim = 0, cur2DDim = 0;
    Random rng;
};

class GlobalSampler : public Sampler {
public:
    GlobalSampler(int64_t samplesPerPixel) : Sampler(samplesPerPixel) {}
    bool startNextSample();
    void startPixel(const Point2i &p);
    bool setSampleNumber(int64_t sampleNum);
    Float get1D();
    Point2f get2D();

    virtual int64_t getIndexForSample(int64_t sampleNum) const = 0;
    virtual Float sampleDimension(int64_t index, int dimension) const = 0;

private:
    int dimension;
    int64_t intervalSampleIndex;
    static const int arrayStartDim = 5;
    int arrayEndDim;
};

namespace Sampling {

Point2f rejectionSampleDisk(Random &rng);
Vector3f uniformSampleHemisphere(const Point2f &u);
Float uniformHemispherePdf();
Vector3f uniformSampleSphere(const Point2f &u);
Float uniformSpherePdf();
Vector3f uniformSampleCone(const Point2f &u, Float thetamax);
Vector3f uniformSampleCone(const Point2f &u, Float thetamax, const Vector3f &x, const Vector3f &y,
                           const Vector3f &z);
Float uniformConePdf(Float thetamax);
Point2f uniformSampleDisk(const Point2f &u);
Point2f concentricSampleDisk(const Point2f &u);
Point2f uniformSampleTriangle(const Point2f &u);

template <typename T>
inline void shuffle(T *samp, int count, int nDimensions, Random &rng) {
    for (int i = 0; i < count; ++i) {
        unsigned other = i + rng.uniformUInt32(count - i);
        for (int j = 0; j < nDimensions; ++j)
            swap(samp[nDimensions * i + j], samp[nDimensions * other + j]); // move entire blocks
    }
}

inline Vector3f cosineSampleHemisphere(const Point2f &u) {
    Point2f d = concentricSampleDisk(u);
    Float z = sqrt(max((Float)0, 1 - d.x * d.x - d.y * d.y));
    return Vector3f(d.x, d.y, z);
}

inline Float cosineHemispherePdf(Float cosTheta) { return cosTheta * INV_PI; }

inline Float balanceHeuristic(int nf, Float fPdf, int ng, Float gPdf) {
    return (nf * fPdf) / (nf * fPdf + ng * gPdf);
}

inline Float powerHeuristic(int nf, Float fPdf, int ng, Float gPdf) {
    Float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
}

};

struct Distribution1D {
    Distribution1D(const Float *f, int n) : func(f, f + n), cdf(n + 1) {
        // Compute integral of step function at $x_i$
        cdf[0] = 0;
        for (int i = 1; i < n + 1; ++i) cdf[i] = cdf[i - 1] + func[i - 1] / n;
        // Transform step function integral into CDF
        funcInt = cdf[n];
        if (funcInt == 0) {
            for (int i = 1; i < n + 1; ++i) cdf[i] = Float(i) / Float(n);
        } else {
            for (int i = 1; i < n + 1; ++i) cdf[i] /= funcInt;
        }
    }

    int count() const { return int(func.size()); }

    Float sampleContinuous(Float u, Float *pdf, int *off = nullptr) const {
        // Find surrounding CDF segments and _offset_
        int offset = findInterval(int(cdf.size()), [&](int index) { return cdf[index] <= u; });
        if (off) *off = offset;
        // Compute offset along CDF segment
        Float du = u - cdf[offset];
        if ((cdf[offset + 1] - cdf[offset]) > 0) {
            CHECK_GT(cdf[offset + 1], cdf[offset]);
            du /= (cdf[offset + 1] - cdf[offset]);
        }
        DCHECK(!isnan(du));

        // Compute PDF for sampled offset
        if (pdf) *pdf = (funcInt > 0) ? func[offset] / funcInt : 0;
        // Return $x\in{}[0,1)$ corresponding to sample
        return (offset + du) / count();
    }

    int sampleDiscrete(Float u, Float *pdf = nullptr, Float *uRemapped = nullptr) const {
        // Find surrounding CDF segments and _offset_
        int offset = findInterval((int)cdf.size(),
                                  [&](int index) { return cdf[index] <= u; });
        if (pdf) *pdf = (funcInt > 0) ? func[offset] / (funcInt * count()) : 0;
        if (uRemapped)
            *uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
        if (uRemapped) CHECK(*uRemapped >= 0.f && *uRemapped <= 1.f);
        return offset;
    }

    Float discretePDF(int index) const {
        CHECK(index >= 0 && index < count());
        return func[index] / (funcInt * count());
    }

    vector<Float> func, cdf;
    Float funcInt;
};

class Distribution2D {
public:
    Distribution2D(const Float *data, int nu, int nv);

    Point2f sampleContinuous(const Point2f &u, Float *pdf) const {
        Float pdfs[2];
        int v;
        Float d1 = pMarginal->sampleContinuous(u[1], &pdfs[1], &v);
        Float d0 = pConditionalV[v]->sampleContinuous(u[0], &pdfs[0]);
        *pdf = pdfs[0] * pdfs[1];
        return Point2f(d0, d1);
    }

    Float pdf(const Point2f &p) const {
        int iu = clamp(int(p[0] * pConditionalV[0]->count()), 0, pConditionalV[0]->count() - 1);
        int iv = clamp(int(p[1] * pMarginal->count()), 0, pMarginal->count() - 1);
        return pConditionalV[iv]->func[iu] / pMarginal->funcInt;
    }

private:
    vector<unique_ptr<Distribution1D>> pConditionalV;
    unique_ptr<Distribution1D> pMarginal;
};


#endif // CORE_SAMPLER
