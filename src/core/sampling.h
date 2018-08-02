#ifndef CORE_SAMPLER
#define CORE_SAMPLER

#include "vector.h"
#include "random.h"

class CameraSample;

class Sampler {
public:
    Sampler(int64_t samplesPerPixel);
    virtual ~Sampler() {}

    virtual void startPixel(const Point2i &p);
    virtual bool startNextSample();
    virtual bool setSampleNumber(int64_t sampleNum);
    virtual float get1D() = 0;
    virtual Point2f get2D() = 0;
    virtual int roundCount(int n) const { return n; }
    virtual unique_ptr<Sampler> clone(int seed) = 0;

    CameraSample getCameraSample(const Point2i &pRaster);
    void request1DArray(int n);
    void request2DArray(int n);
    const float * get1DArray(int n);
    const Point2f * get2DArray(int n);

    string stateString() const {
      return STRING_PRINTF("(%d,%d), sample %" PRId64, currentPixel.x, currentPixel.y, currentPixelSampleIndex);
    }

    int64_t currentSampleNumber() const { return currentPixelSampleIndex; }

    const int64_t samplesPerPixel;

protected:
    Point2i currentPixel;
    int64_t currentPixelSampleIndex;
    vector<int> samples1DArraySizes, samples2DArraySizes;
    vector<vector<float>> sampleArray1D;
    vector<vector<Point2f>> sampleArray2D;

private:
    size_t array1DOffset, array2DOffset;
};

class PixelSampler : public Sampler {
public:
    PixelSampler(int64_t samplesPerPixel, int nSampledDimensions);

    bool startNextSample();
    bool setSampleNumber(int64_t);
    float get1D();
    Point2f get2D();

protected:
    vector<vector<float>> samples1D;
    vector<vector<Point2f>> samples2D;
    int current1DDimension = 0, current2DDimension = 0;
    Random rng;
};

class GlobalSampler : public Sampler {
public:
    GlobalSampler(int64_t samplesPerPixel) : Sampler(samplesPerPixel) {}

    bool startNextSample();
    void startPixel(const Point2i &);
    bool setSampleNumber(int64_t sampleNum);
    float get1D();
    Point2f get2D();

    virtual int64_t getIndexForSample(int64_t sampleNum) const = 0;
    virtual float sampleDimension(int64_t index, int dimension) const = 0;

private:
    int dimension;
    int64_t intervalSampleIndex;
    static const int arrayStartDim = 5;
    int arrayEndDim;
};

namespace Sampling {

inline Point2f rejectionSampleDisk(Random &rng) {
    Point2f p;
    do {
        p.x = 1 - 2 * rng.uniformFloat();
        p.y = 1 - 2 * rng.uniformFloat();
    } while (SQ(p.x) + SQ(p.y) > 1);
    return p;
}

inline Vector3f uniformSampleHemisphere(const Point2f &u) {
    float z = u[0];
    float r = sqrt(max(0.0f, 1.0f - z * z));
    float phi = 2 * PI * u[1];
    return Vector3f(r * cos(phi), r * sin(phi), z);
}

inline constexpr float uniformHemispherePdf() { return INV_TWO_PI; }

inline Vector3f uniformSampleSphere(const Point2f &u) {
    float z = 1 - 2 * u[0];
    float r = sqrt(max(0.0f, 1.0f - z * z));
    float phi = 2 * PI * u[1];
    return Vector3f(r * cos(phi), r * sin(phi), z);
}

inline constexpr float uniformSpherePdf() { return INV_FOUR_PI; }

inline Point2f uniformSampleDisk(const Point2f &u) {
    float r = sqrt(u[0]);
    float theta = 2 * PI * u[1];
    return Point2f(r * cos(theta), r * sin(theta));
}

inline Point2f concentricSampleDisk(const Point2f &u) {
    Point2f uOffset = 2.f * u - Vector2f(1, 1); // map to [-1, 1]
    if (uOffset.x == 0 && uOffset.y == 0) return Point2f(0, 0); // degeneracy at origin
    float theta, r;
    if (abs(uOffset.x) > abs(uOffset.y)) {
        r = uOffset.x;
        theta = PI_OVER_FOUR * (uOffset.y / uOffset.x);
    } else {
        r = uOffset.y;
        theta = PI_OVER_TWO - PI_OVER_FOUR * (uOffset.x / uOffset.y);
    }
    return r * Point2f(cos(theta), sin(theta));
}

inline Vector3f cosineSampleHemisphere(const Point2f &u) {
    Point2f d = concentricSampleDisk(u);
    float z = sqrt(max(0.0f, 1.0f - d.x * d.x - d.y * d.y));
    return Vector3f(d.x, d.y, z);
}

inline constexpr float cosineHemispherePdf(const float cosTheta) { return cosTheta * INV_PI; }

inline Vector3f uniformSampleCone(const Point2f &u, float cosThetaMax) {
    float cosTheta = 1.0f - u[0] + u[0] * cosThetaMax;
    float sinTheta = sqrt(1.0f - SQ(cosTheta));
    float phi = 2 * PI * u[1];
    return Vector3f(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

inline Vector3f uniformSampleCone(const Point2f &u, float cosThetaMax,
                                  const Vector3f &x, const Vector3f &y, const Vector3f &z) {
    float cosTheta = 1.0f - u[0] + u[0] * cosThetaMax;
    float sinTheta = sqrt(1.0f - SQ(cosTheta));
    float phi = 2 * PI * u[1];
    return cos(phi) * sinTheta * x + sin(phi) * sinTheta * y + cosTheta * z;
}

constexpr float uniformConePdf(const float cosThetaMax) {
    return 1.0f / (2 * PI * (1.0f - cosThetaMax));
}

inline Point2f uniformSampleTriangle(const Point2f &u) { // PDF just one over triangle area
    float su0 = sqrt(u[0]);
    return Point2f(1 - su0, u[1] * su0);
}

template <typename T>
inline void shuffle(T *samp, int count, int nDimensions, Random &rng) {
    for (int i = 0; i < count; ++i) {
        unsigned other = i + rng.uniformUInt32(count - i);
        for (int j = 0; j < nDimensions; ++j)
            swap(samp[nDimensions * i + j], samp[nDimensions * other + j]); // move entire blocks
    }
}

inline float balanceHeuristic(int nf, float fPdf, int ng, float gPdf) {
    return (nf * fPdf) / (nf * fPdf + ng * gPdf);
}

inline float powerHeuristic(int nf, float fPdf, int ng, float gPdf) {
    float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
}

};

struct Distribution1D {
    Distribution1D(const float *f, int n);

    float sampleContinuous(float u, float *pdf, int *off = nullptr) const;
    int sampleDiscrete(float u, float *pdf = nullptr, float *uRemapped = nullptr) const;

    int count() const { return int(func.size()); }

    float discretePDF(int index) const {
        CHECK(index >= 0 && index < count());
        return func[index] / (funcInt * count());
    }

    vector<float> func, cdf;
    float funcInt;
};

class Distribution2D {
public:
    Distribution2D(const float *data, int nu, int nv);

    Point2f sampleContinuous(const Point2f &u, float *pdf) const {
        float pdfs[2];
        int v;
        float d1 = pMarginal->sampleContinuous(u[1], &pdfs[1], &v);
        float d0 = pConditionalV[v]->sampleContinuous(u[0], &pdfs[0]);
        *pdf = pdfs[0] * pdfs[1];
        return Point2f(d0, d1);
    }

    float pdf(const Point2f &p) const {
        int iu = clamp(int(p[0] * pConditionalV[0]->count()), 0, pConditionalV[0]->count() - 1);
        int iv = clamp(int(p[1] * pMarginal->count()), 0, pMarginal->count() - 1);
        return pConditionalV[iv]->func[iu] / pMarginal->funcInt;
    }

private:
    vector<unique_ptr<Distribution1D>> pConditionalV;
    unique_ptr<Distribution1D> pMarginal;
};


#endif // CORE_SAMPLER
