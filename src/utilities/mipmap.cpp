#include "mipmap.h"
#include "core/spectrum.h"
#include "log.h"
#include "parallel.h"

template <class T>
float Mipmap<T>::weightLut[WEIGHT_LUT_SIZE] = { 0.0f };

template <class T>
Mipmap<T>::Mipmap(const Point2i &res, const T *img, bool doTrilinear, float maxAnisotropy, ImageWrap wrapMode)
    : doTrilinear(doTrilinear), maxAnisotropy(maxAnisotropy), wrapMode(wrapMode), resolution(res)
{
    unique_ptr<T[]> resampled = nullptr;
    if (!isPowerOf2(resolution[0]) || !isPowerOf2(resolution[1])) {
        // Resample imaged to power-of-2 resolution
        Point2i resPow2(roundUpPow2(resolution[0]), roundUpPow2(resolution[1]));

        // Resample image in s direction
        unique_ptr<ResampleWeight[]> sWeights = resampleWeights(resolution[0], resPow2[0]);
        resampled.reset(new T[resPow2[0] * resPow2[1]]);
        Parallel::forLoop([&] (int t) {
            for (int s = 0; s < resPow2[0]; ++s) {
                // Compute texel (s,t) in s-zoomed image
                resampled[t * resPow2[0] + s] = 0.0f;
                for (int j = 0; j < 4; ++j) {
                    int origS = sWeights[s].firstTexel + j;
                    if (wrapMode == ImageWrap::Repeat)
                        origS = mod(origS, resolution[0]);
                    else if (wrapMode == ImageWrap::Clamp)
                        origS = clamp(origS, 0, resolution[0] - 1);
                    if (origS >= 0 && origS < (int)resolution[0])
                        resampled[t * resPow2[0] + s] += sWeights[s].weight[j] * img[t * resolution[0] + origS];
                }
            }
        }, resolution[1], 16);

        // t direction
        unique_ptr<ResampleWeight[]> tWeights = resampleWeights(resolution[1], resPow2[1]);
        vector<T *> resampleBufs;
        int nThreads = Parallel::maxThreadIndex();
        for (int i = 0; i < nThreads; ++i)
            resampleBufs.push_back(new T[resPow2[1]]);
        Parallel::forLoop([&](int s) {
            T *workData = resampleBufs[Parallel::getThreadIndex()];
            for (int t = 0; t < resPow2[1]; ++t) {
                workData[t] = 0.0f;
                for (int j = 0; j < 4; ++j) {
                    int offset = tWeights[t].firstTexel + j;
                    if (wrapMode == ImageWrap::Repeat)
                        offset = mod(offset, resolution[1]);
                    else if (wrapMode == ImageWrap::Clamp)
                        offset = clamp(offset, 0, resolution[1] - 1);
                    if (offset >= 0 && offset < resolution[1])
                        workData[t] += tWeights[t].weight[j] * resampled[offset * resPow2[0] + s];
                }
            }
            for (int t = 0; t < resPow2[1]; ++t)
                resampled[t * resPow2[0] + s] = workData[t];
        }, resPow2[0], 32);
        for (auto ptr : resampleBufs) delete[] ptr;

        resolution = resPow2;
    }

    // Initialize levels of Mipmap from image
    int nLevels = 1 + log2Int(max(resolution[0], resolution[1]));
    pyramid.resize(nLevels);
    // Initialize most detailed level of Mipmap
    pyramid[0].reset(new BlockedArray<T, 2>(resolution[0], resolution[1], resampled ? resampled.get() : img));
    for (int i = 0; i < nLevels; i++) {
        // // Initialize ith Mipmap from i-1th level
        int sRes = max(1, pyramid[i - 1]->uSize() / 2);
        int tRes = max(1, pyramid[i - 1]->vSize() / 2);
        pyramid[i].reset(new BlockedArray<T, LOG_BLOCK_SIZE>(sRes, tRes));

        // Filter four texels from finer level of pyramid
        Parallel::forLoop([&] (int t) {
            for (int s = 0; s < sRes; s++)
                (*pyramid[i])(s, t) = 0.25f * (texel(i-1, 2*s, 2*t) + texel(i-1, 2*s+1, 2*t) +
                                               texel(i-1, 2*s, 2*t+1) + texel(i-1, 2*s+1, 2*t+1));
        }, tRes, 16);
    }

    // Initialize EWA filter weights if needed
    if (weightLut[0] == 0.0f)
        for (int i = 0; i < WEIGHT_LUT_SIZE; i++) {
            constexpr float alpha = 2;
            float r2 = float(i) / float(WEIGHT_LUT_SIZE - 1);
            weightLut[i] = exp(-alpha * r2) - exp(-alpha);
        }
}

template <class T>
T Mipmap<T>::lookup(const Point2f &st, float width) const {
    // Compute mipmap level for trilinear filtering
    float level = levels() - 1 + Math::log2(max(width, 1e-8f));
    // Preform trilinear interpolation
    if (level < 0)
        return triangle(0, st);
    else if (level > levels() - 1)
        return texel(levels() - 1, 0, 0);
    else {
        int iLevel = floor(level);
        float delta = level - iLevel;
        return lerp(delta, triangle(iLevel, st), triangle(iLevel + 1, st));
    }
}

template <class T>
T Mipmap<T>::triangle(unsigned level, const Point2f &st) const {
    level = clamp(level, 0, levels() - 1);
    float s = st[0] * pyramid[level]->uSize() - 0.5f; // to correctly compute distances to discrete coordinates
    float t = st[1] * pyramid[level]->vSize() - 0.5f;
    int s0 = floor(s), t0 = floor(t);
    float ds = s - s0, dt = t - t0;
    return (1 - ds) * (1 - dt) * texel(level, s0, t0) + (1 - ds) * dt * texel(level, s0, t0 + 1) +
            ds * (1 - dt) * texel(level, s0 + 1, t0) + ds * dt * texel(level, s0 + 1, t0 + 1);
}

template <class T>
T Mipmap<T>::lookup(const Point2f &st, Vector2f dst0, Vector2f dst1) const {
    if (doTrilinear) {
        float width = max(max(dst0[0], dst0[1]), max(dst1[0], dst1[1]));
        return lookup(st, 2 * width);
    }

    // Compute ellipse minor and major axes
    if (dst0.lengthSq() < dst1.lengthSq())
        swap(dst0, dst1);
    float majorLen = dst0.length(), minorLen = dst1.length();

    // Clamp ellipse eccentricity if too large
    if (minorLen * maxAnisotropy < majorLen && minorLen > 0) {
        float scale = majorLen / (minorLen * maxAnisotropy);
        dst1 *= scale;
        minorLen *= scale;

    }
    if (minorLen == 0)
        return triangle(0, st);

    // Choose level of detail for EWA lookup and perform EWA filtering
    float level = max(0.0f, levels() - 1.0f + Math::log2(minorLen));
    unsigned iLevel = floor(level);
    return lerp(level - iLevel, EWA(iLevel, st, dst0, dst1), EWA(iLevel + 1, st, dst0, dst1));
}

template <class T>
T Mipmap<T>::EWA(unsigned level, Point2f st, Vector2f dst0, Vector2f dst1) const {
    if (level >= levels()) return texel(levels() - 1, 0, 0);

    // Convert EWA coordinates to appropriate scale for level
    st[0] = st[0] * pyramid[level]->uSize() - 0.5f;
    st[1] = st[1] * pyramid[level]->vSize() - 0.5f;
    dst0[0] *= pyramid[level]->uSize();
    dst0[1] *= pyramid[level]->vSize();
    dst1[0] *= pyramid[level]->uSize();
    dst1[1] *= pyramid[level]->vSize();

    // Compute ellipse coefficeint to ound EWA filter region
    float A = SQ(dst0[1]) + SQ(dst1[1]) + 1;
    float B = -2 * (dst0[0] * dst0[1] + dst1[0] * dst1[1]);
    float C = SQ(dst0[0]) + SQ(dst1[0]) + 1;
    float invF = 1.0f / (A * C - 0.25 * B * B);
    A *= invF;
    B *= invF;
    C *= invF;

    // Compute the ellipse's bounding box in texture space
    float det = -B * B + 4 * A * C;
    float invDet = 1.0f / det;
    float uSqrt = sqrt(det * C), vSqrt = sqrt(det * A);
    int s0 = ceil(st[0] - 2 * invDet * uSqrt);
    int s1 = floor(st[0] + 2 * invDet * uSqrt);
    int t0 = ceil(st[1] - 2 * invDet * vSqrt);
    int t1 = floor(st[1] + 2 * invDet * vSqrt);

    // Scan over ellipse bound and compute quadratic equation
    T sum(0.0f);
    float sumWeights = 0;
    for (int it = t0; it <= t1; it++) {
        float tt = it - st[1];
        for (int is = s0; is <= s1; is++) {
            float ss = is - st[0];
            float r2 = A * SQ(ss) + B * ss * tt + C * SQ(tt);
            if (r2 < 1) {
                int index = min(int(r2 * WEIGHT_LUT_SIZE), WEIGHT_LUT_SIZE - 1);
                float weight = weightLut[index];
                sum += texel(level, is, it) * weight;
            }
        }
    }
    return sum / sumWeights;
}

template <class T>
const T & Mipmap<T>::texel(unsigned level, int s, int t) const {
    const BlockedArray<T, LOG_BLOCK_SIZE> &map = *pyramid[level];
    switch (wrapMode) {
    case ImageWrap::Repeat:
        s = mod(s, map.uSize());
        t = mod(t, map.vSize());
        break;
    case ImageWrap::Clamp:
        s = clamp(s, 0, map.uSize() - 1);
        t = clamp(t, 0, map.vSize() - 1);
        break;
    case ImageWrap::Black: {
        static const T BLACK = 0.0f;
        if (s < 0 || s >= map.uSize() || t < 0 || t >= map.vSize())
            return BLACK;
        break;
    }
    }
    return map(s, t);
}

template <class T>
unique_ptr<ResampleWeight[]> Mipmap<T>::resampleWeights(int oldRes, int newRes) {
    DCHECK_GE(newRes, oldRes);
    unique_ptr<ResampleWeight[]> wt(new ResampleWeight[newRes]);
    float filterWidth = 2.0f;
    for (int i = 0; i < newRes; i++) {
        // Compute image resampling weights for ith texel
        float center = (i + 0.5f) * oldRes / newRes;
        wt[i].firstTexel = floor(center - filterWidth + 0.5f); // an offset to lower end is needed
        for (int j = 0; j < 4; j++) {
            float pos = wt[i].firstTexel + j + 0.5f;
            wt[i].weight[j] = lanczos((pos - center) / filterWidth);
        }
        // Normalize filter weights for texel sampling
        float invSumWts = 1.0f / wt[i].sumWeight();
        for (int j = 0; j < 4; j++) wt[i].weight[j] *= invSumWts;
    }
    return wt;
}

template class Mipmap<float>;
template class Mipmap<Spectrum>;
