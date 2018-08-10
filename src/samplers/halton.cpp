#include "halton.h"
#include "lowdiscrep.h"
#include "core/renderer.h"

using namespace LowDiscrepancy;

int HaltonSampler::primeSums[PrimeTableSize] = { 1 }; // uninitialized
vector<uint16_t> HaltonSampler::radicalInvPerms;

HaltonSampler::HaltonSampler(int samplesPerPixel, const Bounds2i &sampleBounds)
    : GlobalSampler(samplesPerPixel) {
    // Compute prime sums
    if (primeSums[0]) {
        int sum = 0;
        for (int i = 0; i < PrimeTableSize; i++) {
            primeSums[i] = sum;
            sum += Primes[i];
        }
    }

    // Generate random digit permutations for Halton sampler
    if (radicalInvPerms.empty()) {
        Random rng;
        radicalInvPerms = computeRadicalInversePermutations(rng);
    }

    // Find radical inverse base scales and exponents that cover sampling area
    Vector2i res = sampleBounds.pMax - sampleBounds.pMin;
    for (int i = 0; i < 2; ++i) {
        int base = (i == 0) ? 2 : 3;
        int scale = 1, exp = 0;
        while (scale < min(res[i], MaxResolution)) {
            scale *= base;
            ++exp;
        }
        baseScales[i] = scale;
        baseExponents[i] = exp;
    }

    // Compute stride in samples for visiting each pixel area
    sampleStride = baseScales[0] * baseScales[1];

    // Compute multiplicative inverses for _baseScales_
    multInv[0] = multiplicativeInverse(baseScales[1], baseScales[0]);
    multInv[1] = multiplicativeInverse(baseScales[0], baseScales[1]);
}

int64_t HaltonSampler::getIndexForSample(int64_t sampleNum) const {
    if (currentPixel != pixelForOffset) {
        // Compute Halton sample offset for _currentPixel_
        offsetForCurrentPixel = 0;
        if (sampleStride > 1) {
            Point2i pm(mod(currentPixel[0], MaxResolution), mod(currentPixel[1], MaxResolution));
            for (int i = 0; i < 2; ++i) {
                uint64_t dimOffset =
                        (i == 0)
                        ? inverseRadicalInverse<2>(pm[i], baseExponents[i])
                        : inverseRadicalInverse<3>(pm[i], baseExponents[i]);
                offsetForCurrentPixel +=
                    dimOffset * (sampleStride / baseScales[i]) * multInv[i];
            }
            offsetForCurrentPixel %= sampleStride;
        }
        pixelForOffset = currentPixel;
    }
    return offsetForCurrentPixel + sampleNum * sampleStride;
}

float HaltonSampler::sampleDimension(int64_t index, int dim) const {
    if (dim == 0)
        return radicalInverse(dim, index >> baseExponents[0]);
    else if (dim == 1)
        return radicalInverse(dim, index / baseScales[1]);
    else
        return scrambledRadicalInverse(dim, index, permutationForDimension(dim));
}

vector<uint16_t> HaltonSampler::computeRadicalInversePermutations(Random &rng) {
    vector<uint16_t> perms;
    // Allocate space in _perms_ for radical inverse permutations
    int permArraySize = Primes[PrimeTableSize - 1] + primeSums[PrimeTableSize - 1];
    perms.resize(permArraySize);
    uint16_t *p = &perms[0];
    for (int i = 0; i < PrimeTableSize; ++i) {
        // Generate random permutation for $i$th prime base
        for (int j = 0; j < Primes[i]; ++j) p[j] = j;
        Sampling::shuffle(p, Primes[i], 1, rng);
        p += Primes[i];
    }
    return perms;
}

void HaltonSampler::extendedGCD(uint64_t a, uint64_t b, int64_t *x, int64_t *y) {
    if (b == 0) {
        *x = 1;
        *y = 0;
        return;
    }
    int64_t d = a / b, xp, yp;
    extendedGCD(b, a % b, &xp, &yp);
    *x = yp;
    *y = xp - (d * yp);
}

HaltonSampler * HaltonSampler::create(const ParamSet &params, const Bounds2i &sampleBounds) {
    int nsamp = params.findOneInt("pixelsamples", 16);
    if (Renderer::options.quickRender) nsamp = 1;
    return new HaltonSampler(nsamp, sampleBounds);
}
