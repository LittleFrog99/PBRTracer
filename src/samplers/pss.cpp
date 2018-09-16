#include "pss.h"

void PSSSampler::ensureReady(uint index) {
    // Enlarge X if necessary and get current Xi
    if (index >= X.size()) X.resize(index + 1);
    auto &Xi = X[index];

     // Reset Xi if a large step took place in the meantime
    if (Xi.lastModIte < lastLargeStepIte) {
        Xi.value = rng.uniformFloat();
        Xi.lastModIte = lastLargeStepIte;
    }

    // Apply remaining sequence of mutations to sample
    Xi.backup();
    if (largeStep)
        Xi.value = rng.uniformFloat();
    else {
        uint64_t nSmall = curIte - Xi.lastModIte;

        // Apply _nSmall_ small step mutations
        // Sample the standard normal distribution
        float normalSample = SQRT_TWO * erfInv(2 * rng.uniformFloat() - 1);

        // Compute the effective standard deviation and apply perturbation to Xi
        float effSigma = sigma * sqrt(float(nSmall));
        Xi.value += normalSample * effSigma;
        Xi.value -= floor(Xi.value);
    }

    Xi.lastModIte = curIte;
}
