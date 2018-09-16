#ifndef SAMPLER_PSS
#define SAMPLER_PSS

#include "core/sampling.h"

class PSSSampler : public Sampler {
public:
    PSSSampler(int mutationsPerPixel, uint rngSeqIndex, float sigma, float largeStepProb, uint streamCount)
        : Sampler(mutationsPerPixel), rng(rngSeqIndex), sigma(sigma), largeStepProb(largeStepProb),
          streamCount(streamCount) {}

    float get1D() {
        uint index = getNextIndex();
        ensureReady(index);
        return X[index].value;
    }

    Point2f get2D() { return Point2f(get1D(), get1D()); }

    unique_ptr<Sampler> clone(int seed) {
        auto sampler = make_unique<PSSSampler>(*this);
        sampler->rng = Random(seed);
        return sampler;
    }

    uint getNextIndex() { return streamIndex + streamCount * sampleIndex++; }

    void startIteration() {
        curIte++;
        largeStep = rng.uniformFloat() < largeStepProb;
    }

    void startStream(uint index) {
        streamIndex = index;
        sampleIndex = 0;
    }

    void accept() { if (largeStep) lastLargeStepIte = curIte; }

    void reject() {
        for (auto &Xi : X)
            if (Xi.lastModIte == curIte) Xi.restore();
        --curIte;
    }

private:
    struct PrimarySample {
        float value = 0;
        uint lastModIte = 0;
        float valueBackup = 0;
        uint modBackupIte = 0;

        void backup() {
            valueBackup = value;
            modBackupIte = lastModIte;
        }

        void restore() {
            value = valueBackup;
            lastModIte = modBackupIte;
        }
    };

    void ensureReady(uint index);

    Random rng;
    const float sigma, largeStepProb;
    const uint streamCount;
    vector<PrimarySample> X;
    uint curIte = 0;
    bool largeStep = true;
    uint lastLargeStepIte = 0;
    uint streamIndex, sampleIndex;
};

#endif // SAMPLER_PSS
