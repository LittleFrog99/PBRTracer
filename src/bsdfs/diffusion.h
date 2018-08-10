#ifndef BSSRDF_DIFFUSION
#define BSSRDF_DIFFUSION

#include "core/bssrdf.h"

struct BSSRDFTable {
    BSSRDFTable(int nRhoSamples, int nRadiusSamples)
        : nRhoSamples(nRhoSamples), nRadiusSamples(nRadiusSamples),
          rhoSamples(new float[nRhoSamples]), radiusSamples(new float[nRadiusSamples]),
          profile(new float[nRhoSamples * nRadiusSamples]), profileCDF(new float[nRhoSamples * nRadiusSamples]),
          rhoEff(new float[nRhoSamples]) {}

    inline float getProfile(int rhoIndex, int radiusIndex) const {
        return profile[rhoIndex * nRadiusSamples + radiusIndex];
    }

    const int nRhoSamples, nRadiusSamples;
    unique_ptr<float[]> rhoSamples, radiusSamples;
    unique_ptr<float[]> profile;
    unique_ptr<float[]> profileCDF;
    unique_ptr<float[]> rhoEff;
};

class BeamDiffusionSSS : public SeparableBSSRDF { // photon beam diffusion method
public:
    BeamDiffusionSSS(const SurfaceInteraction &po, const Material *material, TransportMode mode, float eta,
                 const Spectrum &sigma_a, const Spectrum &sigma_s, const BSSRDFTable &table);

    Spectrum compute_Sr(float r) const;
    float sample_Sr(int channel, float u) const;
    float pdf_Sr(int channel, float u) const;

    static void fillTable(float g, float eta, BSSRDFTable *table);

private:
    static float multipleScattering(float sigma_s, float sigma_a, float g, float eta, float r);
    static float singleScattering(float sigma_s, float sigma_a, float g, float eta, float r);

    const BSSRDFTable &table;
    Spectrum sigma_t, rho;
};

#endif // BSSRDF_DIFFUSION
