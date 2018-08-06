#include "diffusion.h"
#include "interpolation.h"

DiffusionSSS::DiffusionSSS(const SurfaceInteraction &po, const Material *material, TransportMode mode,
                           float eta, const Spectrum &sigma_a, const Spectrum &sigma_s, const BSSRDFTable &table)
    : SeparableBSSRDF(po, eta, material, mode), table(table)
{
    sigma_t = sigma_a + sigma_s;
    for (int c = 0; c < Spectrum::nSamples; c++)
        rho[c] = sigma_t[c] == 0 ? 0 : (sigma_s[c] / sigma_t[c]);
}

Spectrum DiffusionSSS::compute_Sr(float r) const {
    Spectrum Sr;
    for (int ch = 0; ch < Spectrum::nSamples; ch++) {
        float rOpt = r * sigma_t[ch]; // unitless optical radius
        // Compute spline weights
        int rhoOffset, radiusOffset;
        float rhoWeights[4], radiusWeights[4];
        using namespace Interpolation;
        if (!catmullRomWeights(table.nRhoSamples, table.rhoSamples.get(), rho[ch],
                               &rhoOffset, rhoWeights) ||
            !catmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(), rOpt,
                               &radiusOffset, radiusWeights))
            continue;

        // Set Sr[ch] using tensor spline interpolation
        float srCh = 0;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                float weight = rhoWeights[i] * radiusWeights[j];
                if (weight != 0)
                    srCh += weight * table.getProfile(rhoOffset + i, radiusOffset + j);
            }
        if (rOpt != 0) srCh /= 2 * PI * rOpt;
        Sr[ch] = srCh;
    }
    Sr *= SQ(sigma_t); // transform Sr from unitless value to world space units
    return Sr.clamp();
}
