#include "diffusion.h"
#include "interpolation.h"
#include "parallel.h"
#include "stats.h"

using namespace Interpolation;

BeamDiffusionSSS::BeamDiffusionSSS(const SurfaceInteraction &po, const Material *material, TransportMode mode,
                           float eta, const Spectrum &sigma_a, const Spectrum &sigma_s, const BSSRDFTable &table)
    : SeparableBSSRDF(po, eta, material, mode), table(table)
{
    sigma_t = sigma_a + sigma_s;
    for (int c = 0; c < Spectrum::nSamples; c++)
        rho[c] = sigma_t[c] == 0 ? 0 : (sigma_s[c] / sigma_t[c]);
}

Spectrum BeamDiffusionSSS::compute_Sr(float r) const {
    Spectrum Sr(0.f);
    for (int ch = 0; ch < Spectrum::nSamples; ++ch) {
        // Convert r into unitless optical radius
        float rOptical = r * sigma_t[ch];

        // Compute spline weights to interpolate BSSRDF on channel _ch_
        int rhoOffset, radiusOffset;
        float rhoWeights[4], radiusWeights[4];
        if (!catmullRomWeights(table.nRhoSamples, table.rhoSamples.get(), rho[ch], &rhoOffset, rhoWeights) ||
            !catmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(), rOptical, &radiusOffset,
                               radiusWeights))
            continue;

        // Set BSSRDF value _Sr[ch]_ using tensor spline interpolation
        float sr = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float weight = rhoWeights[i] * radiusWeights[j];
                if (weight != 0)
                    sr += weight * table.getProfile(rhoOffset + i, radiusOffset + j);
            }
        }

        // Cancel marginal PDF factor from tabulated BSSRDF profile
        if (rOptical != 0) sr /= 2 * PI * rOptical;
        Sr[ch] = sr;
    }

    // Transform BSSRDF value into world space units
    Sr *= sigma_t * sigma_t;
    return Sr.clamp();
}

float BeamDiffusionSSS::sample_Sr(int ch, float u) const {
    if (sigma_t[ch] == 0) return -1;
    return sampleCatmullRom2D(table.nRhoSamples, table.nRadiusSamples, table.rhoSamples.get(),
                              table.radiusSamples.get(), table.profile.get(), table.profileCDF.get(),
                              rho[ch], u) / sigma_t[ch];
}

float BeamDiffusionSSS::pdf_Sr(int ch, float r) const {  // Similar to compute_Sr()
    // Convert r into unitless optical radius
    float rOptical = r * sigma_t[ch];

    // Compute spline weights to interpolate BSSRDF density on channel _ch_
    int rhoOffset, radiusOffset;
    float rhoWeights[4], radiusWeights[4];
    if (!catmullRomWeights(table.nRhoSamples, table.rhoSamples.get(), rho[ch], &rhoOffset, rhoWeights) ||
        !catmullRomWeights(table.nRadiusSamples, table.radiusSamples.get(), rOptical, &radiusOffset,
                           radiusWeights))
        return 0.f;

    // Return BSSRDF profile density for channel _ch_
    float sr = 0, rhoEff = 0;
    for (int i = 0; i < 4; ++i) {
        if (rhoWeights[i] == 0) continue;
        rhoEff += table.rhoEff[rhoOffset + i] * rhoWeights[i];
        for (int j = 0; j < 4; ++j) {
            if (radiusWeights[j] == 0) continue;
            sr += table.getProfile(rhoOffset + i, radiusOffset + j) * rhoWeights[i] * radiusWeights[j];
        }
    }

    // Cancel marginal PDF factor from tabulated BSSRDF profile
    if (rOptical != 0) sr /= 2 * PI * rOptical;
    return max(0.0f, sr * sigma_t[ch] * sigma_t[ch] / rhoEff);
}

void BeamDiffusionSSS::fillTable(float g, float eta, BSSRDFTable *table) {
    // Choose radius values of discretization
    table->radiusSamples[0] = 0;
    table->radiusSamples[1] = 2.5e-3f;
    for (int i = 2; i < table->nRadiusSamples; i++)
        table->radiusSamples[i] = table->radiusSamples[i - 1] * 1.2f;

    // Choose albedo values
    for (int i = 0; i < table->nRhoSamples; i++)
        table->rhoSamples[i] = (1.0f - exp(-8.0f * i / float(table->nRhoSamples - 1))) / (1.0f - exp(-8.0f));

    Parallel::forLoop([&] (int i) {
        for (int j = 0; j < table->nRadiusSamples; j++) {
            float rho = table->rhoSamples[i], radius = table->radiusSamples[j];
            table->profile[i * table->nRadiusSamples + j] = 2 * PI * radius *
                                                            (singleScattering(rho, 1 - rho, g, eta, radius) +
                                                             multipleScattering(rho, 1 - rho, g, eta, radius));
        }
        table->rhoEff[i] = integrateCatmullRom(table->nRadiusSamples, table->radiusSamples.get(),
                                               &table->profile[i * table->nRadiusSamples],
                &table->profileCDF[i * table->nRadiusSamples]);
    }, table->nRhoSamples);
}

float BeamDiffusionSSS::multipleScattering(float sigma_s, float sigma_a, float g, float eta, float r)
{
    constexpr int nSamples = 100;
    float Ed = 0;

    // Precompute information for dipole integrand
    float sigmap_s = sigma_s * (1 - g); // reduced coefficients
    float sigmap_t = sigma_a + sigmap_s;
    float rhop = sigmap_s / sigmap_t;
    float D_g = (2 * sigma_a + sigmap_s) / (3 * SQ(sigmap_t)); // non-classical diffusion 15.24
    float sigma_tr = sqrt(sigma_a / D_g); // effective transport coeffcient
    float fm1 = Fresnel::moment1(eta), fm2 = Fresnel::moment2(eta);
    float ze = -2 * D_g * (1 + 3 * fm2) / (1 - 2 * fm1); // linear extrapolation distance 15.28
    float cPhi = 0.25f * (1 - 2 * fm1), cE = 0.5f * (1 - 3 * fm2); // exitance scale factors 15.31, 15.32

    for (int i = 0; i < nSamples; i++) {
        // Sample real point source depth
        float zr = -log(1 - (i + 0.5f) / nSamples) / sigmap_t;
        float zv = -zr + 2 * ze;

        // Evaluate Ed at zr
        float dr = sqrt(SQ(r) + SQ(zr)), dv = sqrt(SQ(r) + SQ(zv));
        float phiD = INV_FOUR_PI / D_g *
                     (exp(-sigma_tr * dr) / dr - exp(-sigma_tr * dv) / dv); // fluence rate 15.25
        float EDn =  INV_FOUR_PI *
                     (zr * (1 + sigma_tr * dr) * exp(-sigma_tr * dr) / CUB(dr) -
                      zv * (1 + sigma_tr * dv) * exp(-sigma_tr * dv) / CUB(dv)); // vector irradiance 15.27

        // Add scaled contribution to from zr to Ed
        float E = cPhi * phiD + cE * EDn;
        float kappa = 1 - exp(-2 * sigmap_t * (zr + dr)); // correction factor for light source close to surface
        Ed += kappa * rhop * rhop * E;
    }

    return Ed / nSamples;
}

float BeamDiffusionSSS::singleScattering(float sigma_s, float sigma_a, float g, float eta, float r)
{
    float sigma_t = sigma_a + sigma_s, rho = sigma_s / sigma_t;
    float tCrit = r * sqrt(eta * eta - 1);
    float Ess = 0;
    constexpr int nSamples = 100;
    for (int i = 0; i < nSamples; ++i) {
        float ti = tCrit - log(1 - (i + .5f) / nSamples) / sigma_t;
        float d = sqrt(r * r + ti * ti);
        float cosThetaO = ti / d;

        Ess += rho * exp(-sigma_t * (d + tCrit)) / (d * d) * HenyeyGreenstein::phase(cosThetaO, g) *
               (1 - Fresnel::dielectric_Fr(-cosThetaO, 1, eta)) * abs(cosThetaO);
    }

    return Ess / nSamples;
}
