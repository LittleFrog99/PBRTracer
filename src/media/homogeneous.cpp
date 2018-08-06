#include "homogeneous.h"
#include "core/sampling.h"
#include "core/interaction.h"
#include "stats.h"

Spectrum HomogeneousMedium::sample(const Ray &ray, Sampler &sampler, MemoryArena &arena,
                                   MediumInteraction *mi) const
{
    ProfilePhase _(Stage::MediumSample);
    // Sample a channel and distance along the ray
    int channel = min(int(sampler.get1D() * Spectrum::nSamples), Spectrum::nSamples - 1);
    float dist = -logf(1 - sampler.get1D()) / sigma_t[channel];
    float t = min(dist / ray.d.length(), ray.tMax);
    bool sampledMedium = t < ray.tMax;
    if (sampledMedium)
        *mi = MediumInteraction(ray(t), -ray.d, ray.time, this, ARENA_ALLOC(arena, HenyeyGreenstein)(g));

    // Compute bean transmittance
    Spectrum Tr = exp(-sigma_t * min(t, MAX_FLOAT) * ray.d.length());
    // Return weighting factor for scattering from homogeneous medium
    Spectrum density = sampledMedium ? (sigma_t * Tr) : Tr;
    float pdf = 0;
    for (int i = 0; i < Spectrum::nSamples; i++)
        pdf += density[i];
    pdf *= 1.0f / Spectrum::nSamples;
    return sampledMedium ? (Tr * sigma_s / pdf) : (Tr / pdf);
}
