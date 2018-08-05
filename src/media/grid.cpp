#include "grid.h"
#include "log.h"
#include "core/sampling.h"
#include "core/interaction.h"

GridDensityMedium::GridDensityMedium(const Spectrum &sigma_a, const Spectrum &sigma_s, float g,
                                     int nx, int ny, int nz, const Transform &mediumToWorld, const float *d)
    : sigma_a(sigma_a), sigma_s(sigma_s), g(g), nx(nx), ny(ny), nz(nz), worldToMedium(mediumToWorld.inverse()),
      density(new float[nx * ny * nz])
{
    memcpy(density.get(), d, sizeof(float) * nx * ny * nz);

    // Precompute values for Monte Carlo sampling of _GridDensityMedium_
    sigma_t = (sigma_a + sigma_s)[0];
    if (Spectrum(sigma_t) != sigma_a + sigma_s)
        ERROR("GridDensityMedium requires a spectrally uniform attenuation coefficient!");
    float maxDensity = 0;
    for (int i = 0; i < nx * ny * nz; ++i)
        maxDensity = max(maxDensity, density[i]);
    invMaxDensity = 1 / maxDensity;
}

Spectrum GridDensityMedium::compute_Tr(const Ray &rWorld, Sampler &sampler) const {
    Ray ray = worldToMedium(Ray(rWorld.o, normalize(rWorld.d), rWorld.tMax * rWorld.d.length()));
    // Compute [tMin, tMax] interval of ray's overlap with medium bounds
    const Bounds3f b(Point3f(0, 0, 0), Point3f(1, 1, 1));
    float tMin, tMax;
    if (!b.intersectP(ray, &tMin, &tMax)) return Spectrum(1.f);

    // Perform ratio tracking to estimate the transmittance value
    float Tr = 1, t = tMin;
    while (true) {
        t -= log(1 - sampler.get1D()) * invMaxDensity / sigma_t;
        if (t >= tMax) break;
        float density = estimateDensity(ray(t));
        Tr *= 1 - max(0.0f, density * invMaxDensity);
        // Apply Russian roulette to terminate sampling.
        constexpr float rrThreshold = 0.1f;
        if (Tr < rrThreshold) {
            float q = max(0.05f, 1 - Tr);
            if (sampler.get1D() < q) return 0;
            Tr /= 1 - q;
        }
    }
    return Tr;
}

Spectrum GridDensityMedium::sample(const Ray &rWorld, Sampler &sampler, MemoryArena &arena,
                                   MediumInteraction *mi) const
{
    Ray ray = worldToMedium(Ray(rWorld.o, normalize(rWorld.d), rWorld.tMax * rWorld.d.length()));
    // Compute [tMin, tMax] interval of ray's overlap with medium bounds
    const Bounds3f b(Point3f(0, 0, 0), Point3f(1, 1, 1));
    float tMin, tMax;
    if (!b.intersectP(ray, &tMin, &tMax)) return Spectrum(1.f);

    // Run delta tracking iterations to sample a medium interaction
    float t = tMin;
    while (true) {
        t -= log(1 - sampler.get1D()) * invMaxDensity / sigma_t;
        if (t >= tMax) break; // criteria 1: left media without an iteraction
        if (sampler.get1D() < estimateDensity(ray(t)) * invMaxDensity) {
            PhaseFunction *phase = ARENA_ALLOC(arena, HenyeyGreenstein)(g);
            *mi = MediumInteraction(rWorld(t), -rWorld.d, rWorld.time, this, phase);
            return sigma_s / sigma_t; // sampling medium
        }
    }
    return 1.0f; // sampling surface
}

float GridDensityMedium::estimateDensity(const Point3f &p) const {
    // Compute voxel coordinates and offsets for _p_
    Point3f pSamples(p.x * nx - .5f, p.y * ny - .5f, p.z * nz - .5f);
    Point3i pi = Point3i(floor(pSamples));
    Vector3f d = pSamples - Point3f(pi);

    // Trilinearly interpolate density values to compute local density
    float d00 = lerp(d.x, getDensity(pi), getDensity(pi + Vector3i(1, 0, 0)));
    float d10 = lerp(d.x, getDensity(pi + Vector3i(0, 1, 0)), getDensity(pi + Vector3i(1, 1, 0)));
    float d01 = lerp(d.x, getDensity(pi + Vector3i(0, 0, 1)), getDensity(pi + Vector3i(1, 0, 1)));
    float d11 = lerp(d.x, getDensity(pi + Vector3i(0, 1, 1)), getDensity(pi + Vector3i(1, 1, 1)));
    float d0 = lerp(d.y, d00, d10);
    float d1 = lerp(d.y, d01, d11);
    return lerp(d.z, d0, d1);
}
