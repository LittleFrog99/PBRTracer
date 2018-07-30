#ifndef MEDIUM_GRID
#define MEDIUM_GRID

#include "core/medium.h"
#include "core/transform.h"

class GridDensityMedium : public Medium {
public:
    GridDensityMedium(const Spectrum &sigma_a, const Spectrum &sigma_s, float g, int nx, int ny, int nz,
                      const Transform &mediumToWorld, const float *d);

    Spectrum compute_Tr(const Ray &ray, Sampler &sampler) const;

private:
    float estimateDensity(const Point3f &p) const;

    float getDensity(const Point3i &p) const {
        Bounds3i sampleBounds(Point3i(0, 0, 0), Point3i(nx, ny, nz));
        if (!insideExclusive(p, sampleBounds)) return 0;
        return density[(p.z * ny + p.y) * nx + p.x];
    }

    const Spectrum sigma_a, sigma_s;
    const float g;
    const int nx, ny, nz;
    const Transform worldToMedium;
    unique_ptr<float[]> density;
    float sigma_t;
    float invMaxDensity;
};

#endif // MEDIUM_GRID
