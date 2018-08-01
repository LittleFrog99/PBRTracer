#ifndef MEDIUM_HOMOGENEOUS
#define MEDIUM_HOMOGENEOUS

#include "core/medium.h"

class HomogeneousMedium : public Medium {
public:
    HomogeneousMedium(const Spectrum &sigma_a, const Spectrum &sigma_s, float g)
        : sigma_a(sigma_a), sigma_s(sigma_s), sigma_t(sigma_s + sigma_a), g(g) {}

    Spectrum compute_Tr(const Ray &ray, Sampler &) const {
        return exp(-sigma_t * min(ray.tMax * ray.d.length(), numeric_limits<float>::max()));
    }

private:
    const Spectrum sigma_a, sigma_s, sigma_t;
    const float g;
};

#endif // MEDIUM_HOMOGENEOUS
