#include "medium.h"
#include "stats.h"

float HenyeyGreenstein::sample_p(const Vector3f &wo, Vector3f *wi, const Point2f &u) const {
    ProfilePhase _(Stage::PhaseFuncSampling);
    // Compute cosθ for Henyey-Greenstein sample
    float cosTheta;
    if (abs(g) < 1e-3f)
        cosTheta = 1 - 2 * u[0];
    else {
        float sqrTerm = (1 - g * g) / (1 - g + 2 * g * u[0]);
        cosTheta = (1 + g * g - SQ(sqrTerm)) / (2 * g);
    }

    // Compute direction wi
    float sinTheta = sqrt(max(0.0f, 1 - SQ(cosTheta)));
    float phi = 2 * PI * u[1];
    Vector3f v1, v2;
    coordinateSystem(wo, &v1, &v2);
    *wi = sphericalDirection(sinTheta, cosTheta, phi, v1, v2, -wo);

    return phase(-cosTheta, g);
}
