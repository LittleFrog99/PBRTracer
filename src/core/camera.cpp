#include "camera.h"

float Camera::generateRayDifferential(const CameraSample &sample, RayDifferential *rd) const {
    float weight = generateRay(sample, rd);
    // Find camera ray after shifting one pixel in the x direction
    {
        CameraSample sshift = sample;
        sshift.pFilm.x++;
        Ray rx;
        float weightX = generateRay(sshift, &rx);
        if (weightX == 0) return 0;
        rd->rxOrigin = rx.o;
        rd->ryDirection = rx.d;
    }
    // Y direction
    {
        CameraSample sshift = sample;
        sshift.pFilm.y++;
        Ray ry;
        float weightY = generateRay(sshift, &ry);
        if (weightY == 0) return 0;
        rd->ryOrigin = ry.o;
        rd->ryDirection = ry.d;
    }

    rd->hasDifferentials = true;
    return weight;
}
