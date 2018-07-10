#include "environment.h"

Float EnvironmentCamera::generateRay(const CameraSample &sample, Ray *ray) const {
    Float theta = PI * sample.pFilm.y / film->fullResolution.y;
    Float phi = 2 * PI * sample.pFilm.x / film->fullResolution.x;
    Vector3f dir(sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi));
    *ray = Ray(Point3f(), dir, INFINITY, lerp(sample.time, shutterOpen, shutterClose));
    ray->medium = medium;
    *ray = cameraToWorld(*ray);
    return 1;
}
