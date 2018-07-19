#include "environment.h"
#include "paramset.h"

Float EnvironmentCamera::generateRay(const CameraSample &sample, Ray *ray) const {
    Float theta = PI * sample.pFilm.y / film->fullResolution.y;
    Float phi = 2 * PI * sample.pFilm.x / film->fullResolution.x;
    Vector3f dir(sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi));
    *ray = Ray(Point3f(), dir, INFINITY, lerp(sample.time, shutterOpen, shutterClose));
    ray->medium = medium;
    *ray = cameraToWorld(*ray);
    return 1;
}

EnvironmentCamera * EnvironmentCamera::create(const ParamSet &params, const AnimatedTransform &cam2world, Film *film,
                                              const Medium *medium)
{
    Float shutteropen = params.findOneFloat("shutteropen", 0.f);
    Float shutterclose = params.findOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        WARNING("Shutter close time [%f] < shutter open [%f].  Swapping them.",
                shutterclose, shutteropen);
        std::swap(shutterclose, shutteropen);
    }
    Float lensradius = params.findOneFloat("lensradius", 0.f);
    Float focaldistance = params.findOneFloat("focaldistance", 1e30f);
    Float frame = params.findOneFloat(
        "frameaspectratio",
        Float(film->fullResolution.x) / Float(film->fullResolution.y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    int swi;
    const Float *sw = params.findFloat("screenwindow", &swi);
    if (sw) {
        if (swi == 4) {
            screen.pMin.x = sw[0];
            screen.pMax.x = sw[1];
            screen.pMin.y = sw[2];
            screen.pMax.y = sw[3];
        } else
            ERROR("\"screenwindow\" should have four values");
    }
    return new EnvironmentCamera(cam2world, shutteropen, shutterclose, film, medium);
}
