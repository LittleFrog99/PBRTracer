#include "ortho.h"
#include "core/sampling.h"
#include "core/paramset.h"

Float OrthographicCamera::generateRay(const CameraSample &sample, Ray *ray) const {
    auto pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    auto pCam = rasterToCamera(pFilm);
    *ray = Ray(pCam, Vector3f(0, 0, 1));

    // Modify ray for depth of field
    if (lensRadius > 0) {
        auto pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);
        Float ft = focalDistance / ray->d.z;
        auto pFocus = (*ray)(ft);
        ray->o = Point3f(pLens.x, pLens.y, 0);
        ray->d = normalize(pFocus - ray->o);
    }

    ray->time = lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = cameraToWorld(*ray);
    return 1.0;
}

Float OrthographicCamera::generateRayDifferential(const CameraSample &sample, RayDifferential *ray) const
{
    auto pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    auto pCam = rasterToCamera(pFilm);
    *ray = RayDifferential(pCam, Vector3f(0, 0, 1));

    if (lensRadius > 0) {
        auto pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);
        Float ft = focalDistance / ray->d.z;
        auto pFocus = (*ray)(ft);
        ray->o = Point3f(pLens.x, pLens.y, 0);
        ray->d = normalize(pFocus - ray->o);
    }

    if (lensRadius > 0) { // accounting for lens
        Point2f pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);
        Float ft = focalDistance / ray->d.z;

        Point3f pFocus = pCam + dxCam + (ft * Vector3f(0, 0, 1));
        ray->rxOrigin = Point3f(pLens.x, pLens.y, 0);
        ray->rxDirection = normalize(pFocus - ray->rxOrigin);

        pFocus = pCam + dyCam + (ft * Vector3f(0, 0, 1));
        ray->ryOrigin = Point3f(pLens.x, pLens.y, 0);
        ray->ryDirection = normalize(pFocus - ray->ryOrigin);
    } else {
        ray->rxOrigin = ray->o + dxCam;
        ray->ryOrigin = ray->o + dyCam;
        ray->rxDirection = ray->ryDirection = ray->d;
    }

    ray->time = lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = cameraToWorld(*ray);
    return 1.0;
}

OrthographicCamera * OrthographicCamera::create(const ParamSet &params, const AnimatedTransform &cam2world,
                                                Film *film, const Medium *medium)
{
    Float shutteropen = params.findOneFloat("shutteropen", 0.f);
    Float shutterclose = params.findOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        WARNING("Shutter close time [%f] < shutter open [%f].  Swapping them.", shutterclose, shutteropen);
        swap(shutterclose, shutteropen);
    }
    Float lensradius = params.findOneFloat("lensradius", 0.f);
    Float focaldistance = params.findOneFloat("focaldistance", 1e6f);
    Float frame = params.findOneFloat("frameaspectratio",
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
    return new OrthographicCamera(cam2world, screen, shutteropen, shutterclose, lensradius, focaldistance,
                                  film, medium);
}
