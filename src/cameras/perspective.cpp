#include "perspective.h"
#include "core/sampling.h"
#include "core/paramset.h"

PerspectiveCamera::PerspectiveCamera(const AnimatedTransform &camToWorld, const Bounds2f &screenWindow,
                                     Float shutterOpen, Float shutterClose, Float lensRadius,
                                     Float focalDist, Float fov, Film *film, const Medium *medium)
    : ProjectiveCamera(camToWorld, Transform::perspective(fov, 1e-2f, 1e3f), screenWindow,
                       shutterOpen, shutterClose, lensRadius, focalDist, film, medium)
{
    dxCam = rasterToCamera(Point3f(1, 0, 0)) - rasterToCamera(Point3f());
    dyCam = rasterToCamera(Point3f(0, 1, 0)) - rasterToCamera(Point3f());
}

Float PerspectiveCamera::generateRay(const CameraSample &sample, Ray *ray) const {
    auto pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    auto pCamera = rasterToCamera(pFilm);
    *ray = Ray(Point3f(), normalize(Vector3f(pCamera)));

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
    return 1;
}

Float PerspectiveCamera::generateRayDifferential(const CameraSample &sample, RayDifferential *ray) const {
    auto pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    auto pCam = rasterToCamera(pFilm);
    *ray = Ray(Point3f(), normalize(Vector3f(pCam)));

    // Modify ray for depth of field
    if (lensRadius > 0) {
        auto pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);
        Float ft = focalDistance / ray->d.z;
        auto pFocus = (*ray)(ft);
        ray->o = Point3f(pLens.x, pLens.y, 0);
        ray->d = normalize(pFocus - ray->o);
    }

    // Compute offset rays for ray differentials
    if (lensRadius > 0) { // accounting for lens
        Point2f pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);

        Vector3f dx = normalize(Vector3f(pCam + dxCam));
        Float ft = focalDistance / dx.z;
        Point3f pFocus = Point3f(0, 0, 0) + (ft * dx);
        ray->rxOrigin = Point3f(pLens.x, pLens.y, 0);
        ray->rxDirection = normalize(pFocus - ray->rxOrigin);

        Vector3f dy = normalize(Vector3f(pCam + dyCam));
        ft = focalDistance / dy.z;
        pFocus = Point3f(0, 0, 0) + (ft * dy);
        ray->ryOrigin = Point3f(pLens.x, pLens.y, 0);
        ray->ryDirection = normalize(pFocus - ray->ryOrigin);
    } else {
        ray->rxOrigin = ray->ryOrigin = ray->o;
        ray->rxDirection = normalize(Vector3f(pCam) + dxCam);
        ray->ryDirection = normalize(Vector3f(pCam) + dyCam);
    }

    ray->time = lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = cameraToWorld(*ray);
    return 1;
}

PerspectiveCamera * PerspectiveCamera::create(const ParamSet &params, const AnimatedTransform &cam2world,
                                              Film *film, const Medium *medium)
{
    Float shutteropen = params.findOneFloat("shutteropen", 0.f);
    Float shutterclose = params.findOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        WARNING("Shutter close time [%f] < shutter open [%f].  Swapping them.",
                shutterclose, shutteropen);
        swap(shutterclose, shutteropen);
    }
    Float lensradius = params.findOneFloat("lensradius", 0.f);
    Float focaldistance = params.findOneFloat("focaldistance", 1e6);
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
    Float fov = params.findOneFloat("fov", 90.);
    Float halffov = params.findOneFloat("halffov", -1.f);
    if (halffov > 0.f)
        // hack for structure synth, which exports half of the full fov
        fov = 2.f * halffov;
    return new PerspectiveCamera(cam2world, screen, shutteropen, shutterclose, lensradius, focaldistance,
                                 fov, film, medium);
}
