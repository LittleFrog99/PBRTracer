#include "perspective.h"
#include "core/sampling.h"

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
