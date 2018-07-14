#include "ortho.h"
#include "core/sampling.h"

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
