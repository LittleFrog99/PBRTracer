#ifndef CAMERA_ORTHOGRAPHIC
#define CAMERA_ORTHOGRAPHIC

#include "core/camera.h"

class OrthographicCamera : public ProjectiveCamera {
public:
    OrthographicCamera(const AnimatedTransform &camToWorld, const Bounds2f &screenWindow,
                       Float shutterOpen, Float shutterClose, Float lensRadius, Float focalDist,
                       Film *film, const Medium *medium)
        : ProjectiveCamera(camToWorld, Transform::orthographic(0, 1), screenWindow, shutterOpen,
                           shutterClose, lensRadius, focalDist, film, medium)
    {
        dxCam = rasterToCamera(Vector3f(1, 0, 0));
        dyCam = rasterToCamera(Vector3f(0, 1, 0));
    }

    Float generateRay(const CameraSample &sample, Ray *ray) const;
    Float generateRayDifferential(const CameraSample &sample, RayDifferential *ray) const;

private:
    Vector3f dxCam, dyCam;
};

#endif // CAMERA_ORTHOGRAPHIC
