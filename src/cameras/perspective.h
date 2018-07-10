#ifndef CAMERA_PERSPECTIVE
#define CAMERA_PERSPECTIVE

#include "core/camera.h"

class PerspectiveCamera : public ProjectiveCamera {
public:
    PerspectiveCamera(const AnimatedTransform &camToWorld, const Bounds2f &screenWindow,
                      Float shutterOpen, Float shutterClose, Float lensRadius, Float focalDist, Float fov,
                      Film *film, const Medium *medium);
    Float generateRay(const CameraSample &sample, Ray *ray) const;
    Float generateRayDifferential(const CameraSample &sample, RayDifferential *ray) const;

private:
    Vector3f dxCam, dyCam;
};

#endif // CAMERA_PERSPECTIVE
