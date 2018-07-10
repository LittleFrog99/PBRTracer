#ifndef CAMERA_ENVIRONMENT
#define CAMERA_ENVIRONMENT

#include "core/camera.h"

class EnvironmentCamera : public Camera {
public:
    EnvironmentCamera(const AnimatedTransform &camToWorld, Float shutterOpen, Float shutterClose,
                      Film *film, Medium *medium)
        : Camera(camToWorld, shutterOpen, shutterClose, film, medium) {}

    Float generateRay(const CameraSample &sample, Ray *ray) const;
};

#endif // CAMERA_ENVIRONMENT
