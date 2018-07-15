#ifndef CAMERA_ENVIRONMENT
#define CAMERA_ENVIRONMENT

#include "core/camera.h"

class EnvironmentCamera : public Camera {
public:
    EnvironmentCamera(const AnimatedTransform &camToWorld, Float shutterOpen, Float shutterClose,
                      Film *film, const Medium *medium)
        : Camera(camToWorld, shutterOpen, shutterClose, film, medium) {}

    static EnvironmentCamera * create(const ParamSet &params, const AnimatedTransform &cam2world,
                                       Film *film, const Medium *medium);

    Float generateRay(const CameraSample &sample, Ray *ray) const;
};

#endif // CAMERA_ENVIRONMENT
