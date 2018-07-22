#ifndef CAMERA_PERSPECTIVE
#define CAMERA_PERSPECTIVE

#include "core/camera.h"

class PerspectiveCamera : public ProjectiveCamera {
public:
    PerspectiveCamera(const AnimatedTransform &camToWorld, const Bounds2f &screenWindow,
                      float shutterOpen, float shutterClose, float lensRadius, float focalDist, float fov,
                      Film *film, const Medium *medium);
    static PerspectiveCamera * create(const ParamSet &params, const AnimatedTransform &cam2world, Film *film,
                                      const Medium *medium);
    float generateRay(const CameraSample &sample, Ray *ray) const;
    float generateRayDifferential(const CameraSample &sample, RayDifferential *ray) const;

private:
    Vector3f dxCam, dyCam;
};

#endif // CAMERA_PERSPECTIVE
