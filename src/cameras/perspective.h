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

    virtual Spectrum compute_We(const Ray &ray, Point2f *pRaster2 = nullptr) const;
    virtual void pdf_We(const Ray &ray, float *pdfPos, float *pdfDir) const;
    virtual Spectrum sample_Wi(const Interaction &ref, const Point2f &u,Vector3f *wi, float *pdf,
                               Point2f *pRaster, VisibilityTester *vis) const;

private:
    Vector3f dxCam, dyCam;
    float A;
};

#endif // CAMERA_PERSPECTIVE
