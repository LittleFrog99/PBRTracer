#ifndef CORE_CAMERA
#define CORE_CAMERA

#include "interaction.h"
#include "light.h"
#include "film.h"

struct CameraSample {
    Point2f pFilm;
    Point2f pLens;
    Float time;
};

class Camera {
public:
    Camera(const AnimatedTransform &camToWorld, Float shutterOpen, Float shutterClose,
           Film *film, const Medium *medium)
        : cameraToWorld(camToWorld), shutterOpen(shutterOpen), shutterClose(shutterClose),
          film(film), medium(medium) {}

    virtual Float generateRay(const CameraSample &sample, Ray *ray) const = 0;
    virtual Float generateRayDifferential(const CameraSample &sample, RayDifferential *rd) const;
    virtual Spectrum compute_We(const Ray &ray, Point2f *pRaster2 = nullptr) const;
    virtual void pdf_We(const Ray &ray, Float *pdfPos, Float *pdfDir) const;
    virtual Spectrum sample_Wi(const Interaction &ref, const Point2f &u,Vector3f *wi, Float *pdf,
                               Point2f *pRaster, VisibilityTester *vis) const;
    virtual ~Camera() { delete film; }

    AnimatedTransform cameraToWorld;
    const Float shutterOpen, shutterClose;
    Film *film;
    const Medium *medium;
};

inline ostream & operator << (ostream &os, const CameraSample &cs) {
    os << "[ pFilm: " << cs.pFilm << " , pLens: " << cs.pLens <<
        STRING_PRINTF(", time %f ]", cs.time);
    return os;
}

class ProjectiveCamera : public Camera {
public:
    ProjectiveCamera(const AnimatedTransform &camToWorld, const Transform &camToScreen,
                     const Bounds2f &screenWindow, Float shutterOpen,
                     Float shutterClose, Float lensr, Float focald, Film *film,
                     const Medium *medium)
        : Camera(camToWorld, shutterOpen, shutterClose, film, medium),
          cameraToScreen(camToScreen) {
        lensRadius = lensr;
        focalDistance = focald;
        screenToRaster =
            Transform::scale(film->fullResolution.x, film->fullResolution.y, 1) *
            Transform::scale(1 / (screenWindow.pMax.x - screenWindow.pMin.x),
                  1 / (screenWindow.pMin.y - screenWindow.pMax.y), 1) *
            Transform::translate(Vector3f(-screenWindow.pMin.x, -screenWindow.pMax.y, 0));
        rasterToScreen = screenToRaster.inverse();
        rasterToCamera = cameraToScreen.inverse() * rasterToScreen;
    }

protected:
    Transform cameraToScreen, rasterToCamera;
    Transform screenToRaster, rasterToScreen;
    Float lensRadius, focalDistance;
};

#endif // CORE_CAMERA
