#ifndef CAMERA_H
#define CAMERA_H

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
    Camera(const AnimatedTransform &CameraToWorld, Float shutterOpen,
           Float shutterClose, Film *film, const Medium *medium);
    virtual ~Camera();
    virtual Float genRay(const CameraSample &sample, Ray *ray) const = 0;
    virtual Float genRayDifferential(const CameraSample &sample, RayDifferential *rd) const;
    virtual Spectrum compute_We(const Ray &ray, Point2f *pRaster2 = nullptr) const;
    virtual void Pdf_We(const Ray &ray, Float *pdfPos, Float *pdfDir) const;
    virtual Spectrum Sample_Wi(const Interaction &ref, const Point2f &u,
                               Vector3f *wi, Float *pdf, Point2f *pRaster,
                               VisibilityTester *vis) const;

    // Camera Public Data
    AnimatedTransform CameraToWorld;
    const Float shutterOpen, shutterClose;
    Film *film;
    const Medium *medium;
};

inline ostream & operator << (ostream &os, const CameraSample &cs) {
    os << "[ pFilm: " << cs.pFilm << " , pLens: " << cs.pLens <<
        StringPrint::printf(", time %f ]", cs.time);
    return os;
}

class ProjectiveCamera : public Camera {
  public:
    // ProjectiveCamera Public Methods
    ProjectiveCamera(const AnimatedTransform &CameraToWorld,
                     const Transform &CameraToScreen,
                     const Bounds2f &screenWindow, Float shutterOpen,
                     Float shutterClose, Float lensr, Float focald, Film *film,
                     const Medium *medium)
        : Camera(CameraToWorld, shutterOpen, shutterClose, film, medium),
          cameraToScreen(CameraToScreen) {
        // Initialize depth of field parameters
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

#endif // CAMERA_H
