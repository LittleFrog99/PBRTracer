#include "perspective.h"
#include "core/sampling.h"
#include "paramset.h"

PerspectiveCamera::PerspectiveCamera(const AnimatedTransform &camToWorld, const Bounds2f &screenWindow,
                                     float shutterOpen, float shutterClose, float lensRadius,
                                     float focalDist, float fov, Film *film, const Medium *medium)
    : ProjectiveCamera(camToWorld, Transform::perspective(fov, 1e-2f, 1e3f), screenWindow,
                       shutterOpen, shutterClose, lensRadius, focalDist, film, medium)
{
    dxCam = rasterToCamera(Point3f(1, 0, 0)) - rasterToCamera(Point3f());
    dyCam = rasterToCamera(Point3f(0, 1, 0)) - rasterToCamera(Point3f());

    // Compute image plane bounds at z = 1
    Point2i res = film->fullResolution;
    Point3f pMin = rasterToCamera(Point3f());
    Point3f pMax = rasterToCamera(Point3f(res.x, res.y, 0));
    pMin /= pMin.z; pMax /= pMax.z;
    A = abs(Bounds2f(Point2f(pMin), Point2f(pMax)).area());
}

float PerspectiveCamera::generateRay(const CameraSample &sample, Ray *ray) const {
    auto pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    auto pCamera = rasterToCamera(pFilm);
    *ray = Ray(Point3f(), normalize(Vector3f(pCamera)));

    // Modify ray for depth of field
    if (lensRadius > 0) {
        auto pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);
        float ft = focalDistance / ray->d.z;
        auto pFocus = (*ray)(ft);
        ray->o = Point3f(pLens.x, pLens.y, 0);
        ray->d = normalize(pFocus - ray->o);
    }

    ray->time = lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = cameraToWorld(*ray);
    return 1;
}

float PerspectiveCamera::generateRayDifferential(const CameraSample &sample, RayDifferential *ray) const {
    auto pFilm = Point3f(sample.pFilm.x, sample.pFilm.y, 0);
    auto pCam = rasterToCamera(pFilm);
    *ray = Ray(Point3f(), normalize(Vector3f(pCam)));

    // Modify ray for depth of field
    if (lensRadius > 0) {
        auto pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);
        float ft = focalDistance / ray->d.z;
        auto pFocus = (*ray)(ft);
        ray->o = Point3f(pLens.x, pLens.y, 0);
        ray->d = normalize(pFocus - ray->o);
    }

    // Compute offset rays for ray differentials
    if (lensRadius > 0) { // accounting for lens
        Point2f pLens = lensRadius * Sampling::concentricSampleDisk(sample.pLens);

        Vector3f dx = normalize(Vector3f(pCam + dxCam));
        float ft = focalDistance / dx.z;
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

Spectrum PerspectiveCamera::compute_We(const Ray &ray, Point2f *pRaster2) const {
    // Interpolate camera matrix and check ray direction
    Transform c2w;
    cameraToWorld.interpolate(ray.time, &c2w);
    float cosTheta = dot(ray.d, c2w(Vector3f(0, 0, 1)));
    if (cosTheta <= 0) return 0;

    // Map (p, ω) onto the raster grid
    Point3f pFocus = ray((lensRadius > 0 ? focalDistance : 1) / cosTheta);
    Point3f pRaster = (rasterToCamera.inverse() * c2w.inverse())(pFocus);
    if (pRaster2) *pRaster2 = Point2f(pRaster);
    Bounds2f sampleBounds = Bounds2f(film->getSampleBounds());
    if (!insideExclusive(Point2f(pRaster), sampleBounds)) return 0;

    // Compute importance for point on image plane
    float lensArea = lensRadius == 0 ? 1.0f : (PI * SQ(lensRadius));
    return 1.0f / (A * lensArea * QUAD(cosTheta));
}

void PerspectiveCamera::pdf_We(const Ray &ray, float *pdfPos, float *pdfDir) const {
    // Interpolate camera matrix and check ray direction
    Transform c2w;
    cameraToWorld.interpolate(ray.time, &c2w);
    float cosTheta = dot(ray.d, c2w(Vector3f(0, 0, 1)));
    if (cosTheta <= 0) {
        *pdfPos = 0; *pdfDir = 0;
        return;
    }

    // Map (p, ω) onto the raster grid
    Point3f pFocus = ray((lensRadius > 0 ? focalDistance : 1) / cosTheta);
    Point3f pRaster = (rasterToCamera.inverse() * c2w.inverse())(pFocus);
    Bounds2f sampleBounds = Bounds2f(film->getSampleBounds());
    if (!insideExclusive(Point2f(pRaster), sampleBounds)) {
        *pdfPos = 0; *pdfDir = 0;
        return;
    }

    // Compute importance for point on image plane
    float lensArea = lensRadius == 0 ? 1.0f : (PI * SQ(lensRadius));
    *pdfPos = 1 / lensArea;
    *pdfDir = 1 / (A * CUB(cosTheta));
    return;
}

Spectrum PerspectiveCamera::sample_Wi(const Interaction &ref, const Point2f &u, Vector3f *wi, float *pdf,
                                      Point2f *pRaster, VisibilityTester *vis) const
{
    // Uniform sample a lens interaction _lensIntr_
    Point2 pLens = lensRadius * Sampling::concentricSampleDisk(u);
    Point3f pLensWorld = cameraToWorld(ref.time, Point3f(pLens.x, pLens.y, 0));
    Interaction lensIntr(pLensWorld, ref.time, medium);
    lensIntr.n = Normal3f(cameraToWorld(ref.time, Vector3f(0, 0, 1)));

    // Populate arguments and compute the importance value
    *vis = VisibilityTester(ref, lensIntr);
    *wi = lensIntr.p - ref.p;
    float dist = wi->length();
    *wi /= dist;
    float lensArea = lensRadius == 0 ? 1.0f : (PI * SQ(lensRadius));
    *pdf = SQ(dist) / (absDot(lensIntr.n, *wi) * lensArea);
    return compute_We(lensIntr.spawnRay(-*wi), pRaster);
}

PerspectiveCamera * PerspectiveCamera::create(const ParamSet &params, const AnimatedTransform &cam2world,
                                              Film *film, const Medium *medium)
{
    float shutteropen = params.findOneFloat("shutteropen", 0.f);
    float shutterclose = params.findOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        WARNING("Shutter close time [%f] < shutter open [%f].  Swapping them.", shutterclose, shutteropen);
        swap(shutterclose, shutteropen);
    }
    float lensradius = params.findOneFloat("lensradius", 0.f);
    float focaldistance = params.findOneFloat("focaldistance", 1e6);
    float frame = params.findOneFloat(
        "frameaspectratio",
        float(film->fullResolution.x) / float(film->fullResolution.y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    int swi;
    const float *sw = params.findFloat("screenwindow", &swi);
    if (sw) {
        if (swi == 4) {
            screen.pMin.x = sw[0];
            screen.pMax.x = sw[1];
            screen.pMin.y = sw[2];
            screen.pMax.y = sw[3];
        } else
            ERROR("\"screenwindow\" should have four values");
    }
    float fov = params.findOneFloat("fov", 90.);
    float halffov = params.findOneFloat("halffov", -1.f);
    if (halffov > 0.f)
        // hack for structure synth, which exports half of the full fov
        fov = 2.f * halffov;
    return new PerspectiveCamera(cam2world, screen, shutteropen, shutterclose, lensradius, focaldistance,
                                 fov, film, medium);
}
