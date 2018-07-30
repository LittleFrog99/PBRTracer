#include "projection.h"
#include "imageio.h"
#include "paramset.h"

ProjectionLight::ProjectionLight(const Transform &lightToWorld, const MediumInterface &interface, const Spectrum &I,
                const string &texname, float fov)
    : PointLight(lightToWorld, interface, I)
{
    // Create Mipmap
    Point2i resolution;
    auto texels = ImageIO::readImage(texname, &resolution);
    if (texels)
        projMap.reset(new Mipmap<RGBSpectrum>(resolution, texels.get()));

    // Initialize projection matrix
    float aspect = projMap ? float(resolution.x) / float(resolution.y) : 1.0f;
    screenBounds = (aspect > 1) ? Bounds2f(Point2f(-aspect, -1), Point2f(aspect, 1))
                                : Bounds2f(Point2f(-1, -1 / aspect), Point2f(1, 1 / aspect));
    near = 1e-3f; far = 1e30f;
    lightProj = Transform::perspective(fov, near, far);

    // Compute cosine of cones surrounding projection directions
    float opposite = tan(radians(fov) / 2.0f);
    float tanDiag = opposite * sqrtf(1.0f + 1.0f / SQ(aspect));
    cosTotalWidth = cos(atan(tanDiag));
}

Spectrum ProjectionLight::projection(const Vector3f &w) const {
    Vector3f wl = worldToLight(w);
    if (wl.z < near) return 0;
    Point3f p = lightProj(Point3f(wl));
    if (!inside(Point2f(p.x, p.y), screenBounds)) return 0.0f;
    if (!projMap) return 1;
    Point2f st = Point2f(screenBounds.offset(Point2f(p.x, p.y)));
    return projMap->lookup(st);
}

shared_ptr<ProjectionLight> ProjectionLight::create(const Transform &light2world,const Medium *medium,
                                                    const ParamSet &paramSet)
{
    Spectrum I = paramSet.findOneSpectrum("I", Spectrum(1.0));
    Spectrum sc = paramSet.findOneSpectrum("scale", Spectrum(1.0));
    float fov = paramSet.findOneFloat("fov", 45.);
    string texname = paramSet.findOneFilename("mapname", "");
    return make_shared<ProjectionLight>(light2world, medium, I * sc, texname, fov);
}
