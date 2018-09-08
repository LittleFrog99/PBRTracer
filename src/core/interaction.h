#ifndef CORE_INTERACTION
#define CORE_INTERACTION

#include "core/ray.h"
#include "core/medium.h"
#include "core/material.h"
#include "memory.h"

class Shape;
class BSDF;
class Primitive;
class BSSRDF;

struct Interaction {
    Interaction() : time(0) {}

    Interaction(const Point3f &p, const Normal3f &n, const Vector3f &pError,
                const Vector3f &wo, float time, const MediumInterface &mediumInterface)
        : p(p), time(time), pError(pError), wo(normalize(wo)), n(n),
          mediumInterface(mediumInterface) {} // for geometry intersection

    Interaction(const Point3f &p, const Vector3f &wo, float time,
                const MediumInterface &mediumInterface)
        : p(p), time(time), wo(wo), mediumInterface(mediumInterface) {}

    Interaction(const Point3f &p, float time, const MediumInterface &mediumInterface)
        : p(p), time(time), mediumInterface(mediumInterface) {} // for light sample point

    bool isSurfaceInteraction() const { return n != Normal3f(); }

    Ray spawnRay(const Vector3f &d) const {
        Point3f o = offsetRayOrigin(p, pError, n, d);
        return Ray(o, d, INFINITY, time, getMedium(d));
    }

    Ray spawnRayTo(const Point3f &p2) const {
        Point3f origin = offsetRayOrigin(p, pError, n, p2 - p);
        Vector3f d = p2 - p;
        return Ray(origin, d, 1 - SHADOW_EPSILON, time, getMedium(d));
    }

    Ray spawnRayTo(const Interaction &it) const {
        Point3f origin = offsetRayOrigin(p, pError, n, it.p - p);
        Point3f target = offsetRayOrigin(it.p, it.pError, it.n, origin - it.p);
        Vector3f d = target - origin;
        return Ray(origin, d, 1 - SHADOW_EPSILON, time, getMedium(d));
    }

    bool isMediumInteraction() const { return !isSurfaceInteraction(); }

    const Medium * getMedium(const Vector3f &w) const {
        return dot(w, n) > 0 ? mediumInterface.outside : mediumInterface.inside;
    }

    const Medium * getMedium() const {
        CHECK_EQ(mediumInterface.inside, mediumInterface.outside);
        return mediumInterface.inside;
    }

    Point3f p;
    float time;
    Vector3f pError;
    Vector3f wo;
    Normal3f n;
    MediumInterface mediumInterface;
};

struct MediumInteraction : public Interaction {
    MediumInteraction() : phase(nullptr) {}
    MediumInteraction(const Point3f &p, const Vector3f &wo, float time, const Medium *medium,
                      const PhaseFunction *phase)
        : Interaction(p, wo, time, medium), phase(phase) {}

    bool isValid() const { return phase != nullptr; }

    const PhaseFunction *phase;
};

struct SurfaceInteraction : public Interaction {

    SurfaceInteraction() {}
    SurfaceInteraction(const Point3f &p, const Vector3f &pError, const Point2f &uv, const Vector3f &wo,
                       const Vector3f &dpdu, const Vector3f &dpdv, const Normal3f &dndu, const Normal3f &dndv,
                       float time, const Shape *sh);
    void setShadingGeometry(const Vector3f &dpdu, const Vector3f &dpdv, const Normal3f &dndu, const Normal3f &dndv,
                            bool orientationIsAuthoritative);
    void computeScatteringFunctions(const RayDifferential &ray, MemoryArena &arena, bool allowMultipleLobes = false,
                                    TransportMode mode = TransportMode::Radiance);
    void computeDifferentials(const RayDifferential &r) const;
    Spectrum compute_Le(const Vector3f &w) const; // is calculated when intersecting an emissive surface
    bool isEmissive() const;

    Point2f uv;
    Vector3f dpdu, dpdv;
    Normal3f dndu, dndv;
    const Shape *shape = nullptr;
    struct Shading {
        Normal3f n;
        Vector3f dpdu, dpdv;
        Normal3f dndu, dndv;
    } shading;
    const Primitive *primitive = nullptr;
    BSDF *bsdf = nullptr;
    BSSRDF *bssrdf = nullptr;
    mutable Vector3f dpdx, dpdy;
    mutable float dudx = 0, dvdx = 0, dudy = 0, dvdy = 0;

    // int faceIndex = 0;
};

#endif // CORE_INTERACTION
