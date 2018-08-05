#include "shape.h"
#include "stats.h"

STAT_COUNTER("Scene/Shapes created", nShapesCreated);

Shape::Shape(const Transform *ObjectToWorld, const Transform *WorldToObject, bool revOrient)
    : objectToWorld(ObjectToWorld), worldToObject(WorldToObject), reverseOrientation(revOrient),
    transformSwapsHandedness(objectToWorld->swapsHandedness())
{
    ++nShapesCreated;
}

Interaction Shape::sample(const Interaction &ref, const Point2f &u, float *pdf) const {
    Interaction intr = sample(u, pdf);
    Vector3f wi = intr.p - ref.p;
    if (wi.lengthSq() == 0)
        *pdf = 0;
    else {
        wi = normalize(wi);
        // Convert from area measure to solid angle measure.
        *pdf *= distanceSq(ref.p, intr.p) / absDot(intr.n, -wi);
        if (isinf(*pdf)) *pdf = 0.f;
    }
    return intr;
}

float Shape::pdf(const Interaction &ref, const Vector3f &wi) const {
    // Intersect sample ray with area light geometry
    Ray ray = ref.spawnRay(wi);
    float tHit;
    SurfaceInteraction isectLight;
    if (!intersect(ray, &tHit, &isectLight, false))
        return 0;

    // Convert light sample weigh to solid angle measure
    return distanceSq(ref.p, isectLight.p) / (absDot(isectLight.n, -wi) * area());
}
