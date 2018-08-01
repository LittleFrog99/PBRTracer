#include "primitive.h"
#include "stats.h"

STAT_MEMORY_COUNTER("Memory/Primitives", primitiveMemory);

GeometricPrimitive::GeometricPrimitive(const shared_ptr<Shape> &shape, const shared_ptr<Material> &material,
                                       const shared_ptr<AreaLight> &areaLight, const MediumInterface &interface)
    : shape(shape), material(material), areaLight(areaLight), mediumInterface(interface)
{
    primitiveMemory += sizeof(*this);
}

bool GeometricPrimitive::intersect(const Ray &r, SurfaceInteraction *isect) const {
    float tHit;
    if (!shape->intersect(r, &tHit, isect)) return false;
    r.tMax = tHit;
    isect->primitive = this;
    // Initialize SurfaceInteraction::mediumInterface
    if (mediumInterface.isMediumTransition())
        isect->mediumInterface = mediumInterface;
    else
        isect->mediumInterface = MediumInterface(r.medium);
    return true;
}

TransformedPrimitive::TransformedPrimitive(shared_ptr<Primitive> &primitive,
                                           const AnimatedTransform &primToWorld)
    : primitive(primitive), primitiveToWorld(primToWorld)
{
    primitiveMemory += sizeof(*this);
}

bool TransformedPrimitive::intersect(const Ray &worldRay, SurfaceInteraction *isect) const {
    Transform interpPrimToWorld;
    primitiveToWorld.interpolate(worldRay.time, &interpPrimToWorld); // interpolate first for consistency
    auto primRay = interpPrimToWorld.inverse()(worldRay); // world space to primitive space
    if (!primitive->intersect(primRay, isect)) return false;
    worldRay.tMax = primRay.tMax;
    if (!interpPrimToWorld.isIdentity())
        *isect = interpPrimToWorld(*isect);
    return true;
}

bool TransformedPrimitive::intersectP(const Ray &worldRay) const {
    Transform interpPrimToWorld;
    primitiveToWorld.interpolate(worldRay.time, &interpPrimToWorld);
    auto primRay = interpPrimToWorld.inverse()(worldRay);
    return primitive->intersectP(primRay);
}
