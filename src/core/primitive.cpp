#include "primitive.h"

bool GeometricPrimitive::intersect(const Ray &r, SurfaceInteraction *isect) const {
    float tHit;
    if (!shape->intersect(r, &tHit, isect)) return false;
    r.tMax = tHit;
    isect->primitive = this;
    // TODO: Initialize SurfaceInteraction::mediumInterface
    return true;
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
