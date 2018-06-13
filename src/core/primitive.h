#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "bounds.h"
#include "light.h"

class Primitive {
public:
    virtual ~Primitive();
    virtual Bounds3f worldBound() const = 0;
    virtual bool intersect(const Ray &r, SurfaceInteraction *) const = 0;
    virtual bool intersectP(const Ray &r) const = 0;
    virtual const AreaLight *getAreaLight() const = 0;
    virtual const Material *getMaterial() const = 0;
    virtual void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena,
                                        TransportMode mode, bool allowMultipleLobes) const = 0;
};

class GeometricPrimitive : public Primitive {
public:
    GeometricPrimitive(const shared_ptr<Shape> &shape, const shared_ptr<Material> &material,
                       const shared_ptr<AreaLight> &areaLight, const MediumInterface &mediumInterface);
    Bounds3f worldBound() const;
    bool intersect(const Ray &r, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &r) const;
    const AreaLight *getAreaLight() const;
    const Material *getMaterial() const;
    void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
                                bool allowMultipleLobes) const;

private:
    shared_ptr<Shape> shape;
    shared_ptr<Material> material;
    shared_ptr<AreaLight> areaLight;
    MediumInterface mediumInterface;
};

class TransformedPrimitive : public Primitive {
public:
    TransformedPrimitive(shared_ptr<Primitive> &primitive,
                         const AnimatedTransform &PrimitiveToWorld);
    bool intersect(const Ray &r, SurfaceInteraction *in) const;
    bool intersectP(const Ray &r) const;
    const AreaLight * getAreaLight() const { return nullptr; }
    const Material * getMaterial() const { return nullptr; }

    void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
                                bool allowMultipleLobes) const {
        LOG(FATAL) <<
            "TransformedPrimitive::ComputeScatteringFunctions() shouldn't be "
            "called";
    }

    Bounds3f worldBound() const {
        return primitiveToWorld.motionBounds(primitive->worldBound());
    }

private:
    shared_ptr<Primitive> primitive;
    const AnimatedTransform primitiveToWorld;
};

class Aggregate : public Primitive {
public:
    const AreaLight * getAreaLight() const;
    const Material * getMaterial() const;
    void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
                                bool allowMultipleLobes) const;
};
#endif // PRIMITIVE_H

