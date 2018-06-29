#ifndef CORE_PRIMITIVE
#define CORE_PRIMITIVE

#include "bounds.h"
#include "light.h"
#include "shape.h"

class Primitive {
public:
    virtual ~Primitive() {}
    virtual Bounds3f worldBound() const = 0;
    virtual bool intersect(const Ray &r, SurfaceInteraction *) const = 0;
    virtual bool intersectP(const Ray &r) const = 0;
    virtual const AreaLight * getAreaLight() const = 0;
    virtual const Material * getMaterial() const = 0;
    virtual void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena,
                                        TransportMode mode, bool allowMultipleLobes) const = 0;
};

class GeometricPrimitive : public Primitive {
public:
    GeometricPrimitive(const shared_ptr<Shape> &shape, const shared_ptr<Material> &material,
                       const shared_ptr<AreaLight> &areaLight, const MediumInterface &mediumInterface)
        : shape(shape), material(material), areaLight(areaLight), mediumInterface(mediumInterface) {}

    Bounds3f worldBound() const;
    bool intersect(const Ray &r, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &r) const { return shape->intersectP(r); }
    const AreaLight * getAreaLight() const { return areaLight.get(); }
    const Material * getMaterial() const { return material.get(); }

    void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
                                bool allowMultipleLobes) const {
        if (material) material->computeScatteringFuncs(isect, arena, mode, allowMultipleLobes);
    }

private:
    shared_ptr<Shape> shape;
    shared_ptr<Material> material;
    shared_ptr<AreaLight> areaLight;
    MediumInterface mediumInterface;
};

class TransformedPrimitive : public Primitive {
public:
    TransformedPrimitive(shared_ptr<Primitive> &primitive, const AnimatedTransform &primToWorld)
        : primitive(primitive), primitiveToWorld(primToWorld) {}

    bool intersect(const Ray &r, SurfaceInteraction *in) const;
    bool intersectP(const Ray &r) const;

    const AreaLight * getAreaLight() const {
        LOG(FATAL) << "TransformedPrimitive::getAreaLight() shouldn't be called";
    }

    const Material * getMaterial() const {
        LOG(FATAL) << "TransformedPrimitive::getMaterial() shouldn't be called";
    }

    void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
                                bool allowMultipleLobes) const {
        LOG(FATAL) << "TransformedPrimitive::computeScatteringFuncs() shouldn't be called";
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
    const AreaLight * getAreaLight() const {
        LOG(FATAL) << "Aggregate::getAreaLight() shouldn't be called";
    }

    const Material * getMaterial() const {
        LOG(FATAL) << "Aggregate::getMaterial() shouldn't be called";
    }

    void computeScatteringFuncs(SurfaceInteraction *isect, MemoryArena &arena, TransportMode mode,
                                bool allowMultipleLobes) const {
        LOG(FATAL) << "Aggregate::computeScatteringFuncs() shouldn't be called";
    }
};
#endif // CORE_PRIMITIVE

