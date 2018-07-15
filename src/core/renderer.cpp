#include "api.h"
#include "core/renderer.h"
#include "core/paramset.h"

#include "shapes/triangle.h"
#include "shapes/sphere.h"
#include "shapes/cylinder.h"
#include "shapes/disk.h"
#include "shapes/curve.h"
#include "accelerators/bvh.h"
#include "accelerators/kdtree.h"
#include "cameras/environment.h"
#include "cameras/ortho.h"
#include "cameras/perspective.h"

namespace Renderer {

vector<shared_ptr<Shape>> makeShapes(const string &name, const Transform *object2world,
                                     const Transform *world2object, bool reverseOrientation,
                                     const ParamSet &paramSet)
{
    vector<shared_ptr<Shape>> shapes;
    shared_ptr<Shape> s;

    if (name == "sphere")
        s = Sphere::create(object2world, world2object, reverseOrientation,paramSet);
    // Create remaining single _Shape_ types
    else if (name == "cylinder")
        s = Cylinder::create(object2world, world2object, reverseOrientation,
                             paramSet);
    else if (name == "disk")
        s = Disk::create(object2world, world2object, reverseOrientation, paramSet);
    if (s != nullptr) shapes.push_back(s);

    // Create multiple-_Shape_ types
    else if (name == "curve")
        shapes = Curve::create(object2world, world2object,
                               reverseOrientation, paramSet);
    else if (name == "trianglemesh") {
        shapes = TriangleMesh::create(object2world, world2object, reverseOrientation, paramSet,
                                      &*graphicsState.floatTextures);
        // TODO: Write to PLY files
    }
    else
        WARNING("Shape \"%s\" unknown.", name.c_str());
    return shapes;
}

shared_ptr<Primitive> makeAccelerator(const string &name, vector<shared_ptr<Primitive>> prims,
                                      const ParamSet &paramSet)
{
    shared_ptr<Primitive> accel;
    if (name == "bvh")
        accel = BVH::create(move(prims), paramSet);
    else if (name == "kdtree")
        accel = KDTree::create(move(prims), paramSet);
    else
        WARNING("Accelerator \"%s\" unknown.", name.c_str());
    paramSet.reportUnused();
    return accel;
}

Camera * makeCamera(const string &name, const ParamSet &paramSet, const TransformSet &cam2worldSet,
                    Float transformStart, Float transformEnd, Film *film)
{
    Camera *camera = nullptr;
    MediumInterface mediumInterface = graphicsState.createMediumInterface();
    static_assert(TransformSet::MAX_TRANSFORMS == 2, "TransformCache assumes only two transforms");
    Transform *cam2world[2] = {
        transformCache.lookup(cam2worldSet[0]),
        transformCache.lookup(cam2worldSet[1])
    };
    AnimatedTransform animatedCam2World(cam2world[0], transformStart, cam2world[1], transformEnd);
    if (name == "perspective")
        camera = PerspectiveCamera::create(paramSet, animatedCam2World, film, mediumInterface.outside);
    else if (name == "orthographic")
        camera = OrthographicCamera::create(paramSet, animatedCam2World, film, mediumInterface.outside);
    else if (name == "environment")
        camera = EnvironmentCamera::create(paramSet, animatedCam2World, film, mediumInterface.outside);
    else
        WARNING("Camera \"%s\" unknown.", name.c_str());
    paramSet.reportUnused();
    return camera;
}

};

STAT_MEMORY_COUNTER("Memory/TransformCache", transformCacheBytes);
STAT_PERCENT("Scene/TransformCache hits", nTransformCacheHits, nTransformCacheLookups);
STAT_INT_DISTRIB("Scene/Probes per TransformCache lookup", transformCacheProbes);

Transform * TransformCache::lookup(const Transform &t) {
    ++nTransformCacheLookups;

    unsigned long offset = hash(t) & (hashTable.size() - 1);
    unsigned step = 1;
    while (true) {
        // Keep looking until we find the Transform or determine that
        // it's not present.
        if (!hashTable[offset] || *hashTable[offset] == t)
            break;
        // Advance using quadratic probing.
        offset = (offset + step * step) & (hashTable.size() - 1);
        ++step;
    }
    REPORT_VALUE(transformCacheProbes, step);
    Transform *tCached = hashTable[offset];
    if (tCached)
        ++nTransformCacheHits;
    else {
        tCached = arena.alloc<Transform>();
        *tCached = t;
        insert(tCached);
    }
    return tCached;
}

void TransformCache::clear() {
    transformCacheBytes += arena.totalAllocated() + hashTable.size() * sizeof(Transform *);
    hashTable.resize(512);
    hashTable.clear();
    hashTableOccupancy = 0;
    arena.reset();
}

void TransformCache::insert(Transform *tNew) {
    if (++hashTableOccupancy == hashTable.size() / 2)
        grow();

    unsigned long offset = hash(*tNew) & (hashTable.size() - 1);
    unsigned step = 1;
    while (true) {
        if (hashTable[offset] == nullptr) {
            hashTable[offset] = tNew;
            return;
        }
        // Advance using quadratic probing.
        offset = (offset + step * step) & (hashTable.size() - 1);
        ++step;
    }
}

void TransformCache::grow() {
    vector<Transform *> newTable(2 * hashTable.size());
    LOG(INFO) << "Growing transform cache hash table to " << newTable.size();

    // Insert current elements into newTable.
    for (auto tEntry : hashTable) {
        if (!tEntry) continue;

        unsigned long offset = hash(*tEntry) & (newTable.size() - 1);
        unsigned step = 1;
        while (true) {
            if (newTable[offset] == nullptr) {
                newTable[offset] = tEntry;
                break;
            }
            // Advance using quadratic probing.
            offset = (offset + step * step) & (hashTable.size() - 1);
            ++step;
        }
    }

    swap(hashTable, newTable);
}

uint64_t TransformCache::hash(const Transform &t) {
    const char *ptr = (const char *)(&t.getMatrix());
    size_t size = sizeof(Matrix4x4);
    uint64_t hash = 14695981039346656037ull;
    while (size > 0) {
        hash ^= *ptr;
        hash *= 1099511628211ull;
        ++ptr;
        --size;
    }
    return hash;
}
