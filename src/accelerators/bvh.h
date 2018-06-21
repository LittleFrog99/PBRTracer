#ifndef BVH_H
#define BVH_H

#include "core/primitive.h"

struct BVHBuildNode;
struct BVHPrimitiveInfo;
struct MortonPrimitive;
struct LinearBVHNode;

class BVH : public Aggregate {
public:
    enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

    BVH(vector<shared_ptr<Primitive>> p, int maxPrimsInNode = 1,
        SplitMethod splitMethod = SplitMethod::SAH);
    Bounds3f worldBound() const;
    ~BVH();
    bool intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &ray) const;

private:
    BVHBuildNode *recursiveBuild(
        MemoryArena &arena, vector<BVHPrimitiveInfo> &primitiveInfo,
        int start, int end, int *totalNodes,
        vector<shared_ptr<Primitive>> &orderedPrims);
    BVHBuildNode *HLBVHBuild(
        MemoryArena &arena, const vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes,
        vector<shared_ptr<Primitive>> &orderedPrims) const;
    BVHBuildNode *emitLBVH(
        BVHBuildNode *&buildNodes,
        const vector<BVHPrimitiveInfo> &primitiveInfo,
        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
        vector<shared_ptr<Primitive>> &orderedPrims,
        atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BVHBuildNode *buildUpperSAH(MemoryArena &arena,
                                vector<BVHBuildNode *> &treeletRoots,
                                int start, int end, int *totalNodes) const;
    int flattenBVHTree(BVHBuildNode *node, int *offset);

    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    vector<shared_ptr<Primitive>> primitives;
    LinearBVHNode *nodes = nullptr;
};


#endif // BVH_H
