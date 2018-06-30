#ifndef ACCELERATOR_BVH
#define ACCELERATOR_BVH

#include "core/primitive.h"

class BVH : public Aggregate {
public:
    enum class SplitMethod {
        SAH, // Surface Area Heuristic
        HLBVH, // Linear Bounding Volume Hierachies
        Middle, // Divide space in the middle of centroid bounds
        EqualCounts // Divide space into two parts with equal number of primitives
    };

    BVH(vector<shared_ptr<Primitive>> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::SAH);
    Bounds3f worldBound() const;
    bool intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &ray) const;
    ~BVH();

private:
    struct BuildNode;
    struct PrimitiveInfo;
    struct MortonPrimitive;
    struct LinearNode;
    struct LBVHTreelet;

    BuildNode *recursiveBuild(MemoryArena &arena, vector<PrimitiveInfo> &primitiveInfo,
                              int start, int end, int *totalNodes,
                              vector<shared_ptr<Primitive>> &orderedPrims);
    BuildNode *HLBVHBuild(MemoryArena &arena, const vector<PrimitiveInfo> &primitiveInfo,
                          int *totalNodes, vector<shared_ptr<Primitive>> &orderedPrims) const;
    BuildNode *emitLBVH(BuildNode *&buildNodes, const vector<PrimitiveInfo> &primitiveInfo,
                        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
                        vector<shared_ptr<Primitive>> &orderedPrims,
                        atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BuildNode *buildUpperSAH(MemoryArena &arena, vector<BuildNode *> &treeletRoots,
                             int start, int end, int *totalNodes) const;
    int flattenBVHTree(BuildNode *node, int *offset);

    static uint32_t leftShift3(uint32_t x);
    static uint32_t encodeMorton(const Vector3f &v);
    static void radixSort(vector<MortonPrimitive> *v);

    const int maxPrimsInNode;
    vector<shared_ptr<Primitive>> primitives;
    const SplitMethod splitMethod;
    LinearNode *nodes = nullptr;
};


#endif // ACCELERATOR_BVH
