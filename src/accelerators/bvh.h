#ifndef ACCELERATOR_BVH
#define ACCELERATOR_BVH

#include "core/primitive.h"

// BVHAccel Declarations
class BVH : public Aggregate {
public:
    enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

    BVH(vector<shared_ptr<Primitive>> p,
             int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::SAH);
    static shared_ptr<BVH> create(vector<shared_ptr<Primitive>> prims, const ParamSet &ps);
    Bounds3f worldBound() const;
    ~BVH();
    bool intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &ray) const;

private:
    struct BVHBuildNode;
    struct BVHPrimitiveInfo;
    struct MortonPrimitive;
    struct LinearBVHNode;
    struct LBVHTreelet;

    BVHBuildNode *recursiveBuild(MemoryArena &arena, vector<BVHPrimitiveInfo> &primitiveInfo,
                                 int start, int end, int *totalNodes,
                                 vector<shared_ptr<Primitive>> &orderedPrims);
    BVHBuildNode *HLBVHBuild(MemoryArena &arena, const vector<BVHPrimitiveInfo> &primitiveInfo,
                             int *totalNodes, vector<shared_ptr<Primitive>> &orderedPrims) const;
    BVHBuildNode *emitLBVH(BVHBuildNode *&buildNodes,const vector<BVHPrimitiveInfo> &primitiveInfo,
                           MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
                           vector<shared_ptr<Primitive>> &orderedPrims,
                           atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BVHBuildNode *buildUpperSAH(MemoryArena &arena, vector<BVHBuildNode *> &treeletRoots,
                                int start, int end, int *totalNodes) const;
    int flattenBVHTree(BVHBuildNode *node, int *offset);

    static void radixSort(vector<MortonPrimitive> *v);

    inline static uint32_t encodeMorton3(const Vector3f &v) {
        return (leftshift3(v.z) << 2) | (leftshift3(v.y) << 1) | leftshift3(v.x);
    }

    inline static uint32_t leftshift3(uint32_t x) {
        CHECK_LE(x, (1 << 10));
        if (x == (1 << 10)) --x;
        x = (x | (x << 16)) & 0b00000011000000000000000011111111;
        x = (x | (x << 8)) & 0b00000011000000001111000000001111;
        x = (x | (x << 4)) & 0b00000011000011000011000011000011;
        x = (x | (x << 2)) & 0b00001001001001001001001001001001;
        return x;
    }

    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    vector<shared_ptr<Primitive>> primitives;
    LinearBVHNode *nodes = nullptr;
};


#endif // ACCELERATOR_BVH
