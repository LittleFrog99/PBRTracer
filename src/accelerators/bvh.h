#ifndef ACCELERATOR_BVH
#define ACCELERATOR_BVH

#include "core/primitive.h"

struct BVHBuildNode;

// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;
struct MortonPrimitive;
struct LinearBVHNode;

// BVHAccel Declarations
class BVH : public Aggregate {
  public:
    // BVHAccel Public Types
    enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

    // BVHAccel Public Methods
    BVH(std::vector<std::shared_ptr<Primitive>> p,
             int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::SAH);
    static shared_ptr<BVH> create(vector<shared_ptr<Primitive>> prims, const ParamSet &ps);
    Bounds3f worldBound() const;
    ~BVH();
    bool intersect(const Ray &ray, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &ray) const;

  private:
    // BVHAccel Private Methods
    BVHBuildNode *recursiveBuild(
        MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int start, int end, int *totalNodes,
        std::vector<std::shared_ptr<Primitive>> &orderedPrims);
    BVHBuildNode *HLBVHBuild(
        MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes,
        std::vector<std::shared_ptr<Primitive>> &orderedPrims) const;
    BVHBuildNode *emitLBVH(
        BVHBuildNode *&buildNodes,
        const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
        std::vector<std::shared_ptr<Primitive>> &orderedPrims,
        std::atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BVHBuildNode *buildUpperSAH(MemoryArena &arena,
                                std::vector<BVHBuildNode *> &treeletRoots,
                                int start, int end, int *totalNodes) const;
    int flattenBVHTree(BVHBuildNode *node, int *offset);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<std::shared_ptr<Primitive>> primitives;
    LinearBVHNode *nodes = nullptr;
};


#endif // ACCELERATOR_BVH
