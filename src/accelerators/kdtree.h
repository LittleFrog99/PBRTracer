#ifndef UTILITY_KDTREE
#define UTILITY_KDTREE

#include "core/primitive.h"

class KDTree : public Aggregate {
public:
    KDTree(vector<shared_ptr<Primitive>> &p, int isectCost, int travCost, Float emptyBonus, int maxPrims,
           int maxDepth);
    Bounds3f worldBound() const { return bounds; }
    bool intersect(const Ray &r, SurfaceInteraction *isect) const;
    bool intersectP(const Ray &r) const;

private:
    struct Node;
    struct BoundEdge;
    enum class EdgeType { Start, End };

    void buildTree(int nodeNum, const Bounds3f &nodeBounds, const vector<Bounds3f> &allPrimBounds,
                   int *primNums, int nPrims, int depth, const unique_ptr<BoundEdge[]> edges[3],
                   int *prims0, int *prims1, int badRefines);

    const int isectCost, travCost, maxPrims;
    const Float emptyBonus;
    vector<shared_ptr<Primitive>> primitives;
    vector<int> primIndices;
    Bounds3f bounds;
    Node *nodes;
    int nAllocatedNodes, nextFreeNode;
};

#endif // UTILITY_KDTREE
