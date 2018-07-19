#include "kdtree.h"
#include "paramset.h"

struct KDTree::Node {
    void initLeaf(int *primNums, int np, vector<int> *primIndices) {
        flags = 3; // 0b11 indicates leaf node
        nPrims |= (np << 2); // nPrims occupies higher 30 bits
        if (np == 0) onePrim = 0;
        else if (np == 1) onePrim = primNums[0];
        else {
            primIndicesOffset = primIndices->size();
            for (int i = 0; i < np; i++)
                primIndices->push_back(primNums[i]);
        }
    }

    void initInterior(int axis, int ac, Float s) {
        split = s;
        flags = axis;
        child1 |= (ac << 2);
    }

    Float splitPos() const { return split; }
    Float nPrimitives() const { return nPrims >> 2; }
    int splitAxis() const { return flags & 3; } // 0b11
    bool isLeaf() const { return (flags & 3) == 3; }
    int aboveChild() const { return child1 >> 2; }

    union {
        Float split; // interior
        int onePrim; // leaf (only one primitive)
        int primIndicesOffset; // leaf (more than one)
    };
    union {
        int flags; // both
        int nPrims; // leaf
        int child1; // interior (the far child only)
    };
};

struct KDTree::BoundEdge {
    BoundEdge() {}
    BoundEdge(Float t, int primNum, bool starting)
        :t(t), primNum(primNum) {
        type = starting ? EdgeType::Start : EdgeType::End;
    }

    Float t;
    int primNum;
    EdgeType type;
};

KDTree::KDTree(vector<shared_ptr<Primitive>> &p, int isectCost, int travCost, Float emptyBonus,
               int maxPrims, int maxDepth)
    : isectCost(isectCost), travCost(travCost), maxPrims(maxPrims), emptyBonus(emptyBonus), primitives(p)
{
    nextFreeNode = nAllocatedNodes = 0;
    if (maxDepth <= 0) maxDepth = round(8 + 1.3f * log2Int(primitives.size()));
    // Compute bounds for kd-tree construction;
    vector<Bounds3f> primBounds;
    for (const auto &prim : primitives) {
        auto b = prim->worldBound();
        bounds = unionOf(bounds, b);
        primBounds.push_back(b);
    }

    // Allocate working memory for kd-tree construction
    unique_ptr<BoundEdge[]> edges[3];
    for (int dim = 0; dim < 3; dim++)
        edges[dim].reset(new BoundEdge[2 * primitives.size()]);
    unique_ptr<int[]> prims0(new int[primitives.size()]);
    unique_ptr<int[]> prims1(new int[(maxDepth + 1) * primitives.size()]);

    // Initialize _primNums_
    unique_ptr<int[]> primNums(new int[primitives.size()]);
    for (size_t i = 0; i < primitives.size(); i++)
        primNums[i] = i;

    // Start recursive construction
    buildTree(0, bounds, primBounds, primNums.get(), primitives.size(), maxDepth, edges,
              prims0.get(), prims1.get(), 0);
}

void KDTree::buildTree(int nodeNum, const Bounds3f &nodeBounds, const vector<Bounds3f> &allPrimBounds,
                       int *primNums, int nPrims, int depth, const unique_ptr<BoundEdge[]> edges[3],
                       int *prims0, int *prims1, int badRefines)
{
    // Get next free node from _nodes_
    if (nextFreeNode == nAllocatedNodes) {
        int nNewAllocatedNodes = max(2 * nAllocatedNodes, 512);
        auto *n = Memory::allocAligned<Node>(nNewAllocatedNodes);
        if (nAllocatedNodes > 0) {
            memcpy(n, nodes, nAllocatedNodes * sizeof(Node));
            Memory::freeAligned(nodes);
        }
        nodes = n;
        nAllocatedNodes = nNewAllocatedNodes;
    }
    ++nextFreeNode;

    // Initialize leaf node if termination criteria met
    if (nPrims <= maxPrims || depth == 0) {
        nodes[nodeNum].initLeaf(primNums, nPrims, &primIndices);
        return;
    }

    // Initialize interior node and continue recursion
    // Choose split axis
    int bestAxis = -1, bestOffset = -1;
    Float bestCost = INFINITY;
    Float oldCost = isectCost * Float(nPrims);
    Float totalSA = nodeBounds.surfaceArea();
    Float invTotalSA = 1.0 / totalSA;
    Vector3f d = nodeBounds.pMax - nodeBounds.pMin;
    int axis = nodeBounds.maxExtent();
    int retries = 0;

    retrySplit:
    for (int i = 0; i < nPrims; i++) {
        int pn = primNums[i];
        const auto &bounds = allPrimBounds[pn];
        edges[axis][2 * i] = BoundEdge(bounds.pMin[axis], pn, true);
        edges[axis][2 * i + 1] = BoundEdge(bounds.pMax[axis], pn, false);
    }
    sort(&edges[axis][0], &edges[axis][2 * nPrims],
            [] (const BoundEdge &e0, const BoundEdge &e1) {
        if (e0.t == e1.t) return e0.type < e1.type;
        return e0.t < e1.t;
    });

    // Compute cost of all splits for _axis_ to find best
    int nBelow = 0, nAbove = nPrims;
    for (int i = 0; i < 2 * nPrims; i++) {
        if (edges[axis][i].type == EdgeType::End) --nAbove;
        Float edgeT = edges[axis][i].t;
        if (edgeT > nodeBounds.pMin[axis] && edgeT < nodeBounds.pMax[axis]) {
            // Compute cost for split at ith edge
            int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
            Float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                 (edgeT - nodeBounds.pMin[axis]) * (d[otherAxis0] + d[otherAxis1]));
            Float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
                                 (nodeBounds.pMax[axis] - edgeT) * (d[otherAxis0] + d[otherAxis1]));
            Float pBelow = belowSA * invTotalSA;
            Float pAbove = aboveSA * invTotalSA;
            Float eb = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
            Float cost = travCost + isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestOffset = i;
            }
        }
        if (edges[axis][i].type == EdgeType::Start) ++nBelow;
    }

    // Create leaf if no good splits were found
    if (bestAxis == -1 && retries < 2) {
        ++retries;
        axis = (axis + 1) % 3;
        goto retrySplit;
    }
    if (bestCost > oldCost) ++badRefines; // later splits may give better refinement
    if ((bestCost > 4 * oldCost && nPrims < 16) || bestAxis == -1 || badRefines == 3) {
        nodes[nodeNum].initLeaf(primNums, nPrims, &primIndices);
        return;
    }

    // Classify primitives with respect to split
    int n0 = 0, n1 = 0;
    for (int i = 0; i < bestOffset; i++)
        if (edges[bestAxis][i].type == EdgeType::Start)
            prims0[n0++] = edges[bestAxis][i].primNum;
    for (int i = bestOffset + 1; i < 2 * nPrims; i++)
        if (edges[bestAxis][i].type == EdgeType::End)
            prims1[n1++] = edges[bestAxis][i].primNum;

    // Recursively initialize children bounds
    Float tSplit = edges[bestAxis][bestOffset].t;
    Bounds3f bounds0 = nodeBounds, bounds1 = nodeBounds;
    bounds0.pMax[bestAxis] = bounds1.pMin[bestAxis] = tSplit;
    buildTree(nodeNum + 1, bounds0, allPrimBounds, prims0, n0, depth - 1, edges, prims0, prims1 + nPrims,
              badRefines);
    int aboveChild = nextFreeNode;
    nodes[nodeNum].initInterior(bestAxis, aboveChild, tSplit);
    buildTree(aboveChild, bounds1, allPrimBounds, prims1, n1, depth - 1, edges, prims0, prims1 + nPrims,
              badRefines);
}

bool KDTree::intersect(const Ray &ray, SurfaceInteraction *isect) const {
    // Compute initial parametric range of ray inside kd-tree extent
    Float tMin, tMax;
    if (!bounds.intersectP(ray, &tMin, &tMax)) return false;

    // Prepare to traverse kd-tree for ray
    struct KDTodo {
        const Node *node;
        Float tMin, tMax;
    };
    Vector3f invDir(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
    constexpr int maxTodo = 64;
    KDTodo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    bool hit = false;
    const Node *node = &nodes[0];
    while (node) {
        if (ray.tMax < tMin) break; // definitely no closer hit
        if (!node->isLeaf()) { // interior
            // Compute parametric distance along ray to split plane
            int axis = node->splitAxis();
            Float tPlane = (node->splitPos() - ray.o[axis]) * invDir[axis];
            // Get node children pointers for ray
            const Node *firstChild, *secondChild;
            bool belowFirst = (ray.o[axis] < node->splitPos() ||
                              (ray.o[axis] == node->splitPos() && ray.d[axis] <= 0));
            if (belowFirst) {
                firstChild = node + 1;
                secondChild = &nodes[node->aboveChild()];
            } else {
                firstChild = &nodes[node->aboveChild()];
                secondChild = node + 1;
            }
            // Advance to next child node
            if (tPlane > tMax || tPlane <= 0)
                node = firstChild;
            else if (tPlane < tMin)
                node = secondChild;
            else {
                // Enqueue second child in todo list
                todo[todoPos].node = secondChild;
                todo[todoPos].tMin = tPlane;
                todo[todoPos].tMax = tMax;
                ++todoPos;
            }
        } else { // leaf node
            int nPrims = node->nPrimitives();
            if (nPrims == 1) {
                const shared_ptr<Primitive> &p = primitives[node->onePrim];
                if (p->intersect(ray, isect)) hit = true;
            } else {
                for (int i = 0; i < nPrims; ++i) {
                    int index = primIndices[node->primIndicesOffset + i];
                    const shared_ptr<Primitive> &p = primitives[index];
                    if (p->intersect(ray, isect)) hit = true;
                }
            }
            // Grab next node to process
            if (todoPos > 0) {
                --todoPos;
                node = todo[todoPos].node;
                tMin = todo[todoPos].tMin;
                tMax = todo[todoPos].tMax;
            }
            else break;
        }
    }
    return hit;
}

bool KDTree::intersectP(const Ray &ray) const {
    // Compute initial parametric range of ray inside kd-tree extent
    Float tMin, tMax;
    if (!bounds.intersectP(ray, &tMin, &tMax)) return false;

    // Prepare to traverse kd-tree for ray
    struct KDTodo {
        const Node *node;
        Float tMin, tMax;
    };
    Vector3f invDir(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
    constexpr int maxTodo = 64;
    KDTodo todo[maxTodo];
    int todoPos = 0;

    // Traverse kd-tree nodes in order for ray
    bool hit = false;
    const Node *node = &nodes[0];
    while (node) {
        if (ray.tMax < tMin) break; // definitely no closer hit
        if (!node->isLeaf()) { // interior
            // Compute parametric distance along ray to split plane
            int axis = node->splitAxis();
            Float tPlane = (node->splitPos() - ray.o[axis]) * invDir[axis];
            // Get node children pointers for ray
            const Node *firstChild, *secondChild; // front-to-back
            bool belowFirst = (ray.o[axis] < node->splitPos() ||
                              (ray.o[axis] == node->splitPos() && ray.d[axis] <= 0));
            if (belowFirst) {
                firstChild = node + 1;
                secondChild = &nodes[node->aboveChild()];
            } else {
                firstChild = &nodes[node->aboveChild()];
                secondChild = node + 1;
            }
            // Advance to next child node
            if (tPlane > tMax || tPlane <= 0)
                node = firstChild;
            else if (tPlane < tMin)
                node = secondChild;
            else {
                // Enqueue second child in todo list
                todo[todoPos].node = secondChild;
                todo[todoPos].tMin = tPlane;
                todo[todoPos].tMax = tMax;
                ++todoPos;
            }
        } else { // leaf node
            int nPrims = node->nPrimitives();
            if (nPrims == 1) {
                const shared_ptr<Primitive> &p = primitives[node->onePrim];
                if (p->intersectP(ray)) return true;
            } else {
                for (int i = 0; i < nPrims; ++i) {
                    int index = primIndices[node->primIndicesOffset + i];
                    const shared_ptr<Primitive> &p = primitives[index];
                    if (p->intersectP(ray)) return true;
                }
            }
            // Grab next node to process
            if (todoPos > 0) {
                --todoPos;
                node = todo[todoPos].node;
                tMin = todo[todoPos].tMin;
                tMax = todo[todoPos].tMax;
            }
            else break;
        }
    }
    return false;
}

shared_ptr<KDTree> KDTree::create(vector<shared_ptr<Primitive>> prims, const ParamSet &ps)
{
    int isectCost = ps.findOneInt("intersectcost", 80);
    int travCost = ps.findOneInt("traversalcost", 1);
    Float emptyBonus = ps.findOneFloat("emptybonus", 0.5f);
    int maxPrims = ps.findOneInt("maxprims", 1);
    int maxDepth = ps.findOneInt("maxdepth", -1);
    return make_shared<KDTree>((prims), isectCost, travCost, emptyBonus, maxPrims, maxDepth);
}
