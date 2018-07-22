#include "bvh.h"
#include "parallel.h"
#include "paramset.h"
#include "log.h"

struct BVH::PrimitiveInfo {
    PrimitiveInfo() {}
    PrimitiveInfo(size_t primNumber, const Bounds3f &bounds)
        : primitiveNumber(primNumber), bounds(bounds), centroid(0.5 * (bounds.pMin + bounds.pMax)) {}

    size_t primitiveNumber;
    Bounds3f bounds;
    Point3f centroid;
};

struct BVH::BuildNode {
    Bounds3f bounds;
    BuildNode *children[2];
    int splitAxis, firstPrimOffset, nPrims;

    void initLeaf(int first, int n, const Bounds3f &b) {
        firstPrimOffset = first;
        nPrims = n;
        bounds = b;
        children[0] = children[1] = nullptr;
    }

    void initInterior(int axis, BuildNode *c0, BuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = unionOf(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrims = 0;
    }
};

struct BVH::LinearNode {
    Bounds3f bounds;
    union {
        int primsOffset; // leaf
        int secondChildOffset; // interior
    };
    uint16_t nPrims; // 0 -> interior node
    uint16_t axis; // interior node: xyz
};

BVH::BVH(vector<shared_ptr<Primitive>> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(min(255, maxPrimsInNode)), primitives(p), splitMethod(splitMethod)
{
    if (primitives.size() == 0) return;

    // Initialize $primitiveInfo array for primitives
    vector<PrimitiveInfo> primsInfo(primitives.size());
    for (unsigned i = 0; i < primitives.size(); i++)
        primsInfo[i] = PrimitiveInfo(i, primitives[i]->worldBound());

    // Build BVH tree for primitives
    MemoryArena arena(1024 * 1024);
    int totalNodes = 0;
    vector<shared_ptr<Primitive>> orderedPrims;
    BuildNode *root;
    root = (splitMethod == SplitMethod::HLBVH) ?
               HLBVHBuild(arena, primsInfo, &totalNodes, orderedPrims)
             : recursiveBuild(arena, primsInfo, 0, primitives.size(), &totalNodes, orderedPrims);
    primitives.swap(orderedPrims);

    // Compute representation of depth-first traversal of BVH tree
    nodes = Memory::allocAligned<LinearNode>(totalNodes);
    int offset = 0;
    flattenBVHTree(root, &offset);
}

BVH::BuildNode *BVH::recursiveBuild(MemoryArena &arena, vector<PrimitiveInfo> &primsInfo,
                                    int start, int end, int *totalNodes,
                                    vector<shared_ptr<Primitive>> &orderedPrims)
{
    BuildNode *node = arena.alloc<BuildNode>();
    (*totalNodes)++;
    Bounds3f bounds;
    for (int i = start; i < end; i++)
        bounds = unionOf(bounds, primsInfo[i].bounds);

    int nPrims = end - start;
    if (nPrims == 1) {
        // Create leaf node
        int firstPrimOffset = orderedPrims.size();
        for (int i = start; i < end; i++) {
            size_t primNum = primsInfo[i].primitiveNumber;
            orderedPrims.push_back(primitives[primNum]);
        }
        node->initLeaf(firstPrimOffset, nPrims, bounds);
        return node;
    } else {
        // Compute bounds of primitive centroids, choose split dimension
        Bounds3f centroidBounds;
        for (int i = start; i < end; i++)
            centroidBounds = unionOf(centroidBounds, primsInfo[i].centroid);
        int dim = centroidBounds.maxExtent();

        // Partition primitives into two sets and build children
        int mid = (start + end) / 2;
        if (centroidBounds.pMax[mid] == centroidBounds.pMin[mid]) { // centroid points at the same position
            int firstPrimOffset = orderedPrims.size();
            for (int i = start; i < end; i++) {
                size_t primNum = primsInfo[i].primitiveNumber;
                orderedPrims.push_back(primitives[primNum]);
            }
            node->initLeaf(firstPrimOffset, nPrims, bounds);
            return node;
        } else {
            // Partition primitives based on $splitMethod
            switch (splitMethod) {
            case SplitMethod::Middle: {
                float pMid = (centroidBounds.pMin[dim] + centroidBounds.pMax[dim]) / 2;
                auto *midPtr = partition(&primsInfo[start], &primsInfo[end - 1] + 1, // in case of vector error
                        [dim, pMid] (const PrimitiveInfo &pi) { return pi.centroid[dim] < pMid; });
                mid = midPtr - &primsInfo[0];
                if (mid != start && mid != end) break; // successful separation
            }

            [[fallthrough]];
            case SplitMethod::EqualCounts: {
                nth_element(&primsInfo[start], &primsInfo[mid], &primsInfo[end],
                            [dim] (const PrimitiveInfo &a, const PrimitiveInfo &b) {
                    return a.centroid[dim] < b.centroid[dim];
                });
                break;
            }

            case SplitMethod::SAH:
            default: {
                if (nPrims <= 4) { // applying SAH at this point isn't worthwhile
                    nth_element(&primsInfo[start], &primsInfo[mid], &primsInfo[end],
                                [dim] (const PrimitiveInfo &a, const PrimitiveInfo &b) {
                        return a.centroid[dim] < b.centroid[dim];
                    });
                    break;
                } else {
                    // Allocate and initialize $BucketInfo
                    constexpr int nBuckets = 12;
                    struct BucketInfo {
                        int count = 0;
                        Bounds3f bounds;
                    };
                    BucketInfo buckets[nBuckets];
                    for (int i = start; i < end; i++) {
                        auto b = int(nBuckets * centroidBounds.offset(primsInfo[i].centroid)[dim]);
                        if (b == nBuckets) b--;
                        buckets[b].count++;
                        buckets[b].bounds = unionOf(buckets[b].bounds, primsInfo[b].bounds);
                    }

                    // Compute costs for splitting after each bucket
                    float cost[nBuckets - 1];
                    for (int i = 0; i < nBuckets - 1; i++) {
                        Bounds3f b0, b1;
                        int count0 =0, count1 = 0;
                        for (int j = 0; j < i; j++) {
                            b0 = unionOf(b0, buckets[j].bounds);
                            count0 += buckets[j].count;
                        }
                        for (int j = i + 1; j < nBuckets; j++) {
                            b1 = unionOf(b1, buckets[j].bounds);
                            count1 += buckets[j].count;
                        }
                        cost[i] = 0.125f + (count0 * b0.surfaceArea()
                                            + count1 * b1.surfaceArea()) / bounds.surfaceArea();
                    }

                    // Find bucket to split at that minimizes SAH metric
                    float minCost = cost[0];
                    int minCostBucket = 0;
                    for (int i = 1; i < nBuckets - 1; i++)
                        if (cost[i] < minCost) {
                            minCost = cost[i];
                            minCostBucket = i;
                        }

                    // Either create leaf or split primitives at selected SAH bucket
                    float leafCost = nPrims;
                    if (nPrims > maxPrimsInNode || minCost < leafCost) {
                        PrimitiveInfo *pmid = partition(&primsInfo[start], &primsInfo[end-1]+1,
                                [=] (const PrimitiveInfo &pi) {
                            int b = nBuckets * centroidBounds.offset(pi.centroid)[dim];
                            if (b == nBuckets) b = nBuckets - 1;
                            return b <= minCostBucket;
                        });
                        mid = pmid - &primsInfo[0];
                    } else {
                        int firstPrimOffset = orderedPrims.size();
                        for (int i = start; i < end; i++) {
                            size_t primNum = primsInfo[i].primitiveNumber;
                            orderedPrims.push_back(primitives[primNum]);
                        }
                        node->initLeaf(firstPrimOffset, nPrims, bounds);
                        return node;
                    }
                } // end if (nPrims <= 4)
                break;
            } // end case SplitMethod::SAH
            } // end switch (splitMetod)

            node->initInterior(dim, recursiveBuild(arena, primsInfo, start, mid, totalNodes, orderedPrims),
                               recursiveBuild(arena, primsInfo, mid, end, totalNodes, orderedPrims));
        } // end partition primitives
    } // end if (nPrims <= 1)

    return node;
}

inline uint32_t BVH::leftShift3(uint32_t x) {
    if (x == (1 << 10)) --x;
    x = (x | (x << 16)) & 0b00000011000000000000000011111111;
    x = (x | (x << 8))  & 0b00000011000000001111000000001111;
    x = (x | (x << 4))  & 0b00000011000011000011000011000011;
    x = (x | (x << 2))  & 0b00001001001001001001001001001001;
    return x;
}

inline uint32_t BVH::encodeMorton(const Vector3f &v) {
    return (leftShift3(v.z) << 2) | (leftShift3(v.y) << 1) | leftShift3(v.x);
}

struct BVH::MortonPrimitive {
    size_t primitiveIndex;
    uint32_t mortonCode;
};

void BVH::radixSort(vector<MortonPrimitive> *v) {
    vector<MortonPrimitive> temp(v->size());
    constexpr int bitsPerPass = 6;
    constexpr int nBits = 30;
    constexpr int nPasses = nBits / bitsPerPass;
    for (int pass = 0; pass < nPasses; pass++) {
        int lowBit = pass * bitsPerPass;
        // Set in and out vector for radix sort pass
        auto &in = (pass & 1) ? temp : *v;
        auto &out = (pass & 1) ? *v : temp;

        // Count number of zero bits in array for current radix sort bit
        constexpr int nBuckets = 1 << bitsPerPass;
        int bucketCount[nBuckets] = { 0 };
        constexpr int bitMask = (1 << bitsPerPass) - 1; // 0b111111
        for (const auto &mp : in) {
            int bucket = (mp.mortonCode >> lowBit) & bitMask;
            ++bucketCount[bucket];
        }

        // Compute starting index in output array for each bucket
        int outIndex[nBuckets];
        outIndex[0] = 0;
        for (int i = 1; i < nBuckets; i++)
            outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];

        // Store sorted values in output array
        for (const auto &mp : in) {
            int bucket = (mp.mortonCode >> lowBit) & bitMask;
            out[outIndex[bucket]++] = mp;
        }
    }

    if (nPasses & 1) swap(*v, temp);
}

struct BVH::LBVHTreelet {
    int startIndex, nPrims;
    BuildNode *buildNodes;
};

BVH::BuildNode *BVH::HLBVHBuild(MemoryArena &arena, const vector<PrimitiveInfo> &primsInfo,
                                int *totalNodes, vector<shared_ptr<Primitive>> &orderedPrims) const
{
    // Compute bounding box of all primitive centroids
    Bounds3f bounds;
    for (const PrimitiveInfo &pi : primsInfo)
        bounds = unionOf(bounds, pi.centroid);

    // Compute Morton indices of primitives
    vector<MortonPrimitive> mortonPrims(primsInfo.size());
    Parallel::forLoop(
                [&] (unsigned i) {
        constexpr int mortonBits = 10;
        constexpr int mortonScale = 1 << mortonBits;
        mortonPrims[i].primitiveIndex = primsInfo[i].primitiveNumber;
        Vector3f centroidOffset = bounds.offset(primsInfo[i].centroid);
        mortonPrims[i].mortonCode = encodeMorton(centroidOffset * mortonScale);
    }, primsInfo.size(), 512); // amortize the overhead of distributing tasks

    // Radix sort Morton primitives
    radixSort(&mortonPrims);

    // Create LBVH at bottom of BVH
    // Find intervals of primitives for each treelet
    vector<LBVHTreelet> treeletsToBuild;
    for (int start = 0, end = 1; end <= mortonPrims.size(); end++) {
        uint32_t mask = 0b00111111111111000000000000000000; // find primitives that have the same high 12 bits
        if (end == mortonPrims.size()
                || ((mortonPrims[start].mortonCode & mask) != (mortonPrims[end].mortonCode & mask)) ) {
            // Add entry to $treeletsToBuild for this treelet
            int nPrims = end - start;
            int maxBVHNodes = 2 * nPrims;
            auto *nodes = arena.alloc<BuildNode>(maxBVHNodes, false);
            treeletsToBuild.push_back({start, nPrims, nodes});
            start = end;
        }
    }
    // Create LBVHs for treelets in parallel
    atomic<int> atomicTotal(0), orderedPrimsOffset(0);
    orderedPrims.resize(primitives.size());
    Parallel::forLoop(
                [&] (unsigned i) {
        // Generate ith LBVH treelet
        int nodesCreated = 0;
        const int firstBitIndex = 29 - 12;
        LBVHTreelet &tr = treeletsToBuild[i];
        tr.buildNodes = emitLBVH(tr.buildNodes, primsInfo, &mortonPrims[tr.startIndex], tr.nPrims, &nodesCreated,
                orderedPrims, &orderedPrimsOffset, firstBitIndex);
        atomicTotal += nodesCreated;
    }, treeletsToBuild.size());
    *totalNodes = atomicTotal;

    // Create and return SAH BVH from LBVH treelets
    vector<BuildNode *> finishedTreelets;
    finishedTreelets.reserve(treeletsToBuild.size());
    for (LBVHTreelet &treelet : treeletsToBuild)
        finishedTreelets.push_back(treelet.buildNodes);
    return buildUpperSAH(arena, finishedTreelets, 0, finishedTreelets.size(), totalNodes);
}

BVH::BuildNode * BVH::emitLBVH(BuildNode *&buildNodes, const vector<PrimitiveInfo> &primitiveInfo,
                               MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
                               vector<shared_ptr<Primitive>> &orderedPrims,
                               atomic<int> *orderedPrimsOffset, int bitIndex) const {
    if (bitIndex == -1 || nPrimitives < maxPrimsInNode) {
        // Create and return leaf node of LBVH treelet
        (*totalNodes)++;
        BuildNode *node = buildNodes++;
        Bounds3f bounds;
        int firstPrimOffset = orderedPrimsOffset->fetch_add(nPrimitives);
        for (int i = 0; i < nPrimitives; ++i) {
            int primitiveIndex = mortonPrims[i].primitiveIndex;
            orderedPrims[firstPrimOffset + i] = primitives[primitiveIndex];
            bounds = unionOf(bounds, primitiveInfo[primitiveIndex].bounds);
        }
        node->initLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    } else {
        int mask = 1 << bitIndex;
        // Advance to next subtree level if there's no LBVH split for this bit
        if ((mortonPrims[0].mortonCode & mask) == (mortonPrims[nPrimitives - 1].mortonCode & mask))
            return emitLBVH(buildNodes, primitiveInfo, mortonPrims, nPrimitives,totalNodes, orderedPrims,
                            orderedPrimsOffset, bitIndex - 1);

        // Find LBVH split point for this dimension
        int searchStart = 0, searchEnd = nPrimitives - 1;
        while (searchStart + 1 != searchEnd) {
            int mid = (searchStart + searchEnd) / 2;
            if ((mortonPrims[searchStart].mortonCode & mask) == (mortonPrims[mid].mortonCode & mask))
                searchStart = mid;
            else
                searchEnd = mid;
        }
        int splitOffset = searchEnd;

        // Create and return interior LBVH node
        (*totalNodes)++;
        BuildNode *node = buildNodes++;
        auto *lbvh0 = emitLBVH(buildNodes, primitiveInfo, mortonPrims, splitOffset, totalNodes, orderedPrims,
                               orderedPrimsOffset, bitIndex - 1);
        auto *lbvh1 = emitLBVH(buildNodes, primitiveInfo, &mortonPrims[splitOffset], nPrimitives - splitOffset,
                               totalNodes, orderedPrims, orderedPrimsOffset, bitIndex - 1);
        int axis = bitIndex % 3;
        node->initInterior(axis, lbvh0, lbvh1);
        return node;
    }
}

BVH::BuildNode * BVH::buildUpperSAH(MemoryArena &arena, vector<BuildNode *> &treeletRoots,
                                    int start, int end, int *totalNodes) const {
    int nNodes = end - start;
    if (nNodes == 1) return treeletRoots[start];
    (*totalNodes)++;
    BuildNode *node = arena.alloc<BuildNode>();

    // Compute bounds of all nodes under this HLBVH node
    Bounds3f bounds;
    for (int i = start; i < end; ++i)
        bounds = unionOf(bounds, treeletRoots[i]->bounds);

    // Compute bound of HLBVH node centroids, choose split dimension _dim_
    Bounds3f centroidBounds;
    for (int i = start; i < end; ++i) {
        Point3f centroid = (treeletRoots[i]->bounds.pMin + treeletRoots[i]->bounds.pMax) * 0.5f;
        centroidBounds = unionOf(centroidBounds, centroid);
    }
    int dim = centroidBounds.maxExtent();

    // Allocate _BucketInfo_ for SAH partition buckets
    constexpr int nBuckets = 12;
    struct BucketInfo {
        int count = 0;
        Bounds3f bounds;
    };
    BucketInfo buckets[nBuckets];

    // Initialize _BucketInfo_ for HLBVH SAH partition buckets
    for (int i = start; i < end; ++i) {
        float centroid = (treeletRoots[i]->bounds.pMin[dim] +
                          treeletRoots[i]->bounds.pMax[dim]) *
                         0.5f;
        int b = nBuckets * ((centroid - centroidBounds.pMin[dim]) /
                            (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
        if (b == nBuckets) b = nBuckets - 1;
        buckets[b].count++;
        buckets[b].bounds = unionOf(buckets[b].bounds, treeletRoots[i]->bounds);
    }

    // Compute costs for splitting after each bucket
    float cost[nBuckets - 1];
    for (int i = 0; i < nBuckets - 1; ++i) {
        Bounds3f b0, b1;
        int count0 = 0, count1 = 0;
        for (int j = 0; j <= i; ++j) {
            b0 = unionOf(b0, buckets[j].bounds);
            count0 += buckets[j].count;
        }
        for (int j = i + 1; j < nBuckets; ++j) {
            b1 = unionOf(b1, buckets[j].bounds);
            count1 += buckets[j].count;
        }
        cost[i] = .125f + (count0 * b0.surfaceArea() + count1 * b1.surfaceArea()) / bounds.surfaceArea();
    }

    // Find bucket to split at that minimizes SAH metric
    float minCost = cost[0];
    int minCostSplitBucket = 0;
    for (int i = 1; i < nBuckets - 1; ++i) {
        if (cost[i] < minCost) {
            minCost = cost[i];
            minCostSplitBucket = i;
        }
    }

    // Split nodes and create interior HLBVH SAH node
    BuildNode **pmid = partition(&treeletRoots[start], &treeletRoots[end - 1] + 1,
            [=] (const BuildNode *node) {
        float centroid = (node->bounds.pMin[dim] + node->bounds.pMax[dim]) * 0.5f;
        int b = nBuckets * ((centroid - centroidBounds.pMin[dim]) /
                            (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
        if (b == nBuckets) b = nBuckets - 1;
        return b <= minCostSplitBucket;
    });
    int mid = pmid - &treeletRoots[0];
    node->initInterior(dim, this->buildUpperSAH(arena, treeletRoots, start, mid, totalNodes),
                       this->buildUpperSAH(arena, treeletRoots, mid, end, totalNodes));
    return node;
}

int BVH::flattenBVHTree(BuildNode *node, int *offset) {
    auto linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    int localOffset = (*offset)++;
    if (node->nPrims > 0) { // leaf
        linearNode->primsOffset = node->firstPrimOffset;
        linearNode->nPrims = node->nPrims;
    } else { // interior
        linearNode->axis = node->splitAxis;
        linearNode->nPrims = 0;
        flattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset = flattenBVHTree(node->children[1], offset);
    }
    return localOffset;
}

bool BVH::intersect(const Ray &ray, SurfaceInteraction *isect) const {
    if (!nodes) return false;
    bool hit = false;
    Vector3f invDir = Vector3f(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
    int dirIsDeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    int toVisitOffset = 0, curNodeIndex = 0;
    int nodesToVisit[64]; // serve as a stack
    while (true) {
        const auto *node = &nodes[curNodeIndex];
        if (node->bounds.intersectP(ray, invDir, dirIsDeg)) {
            if (node->nPrims > 0) { // leaf
                for (int i = 0; i < node->nPrims; i++)
                    if (primitives[node->primsOffset + i]->intersect(ray, isect))
                        return true;
                if (toVisitOffset == 0) break;
                curNodeIndex = nodesToVisit[--toVisitOffset];
            } else { // interior
                if (dirIsDeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = curNodeIndex + 1; // second child first search
                    curNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    curNodeIndex++;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            curNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return hit;
}

bool BVH::intersectP(const Ray &ray) const {
    if (!nodes) return false;
    Vector3f invDir = Vector3f(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
    int dirIsDeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    int toVisitOffset = 0, curNodeIndex = 0;
    int nodesToVisit[64]; // serve as a stack
    while (true) {
        const auto *node = &nodes[curNodeIndex];
        if (node->bounds.intersectP(ray, invDir, dirIsDeg)) {
            if (node->nPrims > 0) { // leaf
                for (int i = 0; i < node->nPrims; i++)
                    if (primitives[node->primsOffset + i]->intersectP(ray))
                        return true;
                if (toVisitOffset == 0) break;
                curNodeIndex = nodesToVisit[--toVisitOffset];
            } else { // interior
                if (dirIsDeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = curNodeIndex + 1; // second child first search
                    curNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    curNodeIndex++;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            curNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    return false;
}

Bounds3f BVH::worldBound() const {
    return nodes ? nodes[0].bounds : Bounds3f();
}

BVH::~BVH() {
    Memory::freeAligned(nodes);
}

shared_ptr<BVH> BVH::create(vector<shared_ptr<Primitive>> prims, const ParamSet &ps)
{
    std::string splitMethodName = ps.findOneString("splitmethod", "sah");
    SplitMethod splitMethod;
    if (splitMethodName == "sah")
        splitMethod = SplitMethod::SAH;
    else if (splitMethodName == "hlbvh")
        splitMethod = SplitMethod::HLBVH;
    else if (splitMethodName == "middle")
        splitMethod = SplitMethod::Middle;
    else if (splitMethodName == "equal")
        splitMethod = SplitMethod::EqualCounts;
    else {
        WARNING("BVH split method \"%s\" unknown.  Using \"sah\".",
                splitMethodName.c_str());
        splitMethod = SplitMethod::SAH;
    }

    int maxPrimsInNode = ps.findOneInt("maxnodeprims", 4);
    return make_shared<BVH>(move(prims), maxPrimsInNode, splitMethod);
}
