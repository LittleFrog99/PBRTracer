#include "api.h"
#include "renderer.h"
#include "paramset.h"

Options Renderer::options;
APIState Renderer::currentApiState = APIState::Uninitialized;
uint32_t Renderer::activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
TransformSet Renderer::curTransform;
map<string, TransformSet> Renderer::namedCoordinateSystems;
unique_ptr<RenderOptions> Renderer::renderOptions;
GraphicsState Renderer::graphicsState;
vector<GraphicsState> Renderer::pushedGraphicsStates;
vector<TransformSet> Renderer::pushedTransforms;
vector<uint32_t> Renderer::pushedActiveTransformBits;
TransformCache Renderer::transformCache;
int Renderer::catIndentCount = 0;

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
