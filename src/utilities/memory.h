#ifndef MEMORY_H
#define MEMORY_H

#include "utilities.h"
#include <list>

#ifndef L1_CACHE_LINE_SIZE
  #define L1_CACHE_LINE_SIZE 64
#endif

#define ARENA_ALLOC(arena, Type) new ((arena).alloc(sizeof(Type))) Type

class Memory {
public:
    template <typename T>
    static T * allocAligned(size_t count) {
        return (T *)allocAligned(count * sizeof(T));
    }

    static void freeAligned(void *ptr) {
        if (!ptr) return;
        free(ptr);
    }

private:
    static void * allocAligned(size_t size) {
        void *ptr;
        if (posix_memalign(&ptr, L1_CACHE_LINE_SIZE, size) != 0) ptr = nullptr;
        return ptr;
    }

};

class alignas(L1_CACHE_LINE_SIZE) MemoryArena {
  public:
    // MemoryArena Public Methods
    static constexpr size_t DEFAULT_BLOCK_SIZE = 262144; // 256 KB

    MemoryArena(size_t blockSize = DEFAULT_BLOCK_SIZE) : blockSize(blockSize) {}

    ~MemoryArena() {
        Memory::freeAligned(currentBlock);
        for (auto &block : usedBlocks) Memory::freeAligned(block.second);
        for (auto &block : availableBlocks) Memory::freeAligned(block.second);
    }

    void * alloc(size_t nBytes);

    template <typename T>
    T * alloc(size_t n = 1, bool runConstructor = true) {
        T *ret = (T *)alloc(n * sizeof(T));
        if (runConstructor)
            for (size_t i = 0; i < n; ++i) new (&ret[i]) T();
        return ret;
    }

    void reset() {
        currentBlockPos = 0;
        availableBlocks.splice(availableBlocks.begin(), usedBlocks);
    }

    size_t totalAllocated() const {
        size_t total = currentAllocSize;
        for (const auto &alloc : usedBlocks) total += alloc.first;
        for (const auto &alloc : availableBlocks) total += alloc.first;
        return total;
    }

private:
    const size_t blockSize;
    size_t currentBlockPos = 0, currentAllocSize = 0;
    uint8_t *currentBlock = nullptr;
    list<pair<size_t, uint8_t *>> usedBlocks, availableBlocks;

    MemoryArena(const MemoryArena &) = delete;
    MemoryArena & operator = (const MemoryArena &) = delete;
};

template <typename T, int logblockSize>
class BlockedArray {
public:
    BlockedArray(int uRes, int vRes, const T *d = nullptr)
        : uRes(uRes), vRes(vRes), uBlocks(roundUp(uRes) >> logblockSize) {
        int nAlloc = roundUp(uRes) * roundUp(vRes);
        data = Memory::allocAligned<T>(nAlloc);
        for (int i = 0; i < nAlloc; ++i) new (&data[i]) T();
        if (d)
            for (int v = 0; v < vRes; ++v)
                for (int u = 0; u < uRes; ++u) (*this)(u, v) = d[v * uRes + u];
    }

    constexpr int blockSize() const { return 1 << logblockSize; }
    int roundUp(int x) const { return (x + blockSize() - 1) & ~(blockSize() - 1); }
    int uSize() const { return uRes; }
    int vSize() const { return vRes; }
    int block(int a) const { return a >> logblockSize; }
    int offset(int a) const { return a & (blockSize() - 1); }

    T & operator () (int u, int v) {
        int bu = block(u), bv = block(v);
        int ou = offset(u), ov = offset(v);
        int offset = blockSize() * blockSize() * (uBlocks * bv + bu);
        offset += blockSize() * ov + ou;
        return data[offset];
    }

    const T & operator () (int u, int v) const {
        int bu = block(u), bv = block(v);
        int ou = offset(u), ov = offset(v);
        int offset = blockSize() * blockSize() * (uBlocks * bv + bu);
        offset += blockSize() * ov + ou;
        return data[offset];
    }

    void getLinearArray(T *a) const {
        for (int v = 0; v < vRes; ++v)
            for (int u = 0; u < uRes; ++u) *a++ = (*this)(u, v);
    }

    ~BlockedArray() {
        for (int i = 0; i < uRes * vRes; ++i) data[i].~T();
        Memory::freeAligned(data);
    }

private:
    T *data;
    const int uRes, vRes, uBlocks;
};

#endif // MEMORY_H
