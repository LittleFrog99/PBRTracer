#include "memory.h"

void * MemoryArena::alloc(size_t nBytes) {
    // Round up _nBytes_ to minimum machine alignment
    const unsigned align = alignof(max_align_t);
    static_assert(Math::isPowerOf2(align), "Minimum alignment not a power of two");
    nBytes = (nBytes + align - 1) & ~(align - 1);
    if (currentBlockPos + nBytes > currentAllocSize) {
        // Add current block to _usedBlocks_ list
        if (currentBlock) {
            usedBlocks.push_back(make_pair(currentAllocSize, currentBlock));
            currentBlock = nullptr;
            currentAllocSize = 0;
        }

        // Get new block of memory for _MemoryArena_

        // Try to get memory block from _availableBlocks_
        for (auto iter = availableBlocks.begin(); iter != availableBlocks.end(); ++iter) {
            if (iter->first >= nBytes) {
                currentAllocSize = iter->first;
                currentBlock = iter->second;
                availableBlocks.erase(iter);
                break;
            }
        }
        if (!currentBlock) {
            currentAllocSize = max(nBytes, blockSize);
            currentBlock = Memory::allocAligned<uint8_t>(currentAllocSize);
        }
        currentBlockPos = 0;
    }
    void *ret = currentBlock + currentBlockPos;
    currentBlockPos += nBytes;
    return ret;
}
