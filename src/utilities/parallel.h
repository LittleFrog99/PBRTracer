#ifndef PARALLEL_H
#define PARALLEL_H

#include "utilities.h"

class Parallel {
public:
    static void init();
    static void cleanup();
    static void mergeWorkerThreadStats();
    static void pFor(function<void(int64_t)> func, int64_t count, int chunkSize = 1);
    static void pFor2D();
};

#endif // PARALLEL_H
