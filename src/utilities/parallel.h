#ifndef UTILITY_PARALLEL
#define UTILITY_PARALLEL

#include "vector.h"
#include <thread>

class Barrier {
public:
    Barrier(int count) : count(count) {}
    void wait();

private:
    mutex mut;
    condition_variable cv;
    int count;
};

class Parallel {
public:
    static void init();
    static void cleanup();
    static void mergeWorkerThreadStats();

    static void forLoop(function<void(int64_t)> func, int64_t count, int chunkSize = 1);
    static void forLoop2D(function<void(Point2i)> func, const Point2i &count);

    static unsigned numSystemCores() { return max(1u, thread::hardware_concurrency()); }

private:
    struct ForLoop {
        ForLoop(function<void(int64_t)> func1D, uint64_t maxIndex, int chunkSize,
                uint64_t profState)
            : func1D(move(func1D)), maxIndex(maxIndex), chunkSize(chunkSize),
              profilerState(profState) {}

        ForLoop(const function<void(Point2i)> &func, const Point2i &count, uint64_t profState)
            : func2D(func), maxIndex(count.x * count.y), chunkSize(1),
              profilerState(profState) { nX = count.x; }

        bool isFinished() const { return nextIndex >= maxIndex && activeWorkers == 0; }

        function<void(int64_t)> func1D;
        function<void(Point2i)> func2D;
        const int64_t maxIndex;
        const int chunkSize;
        uint64_t profilerState;
        int64_t nextIndex = 0;
        int activeWorkers = 0;
        ForLoop *next = nullptr;
        int nX = -1;
    };

    static vector<thread> threads;
    static bool shutdownThreads;
    static ForLoop *workList;
    static mutex workListMutex;
    static condition_variable workListCondition;

    static atomic<bool> reportWorkerStats;
    static atomic<int> reporterCount;
    static condition_variable reportDoneCondition;
    static mutex reportDoneMutex;

    static thread_local int threadIndex;

    static void workerThreadFunc(int tIndex, shared_ptr<Barrier> barrier);
    static int maxThreadIndex();
};

class AtomicFloat {
public:
    explicit AtomicFloat(float v = 0) { bits = floatToBits(v); }
    operator float() const { return bitsToFloat(bits); }

    float operator = (float v) {
        bits = floatToBits(v);
        return v;
    }

    void add(float v) {
        uint32_t oldBits = bits, newBits;
        do newBits = floatToBits(bitsToFloat(oldBits) + v);
        while (!bits.compare_exchange_weak(oldBits, newBits));
    }

private:
    atomic<uint32_t> bits;
};



#endif // UTILITY_PARALLEL
