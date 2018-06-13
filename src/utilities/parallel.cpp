#include "parallel.h"
#include "stats.h"
#include "core/renderer.h"

vector<thread> Parallel::threads;
bool Parallel::shutdownThreads(false);
Parallel::ForLoop *Parallel::workList = nullptr;
mutex Parallel::workListMutex;
condition_variable Parallel::workListCondition;
atomic<bool> Parallel::reportWorkerStats(false);
atomic<int> Parallel::reporterCount;
condition_variable Parallel::reportDoneCondition;
mutex Parallel::reportDoneMutex;
thread_local int Parallel::threadIndex;

void Parallel::forLoop(function<void(int64_t)> func, int64_t count, int chunkSize)
{
    CHECK(threads.size() > 0 || maxThreadIndex() == 1);
    // Run iterations if not using threads or if count is small
    if (threads.empty() || count < chunkSize) {
        for (int i = 0; i < count; i++) func(i);
        return;
    }

    // Create and enqueue _ParallelForLoop_ for this loop
    ForLoop loop(std::move(func), count, chunkSize, Profiler::state);
    workListMutex.lock();
    loop.next = workList;
    workList = &loop;
    workListMutex.unlock();

    // Notify worker threads of work to be done
    unique_lock<mutex> lock(workListMutex);
    workListCondition.notify_all();

    // Help out with parallel loop iterations in the current thread
    while (!loop.isFinished()) {
        // Run a chunk of loop iterations for _loop_

        // Find the set of loop iterations to run next
        int64_t indexStart = loop.nextIndex;
        int64_t indexEnd = std::min(indexStart + loop.chunkSize, loop.maxIndex);

        // Update _loop_ to reflect iterations this thread will run
        loop.nextIndex = indexEnd;
        if (loop.nextIndex == loop.maxIndex) workList = loop.next;
        loop.activeWorkers++;

        // Run loop indices in _[indexStart, indexEnd)_
        lock.unlock();
        for (int64_t index = indexStart; index < indexEnd; ++index) {
            uint64_t oldState = Profiler::state;
            Profiler::state = loop.profilerState;
            if (loop.func1D)
                loop.func1D(index);
            else {
                CHECK(loop.func2D);
                loop.func2D(Point2i(index % loop.nX, index / loop.nX));
            }

            Profiler::state = oldState;
        }
        lock.lock();

        // Update _loop_ to reflect completion of iterations
        loop.activeWorkers--;
    }
}

void Parallel::forLoop2D(function<void(Point2i)> func, const Point2i &count)
{
    CHECK(threads.size() > 0 || maxThreadIndex() == 1);
    if (threads.empty() || count.x * count.y <= 1) {
        for (int y = 0; y < count.y; ++y)
            for (int x = 0; x < count.x; ++x) func(Point2i(x, y));
        return;
    }

    ForLoop loop(std::move(func), count, Profiler::state);
    {
        lock_guard<mutex> lock(workListMutex);
        loop.next = workList;
        workList = &loop;
    }

    unique_lock<mutex> lock(workListMutex);
    workListCondition.notify_all();

    while (!loop.isFinished()) {
        int64_t indexStart = loop.nextIndex;
        int64_t indexEnd = min(indexStart + loop.chunkSize, loop.maxIndex);

        loop.nextIndex = indexEnd;
        if (loop.nextIndex == loop.maxIndex) workList = loop.next;
        loop.activeWorkers++;

        lock.unlock();
        for (int64_t index = indexStart; index < indexEnd; ++index) {
            uint64_t oldState = Profiler::state;
            Profiler::state = loop.profilerState;
            if (loop.func1D)
                loop.func1D(index);
            else {
                CHECK(loop.func2D);
                loop.func2D(Point2i(index % loop.nX, index / loop.nX));
            }
            Profiler::state = oldState;
        }
        lock.lock();

        loop.activeWorkers--;
    }
}

void Parallel::workerThreadFunc(int tIndex, shared_ptr<Barrier> barrier)
{
    LOG(INFO) << "Started execution in worker thread " << tIndex;
    threadIndex = tIndex;
    Profiler::workerThreadInit();

    // Make sure that all workers have called ProfilerWorkerThreadInit()
    barrier->wait();
    barrier.reset();

    unique_lock<mutex> lock(workListMutex);
    while (!shutdownThreads) {
        if (reportWorkerStats) {
            Stats::reportThread();
            if (--reporterCount == 0)
                // Once all worker threads have merged their stats, wake up
                // the main thread.
                reportDoneCondition.notify_one();
            // Now sleep again.
            workListCondition.wait(lock);
        } else if (!workList)
            // Sleep until there are more tasks to run
            workListCondition.wait(lock);
        else {
            // Get work from _workList_ and run loop iterations
            ForLoop &loop = *workList;

            // Run a chunk of loop iterations for _loop_

            // Find the set of loop iterations to run next
            int64_t indexStart = loop.nextIndex;
            int64_t indexEnd = min(indexStart + loop.chunkSize, loop.maxIndex);

            // Update _loop_ to reflect iterations this thread will run
            loop.nextIndex = indexEnd;
            if (loop.nextIndex == loop.maxIndex) workList = loop.next;
            loop.activeWorkers++;

            // Run loop indices in _[indexStart, indexEnd)_
            lock.unlock();
            for (int64_t index = indexStart; index < indexEnd; ++index) {
                uint64_t oldState = Profiler::state;
                Profiler::state = loop.profilerState;
                if (loop.func1D)
                    loop.func1D(index);
                else {
                    CHECK(loop.func2D);
                    loop.func2D(Point2i(index % loop.nX, index / loop.nX));
                }

                Profiler::state = oldState;
            }
            lock.lock();

            // Update _loop_ to reflect completion of iterations
            loop.activeWorkers--;
            if (loop.isFinished()) workListCondition.notify_all();
        }
    }
    LOG(INFO) << "Exiting worker thread " << tIndex;
}

void Barrier::wait() {
    unique_lock<mutex> lock(mut);
    CHECK_GT(count, 0);
    if (--count == 0)
        cv.notify_all();
    else
        cv.wait(lock, [this] { return count == 0; });
}

void Parallel::init() {
    int nThreads = maxThreadIndex();
    threadIndex = 0;

    shared_ptr<Barrier> barrier = make_shared<Barrier>(nThreads);

    // Launch one fewer worker thread than the total number
    for (int i = 0; i < nThreads - 1; ++i)
        threads.push_back(thread(workerThreadFunc, i + 1, barrier));

    barrier->wait();
}

void Parallel::cleanup() {
    if (threads.empty()) return;

    std::lock_guard<std::mutex> lock(workListMutex);
    shutdownThreads = true;
    workListCondition.notify_all();

    for (thread &thread : threads) thread.join();
    threads.erase(threads.begin(), threads.end());
    shutdownThreads = false;
}

void Parallel::mergeWorkerThreadStats() {
    unique_lock<mutex> lock(workListMutex);
    unique_lock<mutex> doneLock(reportDoneMutex);

    reportWorkerStats = true;
    reporterCount = threads.size();

    workListCondition.notify_all();
    reportDoneCondition.wait(lock, []() { return reporterCount == 0; });
    reportWorkerStats = false;
}

int Parallel::maxThreadIndex() {
    return Renderer::options.nThreads == 0 ? numSystemCores() : Renderer::options.nThreads;
}
