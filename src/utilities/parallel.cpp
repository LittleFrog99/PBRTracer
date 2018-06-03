#include "parallel.h"
#include "stats.h"

void Parallel::forLoop(function<void(int64_t)> func, int64_t count, int chunkSize)
{
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

            else
                loop.func2D(Point2i(index % loop.nX, index / loop.nX));

            Profiler::state = oldState;
        }
        lock.lock();

        // Update _loop_ to reflect completion of iterations
        loop.activeWorkers--;
    }
}

void Barrier::wait() {
    unique_lock<mutex> lock(mut);
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

void Parallel::workerThreadFunc(int tIndex, shared_ptr<Barrier> barrier) {
    threadIndex = tIndex;

    Profiler::workerThreadInit();

    // Make sure that all workers have called ProfilerWorkerThreadInit()
    barrier->wait();
    barrier.reset();

    std::unique_lock<std::mutex> lock(workListMutex);
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
            int64_t indexEnd =
                std::min(indexStart + loop.chunkSize, loop.maxIndex);

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
                else
                    loop.func2D(Point2i(index % loop.nX, index / loop.nX));

                Profiler::state = oldState;
            }
            lock.lock();

            // Update _loop_ to reflect completion of iterations
            loop.activeWorkers--;
            if (loop.isFinished()) workListCondition.notify_all();
        }
    }
}
