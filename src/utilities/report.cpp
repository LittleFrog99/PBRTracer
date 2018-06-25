#include "report.h"
#include "stats.h"
#include "parallel.h"
#include "core/renderer.h"
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

ProgressReporter::ProgressReporter(int64_t totalWork, const string &title)
    : totalWork(max((int64_t)1, totalWork)), title(title),
      startTime(chrono::system_clock::now()) {
    workDone = 0;
    exitThread = false;
    // Launch thread to periodically update progress bar
    if (!Renderer::options.quiet) {
        Profiler::suspend();
        shared_ptr<Barrier> barrier = make_shared<Barrier>(2);
        updateThread = thread([this, barrier]() {
            Profiler::workerThreadInit();
            Profiler::state = 0;
            barrier->wait();
            printBar();
        });
        // Wait for the thread to get past the ProfilerWorkerThreadInit() call.
        barrier->wait();
        Profiler::resume();
    }
}

void ProgressReporter::update(int64_t num) {
    if (num == 0 || Renderer::options.quiet) return;
    workDone += num;
}

ProgressReporter::~ProgressReporter() {
    if (!Renderer::options.quiet) {
        workDone = totalWork;
        exitThread = true;
        updateThread.join();
        printf("\n");
    }
}

void ProgressReporter::printBar() {
    int barLength = terminalWidth() - 28;
    int totalPlusses = max(2, barLength - (int)title.size());
    int plussesPrinted = 0;

    // Initialize progress string
    const int bufLen = title.size() + totalPlusses + 64;
    unique_ptr<char[]> buf(new char[bufLen]);
    snprintf(buf.get(), bufLen, "\r%s: [", title.c_str());
    char *curSpace = buf.get() + strlen(buf.get());
    char *s = curSpace;
    for (int i = 0; i < totalPlusses; ++i) *s++ = ' ';
    *s++ = ']';
    *s++ = ' ';
    *s++ = '\0';
    fputs(buf.get(), stdout);
    fflush(stdout);

    chrono::milliseconds sleepDuration(250);
    int iterCount = 0;
    while (!exitThread) {
        this_thread::sleep_for(sleepDuration);

        // Periodically increase sleepDuration to reduce overhead of
        // updates.
        ++iterCount;
        if (iterCount == 10)
            // Up to 0.5s after ~2.5s elapsed
            sleepDuration *= 2;
        else if (iterCount == 70)
            // Up to 1s after an additional ~30s have elapsed.
            sleepDuration *= 2;
        else if (iterCount == 520)
            // After 15m, jump up to 5s intervals
            sleepDuration *= 5;

        Float percentDone = Float(workDone) / Float(totalWork);
        int plussesNeeded = round(totalPlusses * percentDone);
        while (plussesPrinted < plussesNeeded) {
            *curSpace++ = '+';
            ++plussesPrinted;
        }
        fputs(buf.get(), stdout);

        // Update elapsed time and estimated time to completion
        Float seconds = elapsedMS() / 1000.f;
        Float estRemaining = seconds / percentDone - seconds;
        if (percentDone == 1.f)
            printf(" (%.1fs)       ", seconds);
        else if (!isinf(estRemaining))
            printf(" (%.1fs|%.1fs)  ", seconds,
                   max((Float)0., estRemaining));
        else
            printf(" (%.1fs|?s)  ", seconds);
        fflush(stdout);
    }
}

void ProgressReporter::done() {
    workDone = totalWork;
}

int ProgressReporter::terminalWidth() {
    struct winsize w;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) < 0) {
        // ENOTTY is fine and expected, e.g. if output is being piped to a file.
        if (errno != ENOTTY) {
            static bool warned = false;
            if (!warned) {
                warned = true;
                fprintf(stderr, "Error in ioctl() in TerminalWidth(): %d\n", errno);
            }
        }
        return 80;
    }
    return w.ws_col;
}

void Report::severe(const char *format, ...) {
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Error");
    va_end(args);
}

void Report::error(const char *format, ...) {
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Error");
    va_end(args);
}

void Report::warning(const char *format, ...) {
    if (Renderer::options.quiet) return;
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Warning");
    va_end(args);
}

void Report::info(const char *format, ...) {
    if (Renderer::options.quiet) return;
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Info");
    va_end(args);
}

void Report::processError(Parser::Location *loc, const char *format, va_list args,
                          const char *errorType)
{
    string errorString;
    // Print line and position in input file, if available
    if (loc)
        errorString = StringPrint::printf("%s:%d:%d: ", loc->filename.c_str(), loc->line, loc->column);
    errorString += errorType;
    errorString += ": ";
    errorString += StringPrint::printf(format, args);

    // Print the error message (but not more than one time).
    static string lastError;
    static std::mutex mutex;
    lock_guard<std::mutex> lock(mutex);
    if (errorString != lastError) {
        LOG(INFO) << errorString;
        fprintf(stderr, "%s\n", errorString.c_str());
        lastError = errorString;
    }
}
