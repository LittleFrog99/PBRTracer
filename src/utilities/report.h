#ifndef REPORT_H
#define REPORT_H

#include "stringprint.h"
#include "parser.h"
#include "core/renderer.h"
#include <thread>

class ProgressReporter {
public:
    ProgressReporter(int64_t totalWork, const string &title);
    ~ProgressReporter();

    void update(int64_t num = 1) {
        if (num == 0 || Renderer::options.quiet) return;
        workDone += num;
    }

    Float elapsedMS() const {
        chrono::system_clock::time_point now = chrono::system_clock::now();
        int64_t elapsedMS = chrono::duration_cast<chrono::milliseconds>(now - startTime).count();
        return Float(elapsedMS);
    }

    void done();

  private:
    void printBar();
    int terminalWidth();

    const int64_t totalWork;
    const string title;
    const chrono::system_clock::time_point startTime;
    atomic<int64_t> workDone;
    atomic<bool> exitThread;
    thread updateThread;
};


#ifdef __GNUG__
#define PRINTF_FUNC __attribute__((__format__(__printf__, 1, 2)))
#else
#define PRINTF_FUNC
#endif  // __GNUG__

class Report {
public:
    static void info(const char *format, ...) PRINTF_FUNC;
    static void warning(const char *format, ...) PRINTF_FUNC;
    static void error(const char *format, ...) PRINTF_FUNC;
    static void severe(const char *format, ...) PRINTF_FUNC;

private:
    template <class... Args>
    static string stringVaPrintf(const string format, va_list args) {
        va_list argsCopy;
        va_copy(argsCopy, args);
        size_t size = unsigned(vsnprintf(nullptr, 0, format.c_str(), args)) + 1U;
        string str;
        str.resize(size);
        vsnprintf(&str[0], size, format.c_str(), argsCopy);
        str.pop_back();  // remove trailing NUL
        return str;
    }

    static void processError(Parser::Location *loc, const char *format, va_list args,
                             const char *errorType);
};

#define INFO(args...) Report::info(args)
#define WARNING(args...) Report::warning(args)
#define ERROR(args...) Report::error(args)
#define SEVERE(args...) Report::severe(args)

#ifdef NDEBUG
#define ASSERT(expr) ((void)0)
#else
#define ASSERT(expr) ((expr) ? (void)0 : \
Report::severe("Assertion \"%s\" failed in %s, line %d", expr, __FILE__, __LINE__))
#endif // NDEBUG


#endif // REPORT_H
