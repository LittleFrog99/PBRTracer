#ifndef UTILITY_LOG
#define UTILITY_LOG

#include "stringprint.h"
#include "core/parser.h"
#include <thread>


#ifdef __GNUG__
#define PRINTF_FUNC __attribute__((__format__(__printf__, 1, 2)))
#else
#define PRINTF_FUNC
#endif  // __GNUG__

class Log {
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

#define INFO(args...) Log::info(args)
#define WARNING(args...) Log::warning(args)
#define ERROR(args...) Log::error(args)
#define SEVERE(args...) Log::severe(args)

#ifdef NDEBUG
#define ASSERT(expr) ((void)0)
#else
#define ASSERT(expr) ((expr) ? (void)0 : \
Log::severe("Assertion \"%s\" failed in %s, line %d", expr, __FILE__, __LINE__))
#endif // NDEBUG


#endif // UTILITY_LOG
