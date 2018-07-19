#include "log.h"
#include "stats.h"
#include "parallel.h"
#include "core/renderer.h"

void Log::severe(const char *format, ...) {
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Error");
    va_end(args);
}

void Log::error(const char *format, ...) {
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Error");
    va_end(args);
}

void Log::warning(const char *format, ...) {
    if (Renderer::options.quiet) return;
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Warning");
    va_end(args);
}

void Log::info(const char *format, ...) {
    if (Renderer::options.quiet) return;
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Info");
    va_end(args);
}

void Log::processError(Parser::Location *loc, const char *format, va_list args,
                          const char *errorType)
{
    string errorString;
    // Print line and position in input file, if available
    if (loc)
        errorString = STRING_PRINTF("%s:%d:%d: ", loc->filename.c_str(), loc->line, loc->column);
    errorString += errorType;
    errorString += ": ";
    errorString += STRING_PRINTF(format, args);

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
