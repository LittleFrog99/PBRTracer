#include "error.h"

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
    // if (PbrtOptions.quiet) return;
    va_list args;
    va_start(args, format);
    processError(Parser::loc, format, args, "Warning");
    va_end(args);
}

void Report::info(const char *format, ...) {
    // if (PbrtOptions.quiet) return;
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
        errorString = StringPrint::printf("%s:%d:%d: ", loc->filename.c_str(),
                                          loc->line, loc->column);
    errorString += errorType;
    errorString += ": ";
    errorString += StringPrint::printf(format, args);

    // Print the error message (but not more than one time).
    static std::string lastError;
    static std::mutex mutex;
    lock_guard<std::mutex> lock(mutex);
    if (errorString != lastError) {
        // LOG(INFO) << errorString;
        fprintf(stderr, "%s\n", errorString.c_str());
        lastError = errorString;
    }
}
