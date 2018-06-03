#ifndef STRINGPRINT_H
#define STRINGPRINT_H

#include "utilities.h"
#include <string>
#include <string.h>

#ifdef __GNUG__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif  // __GNUG__

class StringPrint {
public:
    template <typename... Args>
    inline static string printf(const char *fmt, Args... args) {
        string ret;
        printfRecursive(&ret, fmt, args...);
        return ret;
    }

private:
    inline static void printfRecursive(string *s, const char *fmt) {
        const char *c = fmt;
        while (*c) {
            if (*c == '%') {
                // CHECK_EQ(c[1], '%');
                ++c;
            }
            *s += *c++;
        }
    }

    template <typename... Args>
    inline static void printfRecursive(string *s, const char *fmt, double v, Args... args) {
        string nextFmt = copyToFormatString(&fmt, s);
        if (nextFmt == "%f")
            *s += formatOne("%.17g", v);
        else
            *s += formatOne(nextFmt.c_str(), v);
        printfRecursive(s, fmt, args...);
    }

    template <typename T, typename... Args>
    inline static void printfRecursive(string *s, const char *fmt, T v, Args... args) {
        string nextFmt = copyToFormatString(&fmt, s);
        *s += formatOne(nextFmt.c_str(), v);
        printfRecursive(s, fmt, args...);
    }

    template <typename... Args>
    inline static void printfRecursive(string *s, const char *fmt, float v, Args... args) {
        string nextFmt = copyToFormatString(&fmt, s);
        if (nextFmt == "%f")
            *s += formatOne("%.9g", v);
        else
            *s += formatOne(nextFmt.c_str(), v);

        printfRecursive(s, fmt, args...);
    }

    inline static string copyToFormatString(const char **fmt_ptr, string *s) {
        const char *&fmt = *fmt_ptr;
        while (*fmt) {
            if (*fmt != '%') {
                *s += *fmt;
                ++fmt;
            } else if (fmt[1] == '%') {
                // "%%"; let it pass through
                *s += '%';
                *s += '%';
                fmt += 2;
            } else
                // fmt is at the start of a formatting directive.
                break;
        }

        string nextFmt;
        if (*fmt) {
            do {
                nextFmt += *fmt;
                ++fmt;
            } while (*fmt && *fmt != '%' && !isspace(*fmt) && *fmt != ',' &&
                     *fmt != '[' && *fmt != ']' && *fmt != '(' && *fmt != ')');
        }

        return nextFmt;
    }

    template <typename T>
    inline static string formatOne(const char *fmt, T v) {

        size_t size = snprintf(nullptr, 0, fmt, v) + 1;
        string str;
        str.resize(size);
        snprintf(&str[0], size, fmt, v);
        str.pop_back();  // remove trailing NUL
        return str;
    }

};


#ifdef __GNUG__
#pragma GCC diagnostic pop
#endif  // __GNUG__


#endif // STRINGPRINT_H
