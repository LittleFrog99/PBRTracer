#ifndef PARSER_H
#define PARSER_H

#include "utilities.h"
#include <string_view>

class Parser {
public:
    struct Location {
        Location() = default;
        Location(const string &filename) : filename(filename) {}

        string filename;
        int line = 1, column = 0;
    };

    static Location *loc;
};

#endif // PARSER_H
