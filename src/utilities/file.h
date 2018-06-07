#ifndef FILE_H
#define FILE_H

#include "utilities.h"

class File {
public:
    static bool isAbosolutePath(const string &filename);
    static string absolutePath(const string &filename);
    static string resolveFilename(const string &filename);
    static string directoryContaining(const string &filename);
    static bool readFloatFile(const char *filename, vector<Float> *values);

    inline static void setSearchDirectory(const string &dirname) {
        searchDirectory = dirname;
    }

    inline static bool hasExtension(const string &value, const string &ending) {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin(),
                          [](char a, char b) { return std::tolower(a) == std::tolower(b); });
    }

private:
    static string searchDirectory;
};

#endif // FILE_H
