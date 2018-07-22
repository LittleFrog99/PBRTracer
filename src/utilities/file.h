#ifndef UTILITY_FILE
#define UTILITY_FILE

#include "utilities.h"

namespace File {

static string searchDirectory;

bool isAbsolutePath(const string &filename);
string absolutePath(const string &filename);
string resolveFilename(const string &filename);
string directoryContaining(const string &filename);
bool readFloatFile(const char *filename, vector<float> *values);

void setSearchDirectory(const string &dirname) {
    searchDirectory = dirname;
}

bool hasExtension(const string &value, const string &ending) {
    if (ending.size() > value.size()) return false;
    return equal(ending.rbegin(), ending.rend(), value.rbegin(),
                      [](char a, char b) { return tolower(a) == tolower(b); });
    }

};

#endif // UTILITY_FILE
