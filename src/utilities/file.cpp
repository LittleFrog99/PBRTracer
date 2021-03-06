#include "file.h"
#include "log.h"
#include <libgen.h>

namespace File {

static string searchDirectory;

void setSearchDirectory(const string &dirname) {
    searchDirectory = dirname;
}

bool isAbsolutePath(const string &filename) {
    return (filename.size() > 0) && filename[0] == '/';
}

string absolutePath(const string &filename) {
    char full[PATH_MAX];
    if (realpath(filename.c_str(), full))
        return string(full);
    else
        return filename;
}

string resolveFilename(const string &filename) {
    if (searchDirectory.empty() || filename.empty())
        return filename;
    else if (File::isAbsolutePath(filename))
        return filename;
    else if (searchDirectory[searchDirectory.size() - 1] == '/')
        return searchDirectory + filename;
    else
        return searchDirectory + "/" + filename;
}

string directoryContaining(const string &filename) {
    char *t = strdup(filename.c_str());
    string result = dirname(t);
    free(t);
    return result;
}

bool readFloatFile(const char *filename, std::vector<float> *values) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        ERROR("Unable to open file \"%s\"", filename);
        return false;
    }

    int c;
    bool inNumber = false;
    char curNumber[32];
    int curNumberPos = 0;
    int lineNumber = 1;
    while ((c = getc(f)) != EOF) {
        if (c == '\n') ++lineNumber;
        if (inNumber) {
            CHECK_LT(curNumberPos, sizeof(curNumber));
            if (isdigit(c) || c == '.' || c == 'e' || c == '-' || c == '+')
                curNumber[curNumberPos++] = c;
            else {
                curNumber[curNumberPos++] = '\0';
                values->push_back(atof(curNumber));
                inNumber = false;
                curNumberPos = 0;
            }
        } else {
            if (isdigit(c) || c == '.' || c == '-' || c == '+') {
                inNumber = true;
                curNumber[curNumberPos++] = c;
            } else if (c == '#') {
                while ((c = getc(f)) != '\n' && c != EOF)
                    ;
                ++lineNumber;
            } else if (!isspace(c))
                WARNING("Unexpected text found at line %d of float file \"%s\"",
                        lineNumber, filename);
        }
    }
    fclose(f);
    return true;
}

};
