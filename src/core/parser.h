#ifndef CORE_PARSER
#define CORE_PARSER

#include "utilities.h"
#include <string_view>

namespace std {
inline string to_string(string_view str) {
    return string(str.data(), str.size());
}
}

class ParamSet;
class MemoryArena;
class Tokenizer;
enum class SpectrumType;

class Parser {
public:
    struct Location {
        Location() = default;
        Location(const string &filename) : filename(filename) {}

        string filename;
        int line = 1, column = 0;
    };

    struct ParamListItem {
        string name;
        double *doubleValues = nullptr;
        const char **stringValues = nullptr;
        size_t size = 0;
        bool isString = false;
    };

    enum {
        PARAM_TYPE_INT,
        PARAM_TYPE_BOOL,
        PARAM_TYPE_FLOAT,
        PARAM_TYPE_POINT2,
        PARAM_TYPE_VECTOR2,
        PARAM_TYPE_POINT3,
        PARAM_TYPE_VECTOR3,
        PARAM_TYPE_NORMAL,
        PARAM_TYPE_RGB,
        PARAM_TYPE_XYZ,
        PARAM_TYPE_BLACKBODY,
        PARAM_TYPE_SPECTRUM,
        PARAM_TYPE_STRING,
        PARAM_TYPE_TEXTURE
    };

    static char decodeEscaped(int ch);
    static void parse(unique_ptr<Tokenizer> t);

    static Location *loc;
    static constexpr int TOKEN_OPTIONAL = 0;
    static constexpr int TOKEN_REQUIRED = 1;

private:
    static double parseNumber(string_view str);
    static bool lookupType(const string &decl, int *type, string &sname);
    static const char * paramTypeToName(int type);
    static void addParam(ParamSet &ps, const ParamListItem &item, SpectrumType spectrumType);

    template <typename Next, typename Unget>
    static ParamSet parseParams(Next nextToken, Unget ungetToken, MemoryArena &arena,
                                SpectrumType spectrumType);

    static string_view dequoteString(string_view str);
    static bool isQuotedString(string_view str) {
        return str.size() >= 3 && str[0] == '"' && str.back() == '"';
    }

};


class Tokenizer {
public:
    static unique_ptr<Tokenizer> createFromFile(const string &filename,
                                                function<void(const char *)> errorCallback);
    static unique_ptr<Tokenizer> createFromString(string str,
                                                  function<void(const char *)> errorCallback);
    ~Tokenizer();
    string_view next();

    Parser::Location loc;

private:
    Tokenizer(string str, function<void(const char *)> errorCallback);
    Tokenizer(void *ptr, size_t len, string filename, function<void(const char *)> errorCallback);

    int getChar() {
        if (pos == end) return EOF;
        int ch = *pos++;
        if (ch == '\n') {
            ++loc.line;
            loc.column = 0;
        } else
            ++loc.column;
        return ch;
    }

    void ungetChar() {
        --pos;
        if (*pos == '\n')
            --loc.line;
    }

    function<void(const char *)> errorCallback;
    void *unmapPtr = nullptr;
    size_t unmapLength = 0;
    string contents;
    const char *pos, *end;
    string sEscaped;
};

#endif // CORE_PARSER
