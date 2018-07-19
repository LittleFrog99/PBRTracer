#include "parser.h"
#include "file.h"
#include "stats.h"
#include "memory.h"
#include "core/api.h"
#include "core/renderer.h"
#include "paramset.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

Parser::Location *Parser::loc = nullptr;

STAT_MEMORY_COUNTER("Memory/Tokenizer buffers", tokenizerMemory);

char Parser::decodeEscaped(int ch) {
    switch (ch) {
    case EOF:
        ERROR("premature EOF after character escape '\\'");
        exit(1);
    case 'b':
        return '\b';
    case 'f':
        return '\f';
    case 'n':
        return '\n';
    case 'r':
        return '\r';
    case 't':
        return '\t';
    case '\\':
        return '\\';
    case '\'':
        return '\'';
    case '\"':
        return '\"';
    default:
        ERROR("unexpected escaped character \"%c\"", ch);
        exit(1);
    }
    return 0;  // NOTREACHED
}

unique_ptr<Tokenizer> Tokenizer::createFromFile(
    const string &filename,
    function<void(const char *)> errorCallback) {
    if (filename == "-") {
        // Handle stdin by slurping everything into a string.
        string str;
        int ch;
        while ((ch = getchar()) != EOF) str.push_back((char)ch);
        // make_unique...
        return unique_ptr<Tokenizer>(
            new Tokenizer(move(str), move(errorCallback)));
    }

    int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
        errorCallback(STRING_PRINTF("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    struct stat stat;
    if (fstat(fd, &stat) != 0) {
        errorCallback(STRING_PRINTF("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    size_t len = stat.st_size;
    void *ptr = mmap(nullptr, len, PROT_READ, MAP_FILE | MAP_SHARED, fd, 0);
    if (close(fd) != 0) {
        errorCallback(STRING_PRINTF("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    // return make_unique<Tokenizer>(ptr, len);
    return unique_ptr<Tokenizer>(new Tokenizer(ptr, len, filename, move(errorCallback)));

}


unique_ptr<Tokenizer> Tokenizer::createFromString(
    string str, function<void(const char *)> errorCallback) {
    return unique_ptr<Tokenizer>(new Tokenizer(move(str), move(errorCallback)));
}

Tokenizer::Tokenizer(string str, function<void(const char *)> errorCallback)
    : loc("<stdin>"), errorCallback(move(errorCallback)), contents(move(str))
{
    pos = contents.data();
    end = pos + contents.size();
    tokenizerMemory += contents.size();
}

Tokenizer::Tokenizer(void *ptr, size_t len, string filename, function<void(const char *)> errorCallback)
    : loc(filename), errorCallback(move(errorCallback)), unmapPtr(ptr), unmapLength(len)
{
    pos = (const char *)ptr;
    end = pos + len;
}

string_view Tokenizer::next() {
    while (true) {
        const char *tokenStart = pos;
        int ch = getChar();
        if (ch == EOF)
            return {};
        else if (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r') {
            // nothing
        } else if (ch == '"') {
            // scan to closing quote
            bool haveEscaped = false;
            while ((ch = getChar()) != '"') {
                if (ch == EOF) {
                    errorCallback("premature EOF");
                    return {};
                } else if (ch == '\n') {
                    errorCallback("unterminated string");
                    return {};
                } else if (ch == '\\') {
                    haveEscaped = true;
                    // Grab the next character
                    if ((ch = getChar()) == EOF) {
                        errorCallback("premature EOF");
                        return {};
                    }
                }
            }

            if (!haveEscaped)
                return {tokenStart, size_t(pos - tokenStart)};
            else {
                sEscaped.clear();
                for (const char *p = tokenStart; p < pos; ++p) {
                    if (*p != '\\')
                        sEscaped.push_back(*p);
                    else {
                        ++p;
                        CHECK_LT(p, pos);
                        sEscaped.push_back(Parser::decodeEscaped(*p));
                    }
                }
                return {sEscaped.data(), sEscaped.size()};
            }
        } else if (ch == '[' || ch == ']') {
            return {tokenStart, size_t(1)};
        } else if (ch == '#') {
            // comment: scan to EOL (or EOF)
            while ((ch = getChar()) != EOF) {
                if (ch == '\n' || ch == '\r') {
                    ungetChar();
                    break;
                }
            }

            return {tokenStart, size_t(pos - tokenStart)};
        } else {
            // Regular statement or numeric token; scan until we hit a
            // space, opening quote, or bracket.
            while ((ch = getChar()) != EOF) {
                if (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r' ||
                    ch == '"' || ch == '[' || ch == ']') {
                    ungetChar();
                    break;
                }
            }
            return {tokenStart, size_t(pos - tokenStart)};
        }
    }
}

Tokenizer::~Tokenizer() {
    if (unmapPtr && unmapLength > 0)
        if (munmap(unmapPtr, unmapLength) != 0)
            errorCallback(STRING_PRINTF("munmap: %s", strerror(errno)).c_str());
}

string_view Parser::dequoteString(string_view str) {
    if (!isQuotedString(str)) {
        ERROR("\"%s\": expected quoted string", to_string(str).c_str());
        exit(1);
    }
    str.remove_prefix(1);
    str.remove_suffix(1);
    return str;
}

double Parser::parseNumber(string_view str) {
    // Fast path for a single digit
    if (str.size() == 1) {
        if (!(str[0] >= '0' && str[0] <= '9')) {
            ERROR("\"%c\": expected a number", str[0]);
            exit(1);
        }
        return str[0] - '0';
    }

    // Copy to a buffer so we can NUL-terminate it, as strto[idf]() expect.
    char buf[64];
    char *bufp = buf;
    unique_ptr<char[]> allocBuf;
    if (str.size() + 1 >= sizeof(buf)) {
        // This should be very unusual, but is necessary in case we get a
        // goofball number with lots of leading zeros, for example.
        allocBuf.reset(new char[str.size() + 1]);
        bufp = allocBuf.get();
    }

    copy(str.begin(), str.end(), bufp);
    bufp[str.size()] = '\0';

    // Can we just use strtol?
    auto isInteger = [](string_view str) {
        for (char ch : str)
            if (!(ch >= '0' && ch <= '9')) return false;
        return true;
    };

    char *endptr = nullptr;
    double val;
    if (isInteger(str))
        val = double(strtol(bufp, &endptr, 10));
    else if (sizeof(Float) == sizeof(float))
        val = strtof(bufp, &endptr);
    else
        val = strtod(bufp, &endptr);

    if (val == 0 && endptr == bufp) {
        ERROR("%s: expected a number", to_string(str).c_str());
        exit(1);
    }

    return val;
}

bool Parser::lookupType(const string &decl, int *type, string &sname) {
    *type = 0;
    // Skip leading space
    auto skipSpace = [&decl](string::const_iterator iter) {
        while (iter != decl.end() && (*iter == ' ' || *iter == '\t')) ++iter;
        return iter;
    };
    // Skip to the next whitespace character (or the end of the string).
    auto skipToSpace = [&decl](string::const_iterator iter) {
        while (iter != decl.end() && *iter != ' ' && *iter != '\t') ++iter;
        return iter;
    };

    auto typeBegin = skipSpace(decl.begin());
    if (typeBegin == decl.end()) {
        ERROR("Parameter \"%s\" doesn't have a type declaration?!", decl.c_str());
        return false;
    }

    // Find end of type declaration
    auto typeEnd = skipToSpace(typeBegin);

    string_view typeStr(&(*typeBegin), size_t(typeEnd - typeBegin));
    if (typeStr == "float")
        *type = PARAM_TYPE_FLOAT;
    else if (typeStr == "integer")
        *type = PARAM_TYPE_INT;
    else if (typeStr == "bool")
        *type = PARAM_TYPE_BOOL;
    else if (typeStr == "point2")
        *type = PARAM_TYPE_POINT2;
    else if (typeStr == "vector2")
        *type = PARAM_TYPE_VECTOR2;
    else if (typeStr == "point3")
        *type = PARAM_TYPE_POINT3;
    else if (typeStr == "vector3")
        *type = PARAM_TYPE_VECTOR3;
    else if (typeStr == "point")
        *type = PARAM_TYPE_POINT3;
    else if (typeStr == "vector")
        *type = PARAM_TYPE_VECTOR3;
    else if (typeStr == "normal")
        *type = PARAM_TYPE_NORMAL;
    else if (typeStr == "string")
        *type = PARAM_TYPE_STRING;
    else if (typeStr == "texture")
        *type = PARAM_TYPE_TEXTURE;
    else if (typeStr == "color")
        *type = PARAM_TYPE_RGB;
    else if (typeStr == "rgb")
        *type = PARAM_TYPE_RGB;
    else if (typeStr == "xyz")
        *type = PARAM_TYPE_XYZ;
    else if (typeStr == "blackbody")
        *type = PARAM_TYPE_BLACKBODY;
    else if (typeStr == "spectrum")
        *type = PARAM_TYPE_SPECTRUM;
    else {
        ERROR("Unable to decode type from \"%s\"", decl.c_str());
        return false;
    }

    auto nameBegin = skipSpace(typeEnd);
    if (nameBegin == decl.end()) {
        ERROR("Unable to find parameter name from \"%s\"", decl.c_str());
        return false;
    }
    auto nameEnd = skipToSpace(nameBegin);
    sname = string(nameBegin, nameEnd);

    return true;
}


const char * Parser::paramTypeToName(int type) {
    switch (type) {
    case PARAM_TYPE_INT:
        return "int";
    case PARAM_TYPE_BOOL:
        return "bool";
    case PARAM_TYPE_FLOAT:
        return "float";
    case PARAM_TYPE_POINT2:
        return "point2";
    case PARAM_TYPE_VECTOR2:
        return "vector2";
    case PARAM_TYPE_POINT3:
        return "point3";
    case PARAM_TYPE_VECTOR3:
        return "vector3";
    case PARAM_TYPE_NORMAL:
        return "normal";
    case PARAM_TYPE_RGB:
        return "rgb/color";
    case PARAM_TYPE_XYZ:
        return "xyz";
    case PARAM_TYPE_BLACKBODY:
        return "blackbody";
    case PARAM_TYPE_SPECTRUM:
        return "spectrum";
    case PARAM_TYPE_STRING:
        return "string";
    case PARAM_TYPE_TEXTURE:
        return "texture";
    default:
        LOG(FATAL) << "Error in paramTypeToName";
        return nullptr;
    }
}

void Parser::addParam(ParamSet &ps, const ParamListItem &item, SpectrumType spectrumType) {
    int type;
    string name;
    if (lookupType(item.name, &type, name)) {
        if (type == PARAM_TYPE_TEXTURE || type == PARAM_TYPE_STRING ||
            type == PARAM_TYPE_BOOL) {
            if (!item.stringValues) {
                ERROR(
                    "Expected string parameter value for parameter "
                    "\"%s\" with type \"%s\". Ignoring.",
                    name.c_str(), paramTypeToName(type));
                return;
            }
        } else if (type !=
                   PARAM_TYPE_SPECTRUM) { /* spectrum can be either... */
            if (item.stringValues) {
                ERROR(
                    "Expected numeric parameter value for parameter "
                    "\"%s\" with type \"%s\".  Ignoring.",
                    name.c_str(), paramTypeToName(type));
                return;
            }
        }

        int nItems = item.size;
        if (type == PARAM_TYPE_INT) {
            // parser doesn't handle ints, so convert from doubles here....
            int nAlloc = nItems;
            unique_ptr<int[]> idata(new int[nAlloc]);
            for (int j = 0; j < nAlloc; ++j)
                idata[j] = int(item.doubleValues[j]);
            ps.addInt(name, move(idata), nItems);
        } else if (type == PARAM_TYPE_BOOL) {
            // strings -> bools
            int nAlloc = item.size;
            unique_ptr<bool[]> bdata(new bool[nAlloc]);
            for (int j = 0; j < nAlloc; ++j) {
                string s(item.stringValues[j]);
                if (s == "true")
                    bdata[j] = true;
                else if (s == "false")
                    bdata[j] = false;
                else {
                    WARNING(
                        "Value \"%s\" unknown for Boolean parameter \"%s\"."
                        "Using \"false\".",
                        s.c_str(), item.name.c_str());
                    bdata[j] = false;
                }
            }
            ps.addBool(name, move(bdata), nItems);
        } else if (type == PARAM_TYPE_FLOAT) {
            unique_ptr<Float[]> floats(new Float[nItems]);
            for (int i = 0; i < nItems; ++i) floats[i] = item.doubleValues[i];
            ps.addFloat(name, move(floats), nItems);
        } else if (type == PARAM_TYPE_POINT2) {
            if ((nItems % 2) != 0)
                WARNING(
                    "Excess values given with point2 parameter \"%s\". "
                    "Ignoring last one of them.",
                    item.name.c_str());
            unique_ptr<Point2f[]> pts(new Point2f[nItems / 2]);
            for (int i = 0; i < nItems / 2; ++i) {
                pts[i].x = item.doubleValues[2 * i];
                pts[i].y = item.doubleValues[2 * i + 1];
            }
            ps.addPoint2f(name, move(pts), nItems / 2);
        } else if (type == PARAM_TYPE_VECTOR2) {
            if ((nItems % 2) != 0)
                WARNING(
                    "Excess values given with vector2 parameter \"%s\". "
                    "Ignoring last one of them.",
                    item.name.c_str());
            unique_ptr<Vector2f[]> vecs(new Vector2f[nItems / 2]);
            for (int i = 0; i < nItems / 2; ++i) {
                vecs[i].x = item.doubleValues[2 * i];
                vecs[i].y = item.doubleValues[2 * i + 1];
            }
            ps.addVector2f(name, move(vecs), nItems / 2);
        } else if (type == PARAM_TYPE_POINT3) {
            if ((nItems % 3) != 0)
                WARNING(
                    "Excess values given with point3 parameter \"%s\". "
                    "Ignoring last %d of them.",
                    item.name.c_str(), nItems % 3);
            unique_ptr<Point3f[]> pts(new Point3f[nItems / 3]);
            for (int i = 0; i < nItems / 3; ++i) {
                pts[i].x = item.doubleValues[3 * i];
                pts[i].y = item.doubleValues[3 * i + 1];
                pts[i].z = item.doubleValues[3 * i + 2];
            }
            ps.addPoint3f(name, move(pts), nItems / 3);
        } else if (type == PARAM_TYPE_VECTOR3) {
            if ((nItems % 3) != 0)
                WARNING(
                    "Excess values given with vector3 parameter \"%s\". "
                    "Ignoring last %d of them.",
                    item.name.c_str(), nItems % 3);
            unique_ptr<Vector3f[]> vecs(new Vector3f[nItems / 3]);
            for (int j = 0; j < nItems / 3; ++j) {
                vecs[j].x = item.doubleValues[3 * j];
                vecs[j].y = item.doubleValues[3 * j + 1];
                vecs[j].z = item.doubleValues[3 * j + 2];
            }
            ps.addVector3f(name, move(vecs), nItems / 3);
        } else if (type == PARAM_TYPE_NORMAL) {
            if ((nItems % 3) != 0)
                WARNING(
                    "Excess values given with \"normal\" parameter \"%s\". "
                    "Ignoring last %d of them.",
                    item.name.c_str(), nItems % 3);
            unique_ptr<Normal3f[]> normals(new Normal3f[nItems / 3]);
            for (int j = 0; j < nItems / 3; ++j) {
                normals[j].x = item.doubleValues[3 * j];
                normals[j].y = item.doubleValues[3 * j + 1];
                normals[j].z = item.doubleValues[3 * j + 2];
            }
            ps.addNormal3f(name, move(normals), nItems / 3);
        } else if (type == PARAM_TYPE_RGB) {
            if ((nItems % 3) != 0) {
                WARNING(
                    "Excess RGB values given with parameter \"%s\". "
                    "Ignoring last %d of them",
                    item.name.c_str(), nItems % 3);
                nItems -= nItems % 3;
            }
            unique_ptr<Float[]> floats(new Float[nItems]);
            for (int j = 0; j < nItems; ++j) floats[j] = item.doubleValues[j];
            ps.addRGBSpectrum(name, move(floats), nItems);
        } else if (type == PARAM_TYPE_XYZ) {
            if ((nItems % 3) != 0) {
                WARNING(
                    "Excess XYZ values given with parameter \"%s\". "
                    "Ignoring last %d of them",
                    item.name.c_str(), nItems % 3);
                nItems -= nItems % 3;
            }
            unique_ptr<Float[]> floats(new Float[nItems]);
            for (int j = 0; j < nItems; ++j) floats[j] = item.doubleValues[j];
            ps.addXYZSpectrum(name, move(floats), nItems);
        } else if (type == PARAM_TYPE_BLACKBODY) {
            if ((nItems % 2) != 0) {
                WARNING(
                    "Excess value given with blackbody parameter \"%s\". "
                    "Ignoring extra one.",
                    item.name.c_str());
                nItems -= nItems % 2;
            }
            unique_ptr<Float[]> floats(new Float[nItems]);
            for (int j = 0; j < nItems; ++j) floats[j] = item.doubleValues[j];
            ps.addBlackbodySpectrum(name, move(floats), nItems);
        } else if (type == PARAM_TYPE_SPECTRUM) {
            if (item.stringValues) {
                ps.addSampledSpectrumFiles(name, item.stringValues, nItems);
            } else {
                if ((nItems % 2) != 0) {
                    WARNING(
                        "Non-even number of values given with sampled "
                        "spectrum "
                        "parameter \"%s\". Ignoring extra.",
                        item.name.c_str());
                    nItems -= nItems % 2;
                }
                unique_ptr<Float[]> floats(new Float[nItems]);
                for (int j = 0; j < nItems; ++j)
                    floats[j] = item.doubleValues[j];
                ps.addSampledSpectrum(name, move(floats), nItems);
            }
        } else if (type == PARAM_TYPE_STRING) {
            unique_ptr<string[]> strings(new string[nItems]);
            for (int j = 0; j < nItems; ++j)
                strings[j] = string(item.stringValues[j]);
            ps.addString(name, move(strings), nItems);
        } else if (type == PARAM_TYPE_TEXTURE) {
            if (nItems == 1) {
                string val(*item.stringValues);
                ps.addTexture(name, val);
            } else
                ERROR(
                    "Only one string allowed for \"texture\" parameter "
                    "\"%s\"",
                    name.c_str());
        }
    } else
        WARNING("Type of parameter \"%s\" is unknown", item.name.c_str());
}

template <typename Next, typename Unget>
ParamSet Parser::parseParams(Next nextToken, Unget ungetToken, MemoryArena &arena,
                             SpectrumType spectrumType)
{
    ParamSet ps;
    while (true) {
        string_view decl = nextToken(TOKEN_OPTIONAL);
        if (decl.empty()) return ps;

        if (!isQuotedString(decl)) {
            ungetToken(decl);
            return ps;
        }

        ParamListItem item;
        item.name = to_string(dequoteString(decl));
        size_t nAlloc = 0;

        auto addVal = [&](string_view val) {
            if (isQuotedString(val)) {
                if (item.doubleValues) {
                    ERROR("mixed string and numeric parameters");
                    exit(1);
                }
                if (item.size == nAlloc) {
                    nAlloc = max<size_t>(2 * item.size, 4);
                    const char **newData = arena.alloc<const char *>(nAlloc);
                    copy(item.stringValues, item.stringValues + item.size,
                              newData);
                    item.stringValues = newData;
                }

                val = dequoteString(val);
                char *buf = arena.alloc<char>(val.size() + 1);
                memcpy(buf, val.data(), val.size());
                buf[val.size()] = '\0';
                item.stringValues[item.size++] = buf;
            } else {
                if (item.stringValues) {
                    ERROR("mixed string and numeric parameters");
                    exit(1);
                }

                if (item.size == nAlloc) {
                    nAlloc = max<size_t>(2 * item.size, 4);
                    double *newData = arena.alloc<double>(nAlloc);
                    copy(item.doubleValues, item.doubleValues + item.size,
                              newData);
                    item.doubleValues = newData;
                }
                item.doubleValues[item.size++] = parseNumber(val);
            }
        };

        string_view val = nextToken(TOKEN_REQUIRED);

        if (val == "[") {
            while (true) {
                val = nextToken(TOKEN_REQUIRED);
                if (val == "]") break;
                addVal(val);
            }
        } else {
            addVal(val);
        }

        addParam(ps, item, spectrumType);
        arena.reset();
    }

    return ps;
}

void Parser::parse(unique_ptr<Tokenizer> t) {
    vector<unique_ptr<Tokenizer>> fileStack;
    fileStack.push_back(move(t));
    loc = &fileStack.back()->loc;

    bool ungetTokenSet = false;
    string ungetTokenValue;

    // nextToken is a little helper function that handles the file stack,
    // returning the next token from the current file until reaching EOF,
    // at which point it switches to the next file (if any).
    function<string_view(int)> nextToken = [&](int flags) -> string_view {
        if (ungetTokenSet) {
            ungetTokenSet = false;
            return string_view(ungetTokenValue.data(), ungetTokenValue.size());
        }

        if (fileStack.empty()) {
            if (flags & TOKEN_REQUIRED) {
                ERROR("premature EOF");
                exit(1);
            }
            loc = nullptr;
            return {};
        }

        string_view tok = fileStack.back()->next();

        if (tok.empty()) {
            // We've reached EOF in the current file. Anything more to parse?
            fileStack.pop_back();
            if (!fileStack.empty()) loc = &fileStack.back()->loc;
            return nextToken(flags);
        } else if (tok == "Include") {
            // Switch to the given file.
            string filename =
                to_string(dequoteString(nextToken(TOKEN_REQUIRED)));
            filename = File::absolutePath(File::resolveFilename(filename));
            auto tokError = [](const char *msg) { ERROR("%s", msg); };
            unique_ptr<Tokenizer> tinc =
                Tokenizer::createFromFile(filename, tokError);
            if (tinc) {
                fileStack.push_back(move(tinc));
                loc = &fileStack.back()->loc;
            }
            return nextToken(flags);
        } else if (tok[0] == '#') {
            // Swallow comments, unless --cat or --toply was given, in
            // which case they're printed to stdout.
            if (Renderer::options.cat || Renderer::options.toPly)
                printf("%*s%s\n", Renderer::catIndentCount, "", to_string(tok).c_str());
            return nextToken(flags);
        } else
            // Regular token; success.
            return tok;
    };

    auto ungetToken = [&](string_view s) {
        CHECK(!ungetTokenSet);
        ungetTokenValue = string(s.data(), s.size());
        ungetTokenSet = true;
    };

    MemoryArena arena;

    // Helper function for pbrt API entrypoints that take a single string
    // parameter and a ParamSet (e.g. pbrtShape()).
    auto basicParamListEntrypoint = [&](
        SpectrumType spectrumType,
        function<void(const string &n, ParamSet p)> apiFunc) {
        string_view token = nextToken(TOKEN_REQUIRED);
        string_view dequoted = dequoteString(token);
        string n = to_string(dequoted);
        ParamSet params =
            parseParams(nextToken, ungetToken, arena, spectrumType);
        apiFunc(n, move(params));
    };

    auto syntaxError = [&](string_view tok) {
        ERROR("Unexpected token: %s", to_string(tok).c_str());
        exit(1);
    };

    while (true) {
        string_view tok = nextToken(TOKEN_OPTIONAL);
        if (tok.empty()) break;

        switch (tok[0]) {
        case 'A':
            if (tok == "AttributeBegin")
                API::attributeBegin();
            else if (tok == "AttributeEnd")
                API::attributeEnd();
            else if (tok == "ActiveTransform") {
                string_view a = nextToken(TOKEN_REQUIRED);
                if (a == "All")
                    API::activeTransformAll();
                else if (a == "EndTime")
                    API::activeTransformEndTime();
                else if (a == "StartTime")
                    API::activeTransformStartTime();
                else
                    syntaxError(tok);
            } else if (tok == "AreaLightSource")
                basicParamListEntrypoint(SpectrumType::Illuminant, API::areaLightSource);
            else if (tok == "Accelerator")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::accelerator);
            else
                syntaxError(tok);
            break;

        case 'C':
            if (tok == "ConcatTransform") {
                if (nextToken(TOKEN_REQUIRED) != "[") syntaxError(tok);
                Float m[16];
                for (int i = 0; i < 16; ++i)
                    m[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                if (nextToken(TOKEN_REQUIRED) != "]") syntaxError(tok);
                API::concatTransform(m);
            } else if (tok == "CoordinateSystem") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                API::coordinateSystem(to_string(n));
            } else if (tok == "CoordSysTransform") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                API::coordSysTransform(to_string(n));
            } else if (tok == "Camera")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::camera);
            else
                syntaxError(tok);
            break;

        case 'F':
            if (tok == "Film")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::film);
            else
                syntaxError(tok);
            break;

        case 'I':
            if (tok == "Integrator")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::integrator);
            else if (tok == "Identity")
                API::identity();
            else
                syntaxError(tok);
            break;

        case 'L':
            if (tok == "LightSource")
                basicParamListEntrypoint(SpectrumType::Illuminant, API::lightSource);
            else if (tok == "LookAt") {
                Float v[9];
                for (int i = 0; i < 9; ++i)
                    v[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                API::lookAt(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
            } else
                syntaxError(tok);
            break;

        case 'M':
            if (tok == "MakeNamedMaterial")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::makeNamedMaterial);
            else if (tok == "MakeNamedMedium")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::makeNamedMedium);
            else if (tok == "Material")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::material);
            else if (tok == "MediumInterface") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                string names[2];
                names[0] = to_string(n);

                // Check for optional second parameter
                string_view second = nextToken(TOKEN_OPTIONAL);
                if (!second.empty()) {
                    if (isQuotedString(second))
                        names[1] = to_string(dequoteString(second));
                    else {
                        ungetToken(second);
                        names[1] = names[0];
                    }
                } else
                    names[1] = names[0];

                API::mediumInterface(names[0], names[1]);
            } else
                syntaxError(tok);
            break;

        case 'N':
            if (tok == "NamedMaterial") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                API::namedMaterial(to_string(n));
            } else
                syntaxError(tok);
            break;

        case 'O':
            if (tok == "ObjectBegin") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                API::objectBegin(to_string(n));
            } else if (tok == "ObjectEnd")
                API::objectEnd();
            else if (tok == "ObjectInstance") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                API::objectInstance(to_string(n));
            } else
                syntaxError(tok);
            break;

        case 'P':
            if (tok == "PixelFilter")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::pixelFilter);
            else
                syntaxError(tok);
            break;

        case 'R':
            if (tok == "ReverseOrientation")
                API::reverseOrientation();
            else if (tok == "Rotate") {
                Float v[4];
                for (int i = 0; i < 4; ++i)
                    v[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                API::rotate(v[0], v[1], v[2], v[3]);
            } else
                syntaxError(tok);
            break;

        case 'S':
            if (tok == "Shape")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::shape);
            else if (tok == "Sampler")
                basicParamListEntrypoint(SpectrumType::Reflectance, API::sampler);
            else if (tok == "Scale") {
                Float v[3];
                for (int i = 0; i < 3; ++i)
                    v[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                API::scale(v[0], v[1], v[2]);
            } else
                syntaxError(tok);
            break;

        case 'T':
            if (tok == "TransformBegin")
                API::transformBegin();
            else if (tok == "TransformEnd")
                API::transformEnd();
            else if (tok == "Transform") {
                if (nextToken(TOKEN_REQUIRED) != "[") syntaxError(tok);
                Float m[16];
                for (int i = 0; i < 16; ++i)
                    m[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                if (nextToken(TOKEN_REQUIRED) != "]") syntaxError(tok);
                API::transform(m);
            } else if (tok == "Translate") {
                Float v[3];
                for (int i = 0; i < 3; ++i)
                    v[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                API::translate(v[0], v[1], v[2]);
            } else if (tok == "TransformTimes") {
                Float v[2];
                for (int i = 0; i < 2; ++i)
                    v[i] = parseNumber(nextToken(TOKEN_REQUIRED));
                API::transformTimes(v[0], v[1]);
            } else if (tok == "Texture") {
                string_view n = dequoteString(nextToken(TOKEN_REQUIRED));
                string name = to_string(n);
                n = dequoteString(nextToken(TOKEN_REQUIRED));
                string type = to_string(n);

                basicParamListEntrypoint(
                    SpectrumType::Reflectance,
                    [&](const string &texName, const ParamSet &params) {
                        API::texture(name, type, texName, params);
                    });
            } else
                syntaxError(tok);
            break;

        case 'W':
            if (tok == "WorldBegin")
                API::worldBegin();
            else if (tok == "WorldEnd")
                API::worldEnd();
            else
                syntaxError(tok);
            break;

        default:
            syntaxError(tok);
        }
    }
}

void API::parseFile(string filename) {
    if (filename != "-") File::setSearchDirectory(File::directoryContaining(filename));
    auto tokError = [](const char *msg) { ERROR("%s", msg); exit(1); };
    unique_ptr<Tokenizer> t = Tokenizer::createFromFile(filename, tokError);
    if (!t) return;
    Parser::parse(move(t));
}

void API::parseString(string str) {
    auto tokError = [](const char *msg) { ERROR("%s", msg); exit(1); };
    unique_ptr<Tokenizer> t = Tokenizer::createFromString(move(str), tokError);
    if (!t) return;
    Parser::parse(move(t));
}

