#include "paramset.h"
#include "report.h"
#include "file.h"

// ParamSet Macros
#define ADD_PARAM_TYPE(T, vec) \
    (vec).emplace_back(new ParamSetItem<T>(name, std::move(values), nValues));

#define LOOKUP_PTR(vec)             \
    for (const auto &v : vec)       \
        if (v->name == name) {      \
            *nValues = v->nValues;  \
            v->lookedUp = true;     \
            return v->values.get(); \
        }                           \
    return nullptr

#define LOOKUP_ONE(vec)                           \
    for (const auto &v : vec)                     \
        if (v->name == name && v->nValues == 1) { \
            v->lookedUp = true;                   \
            return v->values[0];                  \
        }                                         \
    return d

// ParamSet Methods
void ParamSet::addFloat(const string &name, unique_ptr<Float[]> values, int nValues) {
    eraseFloat(name);
    ADD_PARAM_TYPE(Float, floats);
}

void ParamSet::addInt(const string &name, unique_ptr<int[]> values, int nValues) {
    eraseInt(name);
    ADD_PARAM_TYPE(int, ints);
}

void ParamSet::addBool(const string &name, unique_ptr<bool[]> values, int nValues) {
    eraseBool(name);
    ADD_PARAM_TYPE(bool, bools);
}

void ParamSet::addPoint2f(const string &name, unique_ptr<Point2f[]> values, int nValues) {
    erasePoint2f(name);
    ADD_PARAM_TYPE(Point2f, point2fs);
}

void ParamSet::addVector2f(const string &name, unique_ptr<Vector2f[]> values, int nValues) {
    eraseVector2f(name);
    ADD_PARAM_TYPE(Vector2f, vector2fs);
}

void ParamSet::addPoint3f(const string &name, unique_ptr<Point3f[]> values, int nValues) {
    erasePoint3f(name);
    ADD_PARAM_TYPE(Point3f, point3fs);
}

void ParamSet::addVector3f(const string &name, unique_ptr<Vector3f[]> values, int nValues) {
    eraseVector3f(name);
    ADD_PARAM_TYPE(Vector3f, vector3fs);
}

void ParamSet::addNormal3f(const string &name, unique_ptr<Normal3f[]> values, int nValues) {
    eraseNormal3f(name);
    ADD_PARAM_TYPE(Normal3f, normals);
}

void ParamSet::addRGBSpectrum(const string &name,
                              unique_ptr<Float[]> values, int nValues) {
    eraseSpectrum(name);
    nValues /= 3;
    unique_ptr<RGBSpectrum[]> s(new RGBSpectrum[nValues]);
    for (int i = 0; i < nValues; ++i) s[i] = RGBSpectrum::fromRGB(&values[3 * i]);
    shared_ptr<ParamSetItem<RGBSpectrum>> psi(
        new ParamSetItem<RGBSpectrum>(name, std::move(s), nValues));
    spectra.push_back(psi);
}

void ParamSet::addXYZSpectrum(const string &name,
                              unique_ptr<Float[]> values, int nValues) {
    eraseSpectrum(name);
    nValues /= 3;
    unique_ptr<RGBSpectrum[]> s(new RGBSpectrum[nValues]);
    for (int i = 0; i < nValues; ++i) s[i] = RGBSpectrum::fromXYZ(&values[3 * i]);
    shared_ptr<ParamSetItem<RGBSpectrum>> psi(
        new ParamSetItem<RGBSpectrum>(name, std::move(s), nValues));
    spectra.push_back(psi);
}

void ParamSet::addBlackbodySpectrum(const string &name, unique_ptr<Float[]> values,
                                    int nValues) {
    eraseSpectrum(name);
    nValues /= 2;
    unique_ptr<RGBSpectrum[]> s(new RGBSpectrum[nValues]);
    unique_ptr<Float[]> v(new Float[Spectrum::NUM_CIE_SAMPLES]);
    for (int i = 0; i < nValues; ++i) {
        BlackbodyNormalized(Spectrum::CIE_LAMBDA, Spectrum::NUM_CIE_SAMPLES, values[2 * i], v.get());
        s[i] = values[2 * i + 1] *
               RGBSpectrum::fromSampled(Spectrum::CIE_LAMBDA, v.get(), Spectrum::NUM_CIE_SAMPLES);
    }
    shared_ptr<ParamSetItem<RGBSpectrum>> psi(
        new ParamSetItem<RGBSpectrum>(name, std::move(s), nValues));
    spectra.push_back(psi);
}

void ParamSet::addSampledSpectrum(const string &name,
                                  unique_ptr<Float[]> values,
                                  int nValues) {
    eraseSpectrum(name);
    nValues /= 2;
    unique_ptr<Float[]> wl(new Float[nValues]);
    unique_ptr<Float[]> v(new Float[nValues]);
    for (int i = 0; i < nValues; ++i) {
        wl[i] = values[2 * i];
        v[i] = values[2 * i + 1];
    }
    unique_ptr<Spectrum[]> s(new Spectrum[1]);
    s[0] = Spectrum::FromSampled(wl.get(), v.get(), nValues);
    shared_ptr<ParamSetItem<Spectrum>> psi(
        new ParamSetItem<Spectrum>(name, std::move(s), 1));
    spectra.push_back(psi);
}

void ParamSet::addSampledSpectrumFiles(const string &name,
                                       const char **names, int nValues) {
    eraseSpectrum(name);
    unique_ptr<Spectrum[]> s(new Spectrum[nValues]);
    for (int i = 0; i < nValues; ++i) {
        string fn = File::absolutePath(File::resolveFilename(names[i]));
        if (cachedSpectra.find(fn) != cachedSpectra.end()) {
            s[i] = cachedSpectra[fn];
            continue;
        }

        std::vector<Float> vals;
        if (!File::readFloatFile(fn.c_str(), &vals)) {
            Report::warning(
                "Unable to read SPD file \"%s\".  Using black distribution.",
                fn.c_str());
            s[i] = Spectrum(0.);
        } else {
            if (vals.size() % 2) {
                Report::warning(
                    "Extra value found in spectrum file \"%s\". "
                    "Ignoring it.",
                    fn.c_str());
            }
            std::vector<Float> wls, v;
            for (size_t j = 0; j < vals.size() / 2; ++j) {
                wls.push_back(vals[2 * j]);
                v.push_back(vals[2 * j + 1]);
            }
            s[i] = Spectrum::FromSampled(&wls[0], &v[0], wls.size());
        }
        cachedSpectra[fn] = s[i];
    }

    shared_ptr<ParamSetItem<Spectrum>> psi(
        new ParamSetItem<Spectrum>(name, std::move(s), nValues));
    spectra.push_back(psi);
}

map<string, Spectrum> ParamSet::cachedSpectra;
void ParamSet::addString(const string &name,
                         unique_ptr<string[]> values, int nValues) {
    eraseString(name);
    ADD_PARAM_TYPE(string, strings);
}

void ParamSet::addTexture(const string &name, const string &value) {
    eraseTexture(name);
    unique_ptr<string[]> str(new string[1]);
    str[0] = value;
    shared_ptr<ParamSetItem<string>> psi(
        new ParamSetItem<string>(name, std::move(str), 1));
    textures.push_back(psi);
}

bool ParamSet::eraseInt(const string &n) {
    for (size_t i = 0; i < ints.size(); ++i)
        if (ints[i]->name == n) {
            ints.erase(ints.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseBool(const string &n) {
    for (size_t i = 0; i < bools.size(); ++i)
        if (bools[i]->name == n) {
            bools.erase(bools.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseFloat(const string &n) {
    for (size_t i = 0; i < floats.size(); ++i)
        if (floats[i]->name == n) {
            floats.erase(floats.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::erasePoint2f(const string &n) {
    for (size_t i = 0; i < point2fs.size(); ++i)
        if (point2fs[i]->name == n) {
            point2fs.erase(point2fs.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseVector2f(const string &n) {
    for (size_t i = 0; i < vector2fs.size(); ++i)
        if (vector2fs[i]->name == n) {
            vector2fs.erase(vector2fs.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::erasePoint3f(const string &n) {
    for (size_t i = 0; i < point3fs.size(); ++i)
        if (point3fs[i]->name == n) {
            point3fs.erase(point3fs.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseVector3f(const string &n) {
    for (size_t i = 0; i < vector3fs.size(); ++i)
        if (vector3fs[i]->name == n) {
            vector3fs.erase(vector3fs.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseNormal3f(const string &n) {
    for (size_t i = 0; i < normals.size(); ++i)
        if (normals[i]->name == n) {
            normals.erase(normals.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseSpectrum(const string &n) {
    for (size_t i = 0; i < spectra.size(); ++i)
        if (spectra[i]->name == n) {
            spectra.erase(spectra.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseString(const string &n) {
    for (size_t i = 0; i < strings.size(); ++i)
        if (strings[i]->name == n) {
            strings.erase(strings.begin() + i);
            return true;
        }
    return false;
}

bool ParamSet::eraseTexture(const string &n) {
    for (size_t i = 0; i < textures.size(); ++i)
        if (textures[i]->name == n) {
            textures.erase(textures.begin() + i);
            return true;
        }
    return false;
}

Float ParamSet::findOneFloat(const string &name, Float d) const {
    for (const auto &f : floats)
        if (f->name == name && f->nValues == 1) {
            f->lookedUp = true;
            return f->values[0];
        }
    return d;
}

const Float *ParamSet::findFloat(const string &name, int *n) const {
    for (const auto &f : floats)
        if (f->name == name) {
            *n = f->nValues;
            f->lookedUp = true;
            return f->values.get();
        }
    return nullptr;
}

const int *ParamSet::findInt(const string &name, int *nValues) const {
    LOOKUP_PTR(ints);
}

const bool *ParamSet::findBool(const string &name, int *nValues) const {
    LOOKUP_PTR(bools);
}

int ParamSet::findOneInt(const string &name, int d) const {
    LOOKUP_ONE(ints);
}

bool ParamSet::findOneBool(const string &name, bool d) const {
    LOOKUP_ONE(bools);
}

const Point2f *ParamSet::findPoint2f(const string &name,
                                     int *nValues) const {
    LOOKUP_PTR(point2fs);
}

Point2f ParamSet::findOnePoint2f(const string &name,
                                 const Point2f &d) const {
    LOOKUP_ONE(point2fs);
}

const Vector2f *ParamSet::findVector2f(const string &name,
                                       int *nValues) const {
    LOOKUP_PTR(vector2fs);
}

Vector2f ParamSet::findOneVector2f(const string &name,
                                   const Vector2f &d) const {
    LOOKUP_ONE(vector2fs);
}

const Point3f *ParamSet::findPoint3f(const string &name,
                                     int *nValues) const {
    LOOKUP_PTR(point3fs);
}

Point3f ParamSet::findOnePoint3f(const string &name,
                                 const Point3f &d) const {
    LOOKUP_ONE(point3fs);
}

const Vector3f *ParamSet::findVector3f(const string &name,
                                       int *nValues) const {
    LOOKUP_PTR(vector3fs);
}

Vector3f ParamSet::findOneVector3f(const string &name,
                                   const Vector3f &d) const {
    LOOKUP_ONE(vector3fs);
}

const Normal3f *ParamSet::findNormal3f(const string &name,
                                       int *nValues) const {
    LOOKUP_PTR(normals);
}

Normal3f ParamSet::findOneNormal3f(const string &name,
                                   const Normal3f &d) const {
    LOOKUP_ONE(normals);
}

const Spectrum *ParamSet::findSpectrum(const string &name,
                                       int *nValues) const {
    LOOKUP_PTR(spectra);
}

Spectrum ParamSet::findOneSpectrum(const string &name,
                                   const Spectrum &d) const {
    LOOKUP_ONE(spectra);
}

const string *ParamSet::findString(const string &name,
                                        int *nValues) const {
    LOOKUP_PTR(strings);
}

string ParamSet::findOneString(const string &name,
                                    const string &d) const {
    LOOKUP_ONE(strings);
}

string ParamSet::findOneFilename(const string &name,
                                      const string &d) const {
    string filename = findOneString(name, "");
    if (filename == "") return d;
    filename = AbsolutePath(ResolveFilename(filename));
    return filename;
}

string ParamSet::findTexture(const string &name) const {
    string d = "";
    LOOKUP_ONE(textures);
}

#define CHECK_UNUSED(v)                                                 \
    for (size_t i = 0; i < (v).size(); ++i)                             \
        if (!(v)[i]->lookedUp)                                          \
            Report::Report::warning("Parameter \"%s\" not used", (v)[i]->name.c_str())

void ParamSet::reportUnused() const {
    CHECK_UNUSED(ints);
    CHECK_UNUSED(bools);
    CHECK_UNUSED(floats);
    CHECK_UNUSED(point2fs);
    CHECK_UNUSED(vector2fs);
    CHECK_UNUSED(point3fs);
    CHECK_UNUSED(vector3fs);
    CHECK_UNUSED(normals);
    CHECK_UNUSED(spectra);
    CHECK_UNUSED(strings);
    CHECK_UNUSED(textures);
}

void ParamSet::clear() {
#define DEL_PARAMS(name) (name).erase((name).begin(), (name).end())
    DEL_PARAMS(ints);
    DEL_PARAMS(bools);
    DEL_PARAMS(floats);
    DEL_PARAMS(point2fs);
    DEL_PARAMS(vector2fs);
    DEL_PARAMS(point3fs);
    DEL_PARAMS(vector3fs);
    DEL_PARAMS(normals);
    DEL_PARAMS(spectra);
    DEL_PARAMS(strings);
    DEL_PARAMS(textures);
#undef DEL_PARAMS
}

string ParamSet::toString() const {
    string ret;
    size_t i;
    int j;
    string typeString;
    for (i = 0; i < ints.size(); ++i) {
        const shared_ptr<ParamSetItem<int>> &item = ints[i];
        typeString = "integer ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%d ", item->values[j]);
        ret += string("] ");
    }
    for (i = 0; i < bools.size(); ++i) {
        const shared_ptr<ParamSetItem<bool>> &item = bools[i];
        typeString = "bool ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("\"%s\" ", item->values[j] ? "true" : "false");
        ret += string("] ");
    }
    for (i = 0; i < floats.size(); ++i) {
        const shared_ptr<ParamSetItem<Float>> &item = floats[i];
        typeString = "float ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%.8g ", item->values[j]);
        ret += string("] ");
    }
    for (i = 0; i < point2fs.size(); ++i) {
        const shared_ptr<ParamSetItem<Point2f>> &item = point2fs[i];
        typeString = "point2 ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%.8g %.8g ", item->values[j].x,
                                item->values[j].y);
        ret += string("] ");
    }
    for (i = 0; i < vector2fs.size(); ++i) {
        const shared_ptr<ParamSetItem<Vector2f>> &item = vector2fs[i];
        typeString = "vector2 ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%.8g %.8g ", item->values[j].x,
                                item->values[j].y);
        ret += string("] ");
    }
    for (i = 0; i < point3fs.size(); ++i) {
        const shared_ptr<ParamSetItem<Point3f>> &item = point3fs[i];
        typeString = "point3 ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%.8g %.8g %.8g ", item->values[j].x,
                                item->values[j].y, item->values[j].z);
        ret += string("] ");
    }
    for (i = 0; i < vector3fs.size(); ++i) {
        const shared_ptr<ParamSetItem<Vector3f>> &item = vector3fs[i];
        typeString = "vector3 ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%.8g %.8g %.8g ", item->values[j].x,
                                item->values[j].y, item->values[j].z);
        ret += string("] ");
    }
    for (i = 0; i < normals.size(); ++i) {
        const shared_ptr<ParamSetItem<Normal3f>> &item = normals[i];
        typeString = "normal ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("%.8g %.8g %.8g ", item->values[j].x,
                                item->values[j].y, item->values[j].z);
        ret += string("] ");
    }
    for (i = 0; i < strings.size(); ++i) {
        const shared_ptr<ParamSetItem<string>> &item = strings[i];
        typeString = "string ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("\"%s\" ", item->values[j].c_str());
        ret += string("] ");
    }
    for (i = 0; i < textures.size(); ++i) {
        const shared_ptr<ParamSetItem<string>> &item = textures[i];
        typeString = "texture ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j)
            ret += StringPrint::printf("\"%s\" ", item->values[j].c_str());
        ret += string("] ");
    }
    for (i = 0; i < spectra.size(); ++i) {
        const shared_ptr<ParamSetItem<Spectrum>> &item = spectra[i];
        typeString = "color ";
        // Print _ParamSetItem_ declaration, determine how many to print
        int nPrint = item->nValues;
        ret += string("\"");
        ret += typeString;
        ret += item->name;
        ret += string("\"");
        ret += string(" [");
        for (j = 0; j < nPrint; ++j) {
            Float rgb[3];
            item->values[j].ToRGB(rgb);
            ret += StringPrint::printf("%.8g %.8g %.8g ", rgb[0], rgb[1], rgb[2]);
        }
        ret += string("] ");
    }
    return ret;
}

void ParamSet::printSet(int indent) const {
    printItems("integer", indent, ints);
    printItems("bool", indent, bools);
    printItems("float", indent, floats);
    printItems("point2", indent, point2fs);
    printItems("vector2", indent, vector2fs);
    printItems("point", indent, point3fs);
    printItems("vector", indent, vector3fs);
    printItems("normal", indent, normals);
    printItems("string", indent, strings);
    printItems("texture", indent, textures);
    printItems("rgb", indent, spectra);
}

// TextureParams Method Definitions
shared_ptr<Texture<Spectrum>> TextureParams::getSpectrumTexture(
    const string &n, const Spectrum &def) const {
    shared_ptr<Texture<Spectrum>> tex = getSpectrumTextureOrNull(n);
    if (tex)
        return tex;
    else
        return std::make_shared<ConstantTexture<Spectrum>>(def);
}

shared_ptr<Texture<Spectrum>> TextureParams::getSpectrumTextureOrNull(
    const string &n) const {
    // Check the shape parameters first.
    string name = geomParams.findTexture(n);
    if (name.empty()) {
        int count;
        const Spectrum *s = geomParams.findSpectrum(n, &count);
        if (s) {
            if (count > 1)
                Report::warning("Ignoring excess values provided with parameter \"%s\"",
                        n.c_str());
            return std::make_shared<ConstantTexture<Spectrum>>(*s);
        }

        name = materialParams.findTexture(n);
        if (name.empty()) {
            int count;
            const Spectrum *s = materialParams.findSpectrum(n, &count);
            if (s) {
                if (count > 1)
                    Report::warning("Ignoring excess values provided with parameter \"%s\"",
                            n.c_str());
                return std::make_shared<ConstantTexture<Spectrum>>(*s);
            }
        }

        if (name.empty())
            return nullptr;
    }

    // We have a texture name, from either the shape or the material's
    // parameters.
    if (spectrumTextures.find(name) != spectrumTextures.end())
        return spectrumTextures[name];
    else {
        Report::error("Couldn't find spectrum texture named \"%s\" for parameter \"%s\"",
              name.c_str(), n.c_str());
        return nullptr;
    }
}

shared_ptr<Texture<Float>> TextureParams::getFloatTexture(
    const string &n, Float def) const {
    shared_ptr<Texture<Float>> tex = getFloatTextureOrNull(n);
    if (tex)
        return tex;
    else
        return std::make_shared<ConstantTexture<Float>>(def);
}

shared_ptr<Texture<Float>> TextureParams::getFloatTextureOrNull(
    const string &n) const {
    // Check the shape parameters first.
    string name = geomParams.findTexture(n);
    if (name.empty()) {
        int count;
        const Float *s = geomParams.findFloat(n, &count);
        if (s) {
            if (count > 1)
                Report::warning("Ignoring excess values provided with parameter \"%s\"",
                        n.c_str());
            return std::make_shared<ConstantTexture<Float>>(*s);
        }

        name = materialParams.findTexture(n);
        if (name.empty()) {
            int count;
            const Float *s = materialParams.findFloat(n, &count);
            if (s) {
                if (count > 1)
                    Report::warning("Ignoring excess values provided with parameter \"%s\"",
                            n.c_str());
                return std::make_shared<ConstantTexture<Float>>(*s);
            }
        }

        if (name.empty())
            return nullptr;
    }

    // We have a texture name, from either the shape or the material's
    // parameters.
    if (floatTextures.find(name) != floatTextures.end())
        return floatTextures[name];
    else {
        Report::error("Couldn't find float texture named \"%s\" for parameter \"%s\"",
                      name.c_str(), n.c_str());
        return nullptr;
    }
}

template <typename T> static void
reportUnusedMaterialParams(
    const std::vector<shared_ptr<ParamSetItem<T>>> &mtl,
    const std::vector<shared_ptr<ParamSetItem<T>>> &geom) {
    for (const auto &param : mtl) {
        if (param->lookedUp)
            continue;

        // Don't complain about any unused material parameters if their
        // values were provided by a shape parameter.
        if (std::find_if(geom.begin(), geom.end(),
                         [&param](const shared_ptr<ParamSetItem<T>> &gp) {
                             return gp->name == param->name;
                         }) == geom.end())
            Report::warning("Parameter \"%s\" not used", param->name.c_str());
    }
}

void TextureParams::ReportUnused() const {
    geomParams.reportUnused();
    reportUnusedMaterialParams(materialParams.ints, geomParams.ints);
    reportUnusedMaterialParams(materialParams.bools, geomParams.bools);
    reportUnusedMaterialParams(materialParams.floats, geomParams.floats);
    reportUnusedMaterialParams(materialParams.point2fs, geomParams.point2fs);
    reportUnusedMaterialParams(materialParams.vector2fs, geomParams.vector2fs);
    reportUnusedMaterialParams(materialParams.point3fs, geomParams.point3fs);
    reportUnusedMaterialParams(materialParams.vector3fs, geomParams.vector3fs);
    reportUnusedMaterialParams(materialParams.normals, geomParams.normals);
    reportUnusedMaterialParams(materialParams.spectra, geomParams.spectra);
    reportUnusedMaterialParams(materialParams.strings, geomParams.strings);
    reportUnusedMaterialParams(materialParams.textures, geomParams.textures);
}
