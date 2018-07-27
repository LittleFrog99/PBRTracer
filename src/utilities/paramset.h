#ifndef UTILITY_PARAMSET
#define UTILITY_PARAMSET

#include "vector.h"
#include "log.h"
#include "core/spectrum.h"
#include "core/texture.h"

template <typename T>
struct ParamSetItem {
    ParamSetItem(const string &name, unique_ptr<T[]> val, int nValues = 1)
        : name(name), values(move(val)), nValues(nValues) {}

    const string name;
    const unique_ptr<T[]> values;
    const int nValues;
    mutable bool lookedUp = false;
};

class ParamSet {
public:
    ParamSet() {}

    void addFloat(const string &, unique_ptr<float[]> v, int nValues = 1);
    void addInt(const string &, unique_ptr<int[]> v, int nValues);
    void addBool(const string &, unique_ptr<bool[]> v, int nValues);
    void addPoint2f(const string &, unique_ptr<Point2f[]> v, int nValues);
    void addVector2f(const string &, unique_ptr<Vector2f[]> v, int nValues);
    void addPoint3f(const string &, unique_ptr<Point3f[]> v, int nValues);
    void addVector3f(const string &, unique_ptr<Vector3f[]> v, int nValues);
    void addNormal3f(const string &, unique_ptr<Normal3f[]> v, int nValues);
    void addString(const string &, unique_ptr<string[]> v, int nValues);
    void addTexture(const string &, const string &);
    void addRGBSpectrum(const string &, unique_ptr<float[]> v, int nValues);
    void addXYZSpectrum(const string &, unique_ptr<float[]> v, int nValues);
    void addBlackbodySpectrum(const string &, unique_ptr<float[]> v, int nValues);
    void addSampledSpectrumFiles(const string &, const char **, int nValues);
    void addSampledSpectrum(const string &, unique_ptr<float[]> v, int nValues);

    bool eraseInt(const string &);
    bool eraseBool(const string &);
    bool eraseFloat(const string &);
    bool erasePoint2f(const string &);
    bool eraseVector2f(const string &);
    bool erasePoint3f(const string &);
    bool eraseVector3f(const string &);
    bool eraseNormal3f(const string &);
    bool eraseSpectrum(const string &);
    bool eraseString(const string &);
    bool eraseTexture(const string &);

    float findOneFloat(const string &, float d) const;
    int findOneInt(const string &, int d) const;
    bool findOneBool(const string &, bool d) const;
    Point2f findOnePoint2f(const string &, const Point2f &d) const;
    Vector2f findOneVector2f(const string &, const Vector2f &d) const;
    Point3f findOnePoint3f(const string &, const Point3f &d) const;
    Vector3f findOneVector3f(const string &, const Vector3f &d) const;
    Normal3f findOneNormal3f(const string &, const Normal3f &d) const;
    Spectrum findOneSpectrum(const string &, const Spectrum &d) const;
    string findOneString(const string &, const string &d) const;
    string findOneFilename(const string &, const string &d) const;
    string findTexture(const string &) const;
    const float * findFloat(const string &, int *n) const;
    const int * findInt(const string &, int *nValues) const;
    const bool * findBool(const string &, int *nValues) const;
    const Point2f * findPoint2f(const string &, int *nValues) const;
    const Vector2f * findVector2f(const string &, int *nValues) const;
    const Point3f * findPoint3f(const string &, int *nValues) const;
    const Vector3f * findVector3f(const string &, int *nValues) const;
    const Normal3f * findNormal3f(const string &, int *nValues) const;
    const Spectrum * findSpectrum(const string &, int *nValues) const;
    const string * findString(const string &, int *nValues) const;

    bool shapeMaySetMaterialParameters() const;

    void reportUnused() const;
    void clear();
    string toString() const;

    static int print(int i) { return printf("%d ", i); }
    static int print(bool v) { return v ? printf("\"true\" ") : printf("\"false\" ");}

    static int print(float f) {
        if (int(f) == f)
            return printf("%d ", (int)f);
        else
            return printf("%.9g ", f);
    }

    static int print(const Point2f &p) {
        int np = print(p.x);
        return np + print(p.y);
    }

    static int print(const Vector2f &v) {
        int np = print(v.x);
        return np + print(v.y);
    }

    static int print(const Point3f &p) {
        int np = print(p.x);
        np += print(p.y);
        return np + print(p.z);
    }

    static int print(const Vector3f &v) {
        int np = print(v.x);
        np += print(v.y);
        return np + print(v.z);
    }

    static int print(const Normal3f &n) {
        int np = print(n.x);
        np += print(n.y);
        return np + print(n.z);
    }

    static int print(const string &s) { return printf("\"%s\" ", s.c_str()); }

    static int print(const Spectrum &s) {
        float rgb[3];
        s.toRGB(rgb);
        int np = print(rgb[0]);
        np += print(rgb[1]);
        return np + print(rgb[2]);
    }

    void printSet(int indent) const;

private:
    vector<shared_ptr<ParamSetItem<bool>>> bools;
    vector<shared_ptr<ParamSetItem<int>>> ints;
    vector<shared_ptr<ParamSetItem<float>>> floats;
    vector<shared_ptr<ParamSetItem<Point2f>>> point2fs;
    vector<shared_ptr<ParamSetItem<Vector2f>>> vector2fs;
    vector<shared_ptr<ParamSetItem<Point3f>>> point3fs;
    vector<shared_ptr<ParamSetItem<Vector3f>>> vector3fs;
    vector<shared_ptr<ParamSetItem<Normal3f>>> normals;
    vector<shared_ptr<ParamSetItem<Spectrum>>> spectra;
    vector<shared_ptr<ParamSetItem<string>>> strings;
    vector<shared_ptr<ParamSetItem<string>>> textures;
    static map<string, Spectrum> cachedSpectra;

    friend class TextureParams;

    template <typename T>
    static void printItems(
        const char *type, int indent,
        const vector<shared_ptr<ParamSetItem<T>>> &items) {
        for (const auto &item : items) {
            int np = printf("\n%*s\"%s %s\" [ ", indent + 8, "", type,
                            item->name.c_str());
            for (int i = 0; i < item->nValues; ++i) {
                np += print(item->values[i]);
                if (np > 80 && i < item->nValues - 1)
                    np = printf("\n%*s", indent + 8, "");
            }
            printf("] ");
        }
    }
};


class TextureParams {
public:
    TextureParams();
    TextureParams(const ParamSet &geomParams, const ParamSet &materialParams,
                  map<string, shared_ptr<Texture<float>>> &fTex,
                  map<string, shared_ptr<Texture<Spectrum>>> &sTex)
        : floatTextures(fTex), spectrumTextures(sTex), geomParams(geomParams),
          materialParams(materialParams) {}

    shared_ptr<Texture<Spectrum>> getSpectrumTexture(const string &name, const Spectrum &def) const;
    shared_ptr<Texture<Spectrum>> getSpectrumTextureOrNull(const string &name) const;
    shared_ptr<Texture<float>> getFloatTexture(const string &name, float def) const;
    shared_ptr<Texture<float>> getFloatTextureOrNull(const string &name) const;

    float findFloat(const string &n, float d) const {
        return geomParams.findOneFloat(n, materialParams.findOneFloat(n, d));
    }

    string findString(const string &n, const string &d = "") const {
        return geomParams.findOneString(n, materialParams.findOneString(n, d));
    }

    string findFilename(const string &n, const string &d = "") const {
        return geomParams.findOneFilename(n, materialParams.findOneFilename(n, d));
    }

    int findInt(const string &n, int d) const {
        return geomParams.findOneInt(n, materialParams.findOneInt(n, d));
    }

    bool findBool(const string &n, bool d) const {
        return geomParams.findOneBool(n, materialParams.findOneBool(n, d));
    }

    Point3f findPoint3f(const string &n, const Point3f &d) const {
        return geomParams.findOnePoint3f(n, materialParams.findOnePoint3f(n, d));
    }

    Vector3f findVector3f(const string &n, const Vector3f &d) const {
        return geomParams.findOneVector3f(n, materialParams.findOneVector3f(n, d));
    }

    Normal3f findNormal3f(const string &n, const Normal3f &d) const {
        return geomParams.findOneNormal3f(n, materialParams.findOneNormal3f(n, d));
    }

    Spectrum findSpectrum(const string &n, const Spectrum &d) const {
        return geomParams.findOneSpectrum(n, materialParams.findOneSpectrum(n, d));
    }

    void reportUnused() const;

    const ParamSet & getGeomParams() const { return geomParams; }
    const ParamSet & getMaterialParams() const { return materialParams; }

private:
    map<string, shared_ptr<Texture<float>>> &floatTextures;
    map<string, shared_ptr<Texture<Spectrum>>> &spectrumTextures;
    const ParamSet &geomParams, &materialParams;

    template <typename T>
    static void reportUnusedMaterialParams(
        const vector<shared_ptr<ParamSetItem<T>>> &mtl,
        const vector<shared_ptr<ParamSetItem<T>>> &geom) {
        for (const auto &param : mtl) {
            if (param->lookedUp) continue;

            // Don't complain about any unused material parameters if their
            // values were provided by a shape parameter.
            if (find_if(geom.begin(), geom.end(),
                             [&param](const shared_ptr<ParamSetItem<T>> &gp) {
                                 return gp->name == param->name;
                             }) == geom.end())
                WARNING("Parameter \"%s\" not used", param->name.c_str());
        }
    }
};

#endif // UTILITY_PARAMSET
