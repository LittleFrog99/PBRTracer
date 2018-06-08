#ifndef PARAMSET_H
#define PARAMSET_H


#include "vector.h"
#include "core/texture.h"
#include "core/spectrum.h"
#include <map>


template <typename T>
struct ParamSetItem {
    ParamSetItem(const std::string &name, std::unique_ptr<T[]> val, int nValues = 1)
        : name(name), values(std::move(val)), nValues(nValues) {}

    const string name;
    const unique_ptr<T[]> values;
    const int nValues;
    mutable bool lookedUp = false;
};

class ParamSet {
public:
    ParamSet() {}

    void addFloat(const string &, unique_ptr<Float[]> v, int nValues = 1);
    void addInt(const string &, unique_ptr<int[]> v, int nValues);
    void addBool(const string &, unique_ptr<bool[]> v, int nValues);
    void addPoint2f(const string &, unique_ptr<Point2f[]> v, int nValues);
    void addVector2f(const string &, unique_ptr<Vector2f[]> v, int nValues);
    void addPoint3f(const string &, unique_ptr<Point3f[]> v, int nValues);
    void addVector3f(const string &, unique_ptr<Vector3f[]> v, int nValues);
    void addNormal3f(const string &, unique_ptr<Normal3f[]> v, int nValues);
    void addString(const string &, unique_ptr<string[]> v, int nValues);
    void addTexture(const string &, const string &);
    void addRGBSpectrum(const string &, unique_ptr<Float[]> v, int nValues);
    void addXYZSpectrum(const string &, unique_ptr<Float[]> v, int nValues);
    void addBlackbodySpectrum(const string &, unique_ptr<Float[]> v, int nValues);
    void addSampledSpectrumFiles(const string &, const char **, int nValues);
    void addSampledSpectrum(const string &, unique_ptr<Float[]> v, int nValues);

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

    Float findOneFloat(const string &, Float d) const;
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
    const Float *findFloat(const string &, int *n) const;
    const int *findInt(const string &, int *nValues) const;
    const bool *findBool(const string &, int *nValues) const;
    const Point2f *findPoint2f(const string &, int *nValues) const;
    const Vector2f *findVector2f(const string &, int *nValues) const;
    const Point3f *findPoint3f(const string &, int *nValues) const;
    const Vector3f *findVector3f(const string &, int *nValues) const;
    const Normal3f *findNormal3f(const string &, int *nValues) const;
    const Spectrum *findSpectrum(const string &, int *nValues) const;
    const string *findString(const string &, int *nValues) const;

    void reportUnused() const;
    void clear();
    string toString() const;
    void printSet(int indent) const;

private:
    friend class TextureParams;
    friend bool shapeMaySetMaterialParameters(const ParamSet &ps);

    template <typename T>
    static void printItems(
        const char *type, int indent,
        const std::vector<std::shared_ptr<ParamSetItem<T>>> &items) {
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

    inline static int print(int i) { return printf("%d ", i); }
    inline static int print(bool v) { return v ? printf("\"true\" ") : printf("\"false\" ");}

    inline static int print(Float f) {
        if (int(f) == f)
            return printf("%d ", (int)f);
        else
            return printf("%.9g ", f);
    }

    inline static int print(const Point2f &p) {
        int np = print(p.x);
        return np + print(p.y);
    }

    inline static int print(const Vector2f &v) {
        int np = print(v.x);
        return np + print(v.y);
    }

    inline static int print(const Point3f &p) {
        int np = print(p.x);
        np += print(p.y);
        return np + print(p.z);
    }

    inline static int print(const Vector3f &v) {
        int np = print(v.x);
        np += print(v.y);
        return np + print(v.z);
    }

    inline static int print(const Normal3f &n) {
        int np = print(n.x);
        np += print(n.y);
        return np + print(n.z);
    }

    inline static int print(const string &s) { return printf("\"%s\" ", s.c_str()); }

    inline static int print(const Spectrum &s) {
        Float rgb[3];
        s.toRGB(rgb);
        int np = print(rgb[0]);
        np += print(rgb[1]);
        return np + print(rgb[2]);
    }

    vector<shared_ptr<ParamSetItem<bool>>> bools;
    vector<shared_ptr<ParamSetItem<int>>> ints;
    vector<shared_ptr<ParamSetItem<Float>>> floats;
    vector<shared_ptr<ParamSetItem<Point2f>>> point2fs;
    vector<shared_ptr<ParamSetItem<Vector2f>>> vector2fs;
    vector<shared_ptr<ParamSetItem<Point3f>>> point3fs;
    vector<shared_ptr<ParamSetItem<Vector3f>>> vector3fs;
    vector<shared_ptr<ParamSetItem<Normal3f>>> normals;
    vector<shared_ptr<ParamSetItem<RGBSpectrum>>> spectra;
    vector<shared_ptr<ParamSetItem<string>>> strings;
    vector<shared_ptr<ParamSetItem<string>>> textures;
    static map<string, Spectrum> cachedSpectra;
};


class TextureParams {
public:
    // TextureParams Public Methods
    TextureParams(const ParamSet &geomParams, const ParamSet &materialParams,
                  map<string, shared_ptr<Texture<Float>>> &fTex,
                  map<string, shared_ptr<Texture<Spectrum>>> &sTex)
        : floatTextures(fTex), spectrumTextures(sTex), geomParams(geomParams),
          materialParams(materialParams) {}

    shared_ptr<Texture<Spectrum>> getSpectrumTexture(const string &name, const Spectrum &def) const;
    shared_ptr<Texture<Spectrum>> getSpectrumTextureOrNull(const string &name) const;
    shared_ptr<Texture<Float>> getFloatTexture(const string &name, Float def) const;
    shared_ptr<Texture<Float>> getFloatTextureOrNull(const string &name) const;

    inline Float findFloat(const string &n, Float d) const {
        return geomParams.findOneFloat(n, materialParams.findOneFloat(n, d));
    }

    inline string findString(const string &n, const string &d = "") const {
        return geomParams.findOneString(n, materialParams.findOneString(n, d));
    }

    inline string findFilename(const string &n, const string &d = "") const {
        return geomParams.findOneFilename(n, materialParams.findOneFilename(n, d));
    }

    inline int findInt(const string &n, int d) const {
        return geomParams.findOneInt(n, materialParams.findOneInt(n, d));
    }

    inline bool findBool(const string &n, bool d) const {
        return geomParams.findOneBool(n, materialParams.findOneBool(n, d));
    }

    inline Point3f findPoint3f(const string &n, const Point3f &d) const {
        return geomParams.findOnePoint3f(n, materialParams.findOnePoint3f(n, d));
    }

    inline Vector3f findVector3f(const string &n, const Vector3f &d) const {
        return geomParams.findOneVector3f(n, materialParams.findOneVector3f(n, d));
    }

    inline Normal3f findNormal3f(const string &n, const Normal3f &d) const {
        return geomParams.findOneNormal3f(n, materialParams.findOneNormal3f(n, d));
    }

    inline Spectrum findSpectrum(const string &n, const Spectrum &d) const {
        return geomParams.findOneSpectrum(n, materialParams.findOneSpectrum(n, d));
    }

    void ReportUnused() const;

    inline const ParamSet & getGeomParams() const { return geomParams; }
    inline const ParamSet & getMaterialParams() const { return materialParams; }

private:
    map<string, shared_ptr<Texture<Float>>> &floatTextures;
    map<string, shared_ptr<Texture<Spectrum>>> &spectrumTextures;
    const ParamSet &geomParams, &materialParams;
};

#endif // PARAMSET_H
