#ifndef CORE_BSDF
#define CORE_BSDF

#include "interaction.h"

enum BxDFType {
    BSDF_REFLECTION = 1 << 0,
    BSDF_TRANSMISSION = 1 << 1,
    BSDF_DIFFUSE = 1 << 2,
    BSDF_GLOSSY = 1 << 3,
    BSDF_SPECULAR = 1 << 4,
    BSDF_ALL = BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR | BSDF_REFLECTION | BSDF_TRANSMISSION,
};

class BxDF {
public:
    BxDF(BxDFType type) : type(type) {}
    virtual ~BxDF() {}

    virtual Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const = 0;
    virtual Spectrum sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &sample, Float *pdf,
                                 BxDFType *sampledType = nullptr) const;
    virtual Spectrum rho_hd(const Vector3f &wo, int nSamples, const Point2f *samples) const;
    virtual Spectrum rho_hh(int nSamples, const Point2f *samples1, const Point2f *samples2) const;
    virtual Float pdf(const Vector3f &wo, const Vector3f &wi) const;
    virtual string toString() const = 0;

    bool matchesFlags(BxDFType t) const { return (type & t) == type; }

    const BxDFType type;
};

inline ostream & operator << (ostream &os, const BxDF &bxdf) {
    os << bxdf.toString();
    return os;
}

class ScaledBxDF : public BxDF {
public:
    ScaledBxDF(BxDF *bxdf, const Spectrum &scale)
        : BxDF(bxdf->type), bxdf(bxdf), scale(scale) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const {
        return scale * bxdf->compute_f(wo, wi);
    }

    string toString() const {
        return string("[ ScaledBxDF bxdf: ") + bxdf->toString() + string(" scale: ")
                + scale.toString() + string(" ]");
    }

private:
    BxDF *bxdf;
    Spectrum scale;
};

class BSDF {
public:
    BSDF(const SurfaceInteraction &si, Float eta = 1)
        : eta(eta), ns(si.shading.n), ng(si.n), ss(normalize(si.shading.dpdu)), ts(cross(ns, ss)) {}

    int numComponents(BxDFType flags = BSDF_ALL) const;
    Spectrum compute_f(const Vector3f &woW, const Vector3f &wiW, BxDFType flags = BSDF_ALL) const;
    Spectrum compute_rho(int nSamples, const Point2f *samples1, const Point2f *samples2,
                            BxDFType flags = BSDF_ALL) const;
    Spectrum compute_rho(const Vector3f &wo, int nSamples, const Point2f *samples,
                            BxDFType flags = BSDF_ALL) const;
    Spectrum sample_f(const Vector3f &wo, Vector3f *wi, const Point2f &u, Float *pdf,
                         BxDFType type = BSDF_ALL, BxDFType *sampledType = nullptr) const;
    Float pdf(const Vector3f &wo, const Vector3f &wi, BxDFType flags = BSDF_ALL) const;
    string toString() const;

    void add(BxDF *b) {
        CHECK_LT(nBxDFs, MAX_BXDFS);
        bxdfs[nBxDFs++] = b;
    }

    Vector3f worldToLocal(const Vector3f &v) const {
        return Vector3f(dot(v, ss), dot(v, ts), dot(v, ns));
    }

    Vector3f localToWorld(const Vector3f &v) const {
        return Vector3f(ss.x * v.x + ts.x * v.y + ns.x * v.z,
                        ss.y * v.x + ts.y * v.y + ns.y * v.z,
                        ss.z * v.x + ts.z * v.y + ns.z * v.z);
    }

    static constexpr int MAX_BXDFS = 8;
    const Float eta;

private:
    const Normal3f ns, ng;
    const Vector3f ss, ts;
    int nBxDFs = 0;
    BxDF *bxdfs[MAX_BXDFS];

    friend class MixMaterial;
};

inline ostream & operator << (ostream &os, const BSDF &bsdf) {
    os << bsdf.toString();
    return os;
}


#endif // CORE_BSDF
