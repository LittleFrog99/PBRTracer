#ifndef CORE_BSSRDF
#define CORE_BSSRDF

#include "core/scene.h"
#include "bsdfs/fresnel.h"

class BSSRDF {
public:
    BSSRDF(const SurfaceInteraction &po, float eta) : po(po), eta(eta) {}
    virtual ~BSSRDF() {}

    virtual Spectrum compute_S(const SurfaceInteraction &pi, const Vector3f &wi) = 0;
    virtual Spectrum sample_S(const Scene &scene, float u1, const Point2f &u2, MemoryArena &arena,
                              SurfaceInteraction *pi, float *pdf) const = 0;

    static bool getScatteringProperties(const string &name, Spectrum *sigma_a, Spectrum *sigma_prime_s);

protected:
    const SurfaceInteraction &po;
    float eta;

private:
    struct MeasuredSS;
    static MeasuredSS SubsurfaceParameterTable[];
};

class SeparableBSSRDF : public BSSRDF {
public:
    SeparableBSSRDF(const SurfaceInteraction &po, float eta, const Material *material, TransportMode mode)
        : BSSRDF(po, eta), ns(po.shading.n), ss(normalize(po.shading.dpdu)), ts(cross(ns, ss)),
          material(material), mode(mode) {}

    virtual Spectrum compute_Sr(float d) const = 0;

    Spectrum compute_S(const SurfaceInteraction &pi, const Vector3f &wi) {
        float Ft = 1 - Fresnel::dielectric_Fr(dot(po.wo, po.shading.n), 1, eta);
        return Ft * compute_Sp(pi) * compute_Sw(wi);
    }

    Spectrum sample_S(const Scene &scene, float u1, const Point2f &u2, MemoryArena &arena,
                      SurfaceInteraction *pi, float *pdf) const;

    Spectrum compute_Sw(const Vector3f &wi) const {
        float c = 1 - 2 * Fresnel::moment1(1.0f / eta);
        return (1 - Fresnel::dielectric_Fr(cosTheta(wi), 1, eta)) / (c * PI);
    }

    Spectrum compute_Sp(const SurfaceInteraction &pi) const {
        return compute_Sr(distance(pi.p, po.p));
    }

    Spectrum sample_Sp(const Scene &scene, float u1, const Point2f &u2, MemoryArena &arena,
                       SurfaceInteraction *pi, float *pdf) const;

private:
    const Normal3f ns;
    const Vector3f ss, ts;
    const Material *material;
    const TransportMode mode;

    friend class SeparableBSSRDFAdpater;
};

class SeparableBSSRDFAdpater : public BxDF {
public:
    SeparableBSSRDFAdpater(const SeparableBSSRDF *bssrdf)
        : BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), bssrdf(bssrdf) {}

    Spectrum compute_f(const Vector3f &wo, const Vector3f &wi) const {
        Spectrum f = bssrdf->compute_Sw(wi);
        if (bssrdf->mode == TransportMode::Radiance)
            f *= SQ(bssrdf->eta); // account for light transport mode non-symmetry
        return f;
    }
    // use default cosine sampling

    string toString() const { return "[ SeparableBSSRDFAdapter ]"; }

private:
    const SeparableBSSRDF *bssrdf;
};

#endif // CORE_BSSRDF
