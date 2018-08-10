#include "subsurface.h"
#include "bsdfs/microfacet.h"
#include "core/texture.h"
#include "paramset.h"
#include "interpolation.h"

SubsurfaceMaterial::SubsurfaceMaterial(float scale, const shared_ptr<Texture<Spectrum>> &Kr,
                                       const shared_ptr<Texture<Spectrum>> &Kt,
                                       const shared_ptr<Texture<Spectrum>> &sigma_a,
                                       const shared_ptr<Texture<Spectrum>> &sigma_s,
                                       float g, float eta,
                                       const shared_ptr<Texture<float>> &uRoughness,
                                       const shared_ptr<Texture<float>> &vRoughness,
                                       const shared_ptr<Texture<float>> &bumpMap,
                                       bool remapRoughness)
    : scale(scale), Kr(Kr), Kt(Kt), sigma_a(sigma_a), sigma_s(sigma_s),
      uRoughness(uRoughness), vRoughness(vRoughness), bumpMap(bumpMap), eta(eta), remapRoughness(remapRoughness),
      table(100, 64)
{
    BeamDiffusionSSS::fillTable(g, eta, &table);
}

void SubsurfaceMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                                    TransportMode mode, bool allowMultipleLobes) const
{
    // Perform bump mapping with _bumpMap_, if present
    if (bumpMap) bump(bumpMap, si);

    // Initialize BSDF for _SubsurfaceMaterial_
    Spectrum R = Kr->evaluate(*si).clamp();
    Spectrum T = Kt->evaluate(*si).clamp();
    float urough = uRoughness->evaluate(*si);
    float vrough = vRoughness->evaluate(*si);

    // Initialize _bsdf_ for smooth or rough dielectric
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si, eta);

    if (R.isBlack() && T.isBlack()) return;

    bool isSpecular = urough == 0 && vrough == 0;
    if (isSpecular && allowMultipleLobes) {
        si->bsdf->add(ARENA_ALLOC(arena, FresnelSpecular)(R, T, 1.f, eta, mode));
    } else {
        if (remapRoughness) {
            urough = TrowbridgeReitzDistribution::roughnessToAlpha(urough);
            vrough = TrowbridgeReitzDistribution::roughnessToAlpha(vrough);
        }
        MicrofacetDistribution *distrib =
            isSpecular ? nullptr : ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(urough, vrough);
        if (!R.isBlack()) {
            Fresnel *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.f, eta);
            if (isSpecular)
                si->bsdf->add(ARENA_ALLOC(arena, SpecularReflection)(R, fresnel));
            else
                si->bsdf->add(ARENA_ALLOC(arena, MicrofacetReflection)(R, distrib, fresnel));
        }
        if (!T.isBlack()) {
            if (isSpecular)
                si->bsdf->add(ARENA_ALLOC(arena, SpecularTransmission)(T, 1.f, eta, mode));
            else
                si->bsdf->add(ARENA_ALLOC(arena, MicrofacetTransmission)(T, distrib, 1.f, eta, mode));
        }
    }
    Spectrum sig_a = scale * sigma_a->evaluate(*si).clamp();
    Spectrum sig_s = scale * sigma_s->evaluate(*si).clamp();
    si->bssrdf = ARENA_ALLOC(arena, BeamDiffusionSSS)(*si, this, mode, eta, sig_a, sig_s, table);
}

SubsurfaceMaterial * SubsurfaceMaterial::create(const TextureParams &mp) {
    float sig_a_rgb[3] = {.0011f, .0024f, .014f}, sig_s_rgb[3] = {2.55f, 3.21f, 3.77f};
    Spectrum sig_a = Spectrum::fromRGB(sig_a_rgb), sig_s = Spectrum::fromRGB(sig_s_rgb);
    string name = mp.findString("name");
    bool found = BSSRDF::getScatteringProperties(name, &sig_a, &sig_s);
    float g = mp.findFloat("g", 0.0f);
    if (name != "") {
        if (!found)
            WARNING("Named material \"%s\" not found.  Using defaults.", name.c_str());
        else
            g = 0;
    }
    float scale = mp.findFloat("scale", 1.f);
    float eta = mp.findFloat("eta", 1.33f);

    shared_ptr<Texture<Spectrum>> sigma_a, sigma_s;
    sigma_a = mp.getTexture("sigma_a", sig_a);
    sigma_s = mp.getTexture("sigma_s", sig_s);
    shared_ptr<Texture<Spectrum>> Kr = mp.getTexture("Kr", Spectrum(1.f));
    shared_ptr<Texture<Spectrum>> Kt = mp.getTexture("Kt", Spectrum(1.f));
    shared_ptr<Texture<float>> roughu =  mp.getTexture("uroughness", 0.f);
    shared_ptr<Texture<float>> roughv = mp.getTexture("vroughness", 0.f);
    shared_ptr<Texture<float>> bumpMap = mp.getFloatTextureOrNull("bumpmap");
    bool remapRoughness = mp.findBool("remaproughness", true);
    return new SubsurfaceMaterial(scale, Kr, Kt, sigma_a, sigma_s, g, eta, roughu, roughv, bumpMap,
                                  remapRoughness);
}

KdSubsurfaceMaterial::KdSubsurfaceMaterial(float scale, const shared_ptr<Texture<Spectrum>> &Kd,
                                           const shared_ptr<Texture<Spectrum>> &Kr,
                                           const shared_ptr<Texture<Spectrum>> &Kt,
                                           const shared_ptr<Texture<Spectrum>> &mfp,
                                           float g, float eta,
                                           const shared_ptr<Texture<float> > &uRoughness,
                                           const shared_ptr<Texture<float> > &vRoughness,
                                           const shared_ptr<Texture<float> > &bumpMap, bool remapRoughness)
    : scale(scale), Kd(Kd), Kr(Kr), Kt(Kt), mfp(mfp),
      uRoughness(uRoughness), vRoughness(vRoughness), bumpMap(bumpMap), eta(eta), remapRoughness(remapRoughness),
      table(100, 64)
{
    BeamDiffusionSSS::fillTable(g, eta, &table);
}

void KdSubsurfaceMaterial::computeScatteringFunctions(SurfaceInteraction *si, MemoryArena &arena,
                                                      TransportMode mode, bool allowMultipleLobes) const
{
    // Perform bump mapping with _bumpMap_, if present
    if (bumpMap) bump(bumpMap, si);
    Spectrum R = Kr->evaluate(*si).clamp();
    Spectrum T = Kt->evaluate(*si).clamp();
    float urough = uRoughness->evaluate(*si);
    float vrough = vRoughness->evaluate(*si);

    // Initialize _bsdf_ for smooth or rough dielectric
    si->bsdf = ARENA_ALLOC(arena, BSDF)(*si, eta);

    if (R.isBlack() && T.isBlack()) return;

    bool isSpecular = urough == 0 && vrough == 0;
    if (isSpecular && allowMultipleLobes) {
        si->bsdf->add(ARENA_ALLOC(arena, FresnelSpecular)(R, T, 1.f, eta, mode));
    } else {
        if (remapRoughness) {
            urough = TrowbridgeReitzDistribution::roughnessToAlpha(urough);
            vrough = TrowbridgeReitzDistribution::roughnessToAlpha(vrough);
        }
        MicrofacetDistribution *distrib =
            isSpecular ? nullptr
                       : ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(urough, vrough);
        if (!R.isBlack()) {
            Fresnel *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.f, eta);
            if (isSpecular)
                si->bsdf->add(ARENA_ALLOC(arena, SpecularReflection)(R, fresnel));
            else
                si->bsdf->add(ARENA_ALLOC(arena, MicrofacetReflection)(R, distrib, fresnel));
        }
        if (!T.isBlack()) {
            if (isSpecular)
                si->bsdf->add(ARENA_ALLOC(arena, SpecularTransmission)(T, 1.f, eta, mode));
            else
                si->bsdf->add(ARENA_ALLOC(arena, MicrofacetTransmission)(T, distrib, 1.f, eta, mode));
        }
    }

    Spectrum mfree = scale * mfp->evaluate(*si).clamp();
    Spectrum kd = Kd->evaluate(*si).clamp();
    Spectrum sig_a, sig_s;
    fromDiffuse(kd, mfree, &sig_a, &sig_s);
    si->bssrdf = ARENA_ALLOC(arena, BeamDiffusionSSS)(*si, this, mode, eta, sig_a, sig_s, table);
}

void KdSubsurfaceMaterial::fromDiffuse(const Spectrum &rhoEff, const Spectrum &mfp, Spectrum *sigma_a,
                                       Spectrum *sigma_s) const
{
    using namespace Interpolation;
    for (int c = 0; c < Spectrum::nSamples; ++c) {
        float rho = invertCatmullRom(table.nRhoSamples, table.rhoSamples.get(), table.rhoEff.get(), rhoEff[c]);
        (*sigma_s)[c] = rho / mfp[c];
        (*sigma_a)[c] = (1 - rho) / mfp[c];
    }
}

KdSubsurfaceMaterial * KdSubsurfaceMaterial::create(const TextureParams &mp) {
    float Kd[3] = {.5, .5, .5};
    shared_ptr<Texture<Spectrum>> kd = mp.getTexture("Kd", Spectrum::fromRGB(Kd));
    shared_ptr<Texture<Spectrum>> mfp = mp.getTexture("mfp", Spectrum(1.f));
    shared_ptr<Texture<Spectrum>> kr = mp.getTexture("Kr", Spectrum(1.f));
    shared_ptr<Texture<Spectrum>> kt = mp.getTexture("Kt", Spectrum(1.f));
    shared_ptr<Texture<float>> roughu = mp.getTexture("uroughness", 0.f);
    shared_ptr<Texture<float>> roughv = mp.getTexture("vroughness", 0.f);
    shared_ptr<Texture<float>> bumpMap = mp.getFloatTextureOrNull("bumpmap");
    float eta = mp.findFloat("eta", 1.33f);
    float scale = mp.findFloat("scale", 1.0f);
    float g = mp.findFloat("g", 0.0f);
    bool remapRoughness = mp.findBool("remaproughness", true);
    return new KdSubsurfaceMaterial(scale, kd, kr, kt, mfp, g, eta, roughu, roughv, bumpMap, remapRoughness);
}
