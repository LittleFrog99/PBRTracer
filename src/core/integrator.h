#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "interaction.h"
#include "light.h"
#include "camera.h"
#include "sampler.h"

class Integrator {
public:
    virtual ~Integrator();
    virtual void render(const Scene &scene) = 0;
};

class SamplerIntegrator : public Integrator {
public:
    SamplerIntegrator(shared_ptr<const Camera> camera, shared_ptr<Sampler> sampler,
                      const Bounds2i &pixelBounds)
        : camera(camera), sampler(sampler), pixelBounds(pixelBounds) {}
    virtual void preprocess(const Scene &scene, Sampler &sampler) {}
    void render(const Scene &scene);
    virtual RGBSpectrum compute_Li(const RayDifferential &ray, const Scene &scene,
                                Sampler &sampler, MemoryArena &arena, int depth = 0) const = 0;

    RGBSpectrum specularReflect(const RayDifferential &ray, const SurfaceInteraction &isect,
                             const Scene &scene, Sampler &sampler, MemoryArena &arena,
                             int depth) const;
    RGBSpectrum specularTransmit(const RayDifferential &ray, const SurfaceInteraction &isect,
                              const Scene &scene, Sampler &sampler, MemoryArena &arena,
                              int depth) const;

    static RGBSpectrum uniformSampleAllLights(const Interaction &it, const Scene &scene,
                                           MemoryArena &arena, Sampler &sampler,
                                           const vector<int> &nLightSamples,
                                           bool handleMedia = false);
    static RGBSpectrum uniformSampleOneLight(const Interaction &it, const Scene &scene,
                                          MemoryArena &arena, Sampler &sampler,
                                          bool handleMedia = false,
                                          const Distribution1D *lightDistrib = nullptr);
    static RGBSpectrum estimateDirect(const Interaction &it, const Point2f &uShading,
                                   const Light &light, const Point2f &uLight,
                                   const Scene &scene, Sampler &sampler,
                                   MemoryArena &arena, bool handleMedia = false,
                                   bool specular = false);
    static unique_ptr<Distribution1D> computeLightPowerDistrib(const Scene &scene);

protected:
    shared_ptr<const Camera> camera;

private:
    shared_ptr<Sampler> sampler;
    const Bounds2i pixelBounds;
};

#endif // INTEGRATOR_H
