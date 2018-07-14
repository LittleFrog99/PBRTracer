#ifndef CORE_INTEGRATOR
#define CORE_INTEGRATOR

#include "interaction.h"
#include "light.h"
#include "camera.h"
#include "sampling.h"

class Integrator {
public:
    virtual ~Integrator() { }
    virtual void render(const Scene &scene) = 0;
};

class SamplerIntegrator : public Integrator {
public:
    SamplerIntegrator(shared_ptr<const Camera> camera, shared_ptr<Sampler> sampler,
                      const Bounds2i &pixelBounds)
        : camera(camera), sampler(sampler), pixelBounds(pixelBounds) {}
    virtual void preprocess(const Scene &scene, Sampler &sampler) {}
    void render(const Scene &scene);
    virtual Spectrum compute_Li(const RayDifferential &ray, const Scene &scene,
                                   Sampler &sampler, MemoryArena &arena, int depth = 0) const = 0;

    Spectrum specularReflect(const RayDifferential &ray, const SurfaceInteraction &isect,
                                       const Scene &scene, Sampler &sampler, MemoryArena &arena,
                                       int depth) const;
    Spectrum specularTransmit(const RayDifferential &ray, const SurfaceInteraction &isect,
                                        const Scene &scene, Sampler &sampler, MemoryArena &arena,
                                        int depth) const;

    static Spectrum uniformSampleAllLights(const Interaction &it, const Scene &scene,
                                              MemoryArena &arena, Sampler &sampler,
                                              const vector<int> &nLightSamples,
                                              bool handleMedia = false);
    static Spectrum uniformSampleOneLight(const Interaction &it, const Scene &scene,
                                             MemoryArena &arena, Sampler &sampler,
                                             bool handleMedia = false,
                                             const Distribution1D *lightDistrib = nullptr);
    static Spectrum estimateDirect(const Interaction &it, const Point2f &uShading,
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

#endif // CORE_INTEGRATOR
