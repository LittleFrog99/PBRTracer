#ifndef RENDERER_H
#define RENDERER_H

#include "scene.h"
#include "medium.h"
#include "paramset.h"
#include "transform.h"
#include "integrator.h"

enum class APIState { Uninitialized, OptionsBlock, WorldBlock };

class TransformSet;
class TransformCache;
struct RenderOptions;
struct MaterialInstance;
struct GraphicsState;

class Renderer {
public:
    static Options options;
    static int catIndentCount;

private:
    static APIState currentApiState;
    static TransformSet curTransform;
    static uint32_t activeTransformBits;
    static map<string, TransformSet> namedCoordinateSystems;
    static unique_ptr<RenderOptions> renderOptions;
    static GraphicsState graphicsState;
    static vector<GraphicsState> pushedGraphicsStates;
    static vector<TransformSet> pushedTransforms;
    static vector<uint32_t> pushedActiveTransformBits;
    static TransformCache transformCache;

};

class TransformSet {
public:
    Transform & operator [] (int i) {
        CHECK_GE(i, 0);
        CHECK_LT(i, MAX_TRANSFORMS);
        return t[i];
    }

    const Transform & operator [] (int i) const {
        CHECK_GE(i, 0);
        CHECK_LT(i, MAX_TRANSFORMS);
        return t[i];
    }

    TransformSet inverse() {
        TransformSet tInv;
        for (int i = 0; i < MAX_TRANSFORMS; ++i) tInv.t[i] = t[i].inverse();
        return tInv;
    }

    bool isAnimated() const {
        for (int i = 0; i < MAX_TRANSFORMS - 1; ++i)
            if (t[i] != t[i + 1]) return true;
        return false;
    }

    static constexpr int MAX_TRANSFORMS = 2;
    static constexpr int START_TRANSFORM_BITS = 1 << 0;
    static constexpr int END_TRANSFORM_BITS = 1 << 1;
    static constexpr int ALL_TRANSFORM_BITS = (1 << MAX_TRANSFORMS) - 1;

private:
    Transform t[MAX_TRANSFORMS];
};

class TransformCache {
public:
    TransformCache() : hashTable(512), hashTableOccupancy(0) {}
    Transform * lookup(const Transform &t);
    void clear();

private:
    void insert(Transform *tNew);
    void grow();

    static uint64_t hash(const Transform &t);

    vector<Transform *> hashTable;
    int hashTableOccupancy;
    MemoryArena arena;
};

struct MaterialInstance {
    MaterialInstance() = default;
    MaterialInstance(const string &name, const shared_ptr<Material> &mtl,
                     ParamSet params)
        : name(name), material(mtl), params(move(params)) {}

    string name;
    shared_ptr<Material> material;
    ParamSet params;
};

struct GraphicsState {
    GraphicsState();
    shared_ptr<Material> getMaterialForShape(const ParamSet &geomParams);
    MediumInterface createMediumInterface();

    string currentInsideMedium, currentOutsideMedium;

    typedef map<string, shared_ptr<Texture<Float>>> FloatTextureMap;
    shared_ptr<FloatTextureMap> floatTextures;
    bool floatTexturesShared = false;

    typedef map<string, shared_ptr<Texture<Spectrum>>> SpectrumTextureMap;
    shared_ptr<SpectrumTextureMap> spectrumTextures;
    bool spectrumTexturesShared = false;

    typedef map<string, shared_ptr<MaterialInstance>> NamedMaterialMap;
    shared_ptr<NamedMaterialMap> namedMaterials;
    bool namedMaterialsShared = false;

    shared_ptr<MaterialInstance> currentMaterial;
    ParamSet areaLightParams;
    string areaLight;
    bool reverseOrientation = false;
};


struct RenderOptions {
    Integrator *makeIntegrator() const;
    Scene *makeScene();
    Camera *makeCamera() const;

    Float transformStartTime = 0, transformEndTime = 1;
    string filterName = "box";
    ParamSet filterParams;
    string filmName = "image";
    ParamSet filmParams;
    string samplerName = "halton";
    ParamSet samplerParams;
    string acceleratorName = "bvh";
    ParamSet acceleratorParams;
    string integratorName = "path";
    ParamSet integratorParams;
    string cameraName = "perspective";
    ParamSet cameraParams;
    TransformSet cameraToWorld;
    map<string, shared_ptr<Medium>> namedMedia;
    vector<shared_ptr<Light>> lights;
    vector<shared_ptr<Primitive>> primitives;
    map<string, vector<shared_ptr<Primitive>>> instances;
    vector<shared_ptr<Primitive>> *currentInstance = nullptr;
    bool haveScatteringMedia = false;
};



#endif // RENDERER_H
