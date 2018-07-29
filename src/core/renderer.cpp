    #include "api.h"
#include "core/renderer.h"
#include "paramset.h"

#include "shapes/triangle.h"
#include "shapes/sphere.h"
#include "shapes/cylinder.h"
#include "shapes/disk.h"
#include "shapes/curve.h"
#include "accelerators/bvh.h"
#include "accelerators/kdtree.h"
#include "cameras/environment.h"
#include "cameras/ortho.h"
#include "cameras/perspective.h"
#include "samplers/stratified.h"
#include "samplers/halton.h"
#include "core/filter.h"
#include "core/film.h"
#include "materials/matte.h"
#include "materials/plastic.h"
#include "materials/mix.h"
#include "textures/mix.h"
#include "textures/imagemap.h"
#include "textures/uv.h"
#include "textures/checkerboard.h"

namespace Renderer {

Options options;
int catIndentCount = 0;
TransformSet curTransform;
uint32_t activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
map<string, TransformSet> namedCoordinateSystems;
unique_ptr<RenderOptions> renderOptions;
GraphicsState graphicsState;
vector<GraphicsState> pushedGraphicsStates;
vector<TransformSet> pushedTransforms;
vector<uint32_t> pushedActiveTransformBits;
TransformCache transformCache;

vector<shared_ptr<Shape>> makeShapes(const string &name, const Transform *object2world,
                                     const Transform *world2object, bool reverseOrientation,
                                     const ParamSet &paramSet)
{
    vector<shared_ptr<Shape>> shapes;
    shared_ptr<Shape> s;

    if (name == "sphere")
        s = Sphere::create(object2world, world2object, reverseOrientation,paramSet);
    // Create remaining single _Shape_ types
    else if (name == "cylinder")
        s = Cylinder::create(object2world, world2object, reverseOrientation, paramSet);
    else if (name == "disk")
        s = Disk::create(object2world, world2object, reverseOrientation, paramSet);
    if (s != nullptr) shapes.push_back(s);

    // Create multiple-_Shape_ types
    else if (name == "curve")
        shapes = Curve::create(object2world, world2object, reverseOrientation, paramSet);
    else if (name == "trianglemesh") {
        shapes = TriangleMesh::create(object2world, world2object, reverseOrientation, paramSet,
                                      &*graphicsState.floatTextures);
        // TODO: Write to PLY files
    }
    else
        WARNING("Shape \"%s\" unknown.", name.c_str());
    return shapes;
}

shared_ptr<Primitive> makeAccelerator(const string &name, vector<shared_ptr<Primitive>> prims,
                                      const ParamSet &paramSet)
{
    shared_ptr<Primitive> accel;
    if (name == "bvh")
        accel = BVH::create(move(prims), paramSet);
    else if (name == "kdtree")
        accel = KDTree::create(move(prims), paramSet);
    else
        WARNING("Accelerator \"%s\" unknown.", name.c_str());
    paramSet.reportUnused();
    return accel;
}

Camera * makeCamera(const string &name, const ParamSet &paramSet, const TransformSet &cam2worldSet,
                    float transformStart, float transformEnd, Film *film)
{
    Camera *camera = nullptr;
    MediumInterface mediumInterface = graphicsState.createMediumInterface();
    static_assert(TransformSet::MAX_TRANSFORMS == 2, "TransformCache assumes only two transforms");
    Transform *cam2world[2] = {
        transformCache.lookup(cam2worldSet[0]),
        transformCache.lookup(cam2worldSet[1])
    };
    AnimatedTransform animatedCam2World(cam2world[0], transformStart, cam2world[1], transformEnd);
    if (name == "perspective")
        camera = PerspectiveCamera::create(paramSet, animatedCam2World, film, mediumInterface.outside);
    else if (name == "orthographic")
        camera = OrthographicCamera::create(paramSet, animatedCam2World, film, mediumInterface.outside);
    else if (name == "environment")
        camera = EnvironmentCamera::create(paramSet, animatedCam2World, film, mediumInterface.outside);
    else
        WARNING("Camera \"%s\" unknown.", name.c_str());
    paramSet.reportUnused();
    return camera;
}

shared_ptr<Sampler> makeSampler(const string &name, const ParamSet &paramSet, const Film *film) {
    Sampler *sampler = nullptr;
    if (name == "halton")
        sampler = HaltonSampler::create(paramSet, film->getSampleBounds());
    else if (name == "stratified")
        sampler = StratifiedSampler::create(paramSet);
    else
        WARNING("Sampler \"%s\" unknown.", name.c_str());
    paramSet.reportUnused();
    return shared_ptr<Sampler>(sampler);
}

unique_ptr<Filter> makeFilter(const string &name, const ParamSet &paramSet) {
    Filter *filter = nullptr;
    if (name == "box")
        filter = BoxFilter::create(paramSet);
    else if (name == "gaussian")
        filter = GaussianFilter::create(paramSet);
    else if (name == "mitchell")
        filter = MitchellFilter::create(paramSet);
    else if (name == "sinc")
        filter = LanczosSincFilter::create(paramSet);
    else if (name == "triangle")
        filter = TriangleFilter::create(paramSet);
    else {
        ERROR("Filter \"%s\" unknown.", name.c_str());
        exit(1);
    }
    paramSet.reportUnused();
    return unique_ptr<Filter>(filter);
}

Film * makeFilm(const string &name, const ParamSet &paramSet, unique_ptr<Filter> filter) {
    Film *film = nullptr;
    if (name == "image")
        film = Film::create(paramSet, move(filter));
    else
        WARNING("Film \"%s\" unknown.", name.c_str());
    paramSet.reportUnused();
    return film;
}

shared_ptr<Material> makeMaterial(const string &name, const TextureParams &mp) {
    Material *material = nullptr;
    if (name == "" || name == "none")
        return nullptr;
    else if (name == "matte")
        material = MatteMaterial::create(mp);
    else if (name == "plastic")
        material = PlasticMaterial::create(mp);
    else if (name == "mix") {
        string m1 = mp.findString("namedmaterial1", "");
        string m2 = mp.findString("namedmaterial2", "");
        shared_ptr<Material> mat1, mat2;
        if (graphicsState.namedMaterials->find(m1) == graphicsState.namedMaterials->end()) {
            ERROR("Named material \"%s\" undefined.  Using \"matte\"", m1.c_str());
            mat1 = makeMaterial("matte", mp);
        } else
            mat1 = (*graphicsState.namedMaterials)[m1]->material;

        if (graphicsState.namedMaterials->find(m2) == graphicsState.namedMaterials->end()) {
            ERROR("Named material \"%s\" undefined.  Using \"matte\"", m2.c_str());
            mat2 = makeMaterial("matte", mp);
        } else
            mat2 = (*graphicsState.namedMaterials)[m2]->material;

        material = MixMaterial::create(mp, mat1, mat2);
    } else {
        WARNING("Material \"%s\" unknown. Using \"matte\".", name.c_str());
        material = MatteMaterial::create(mp);
    }

    mp.reportUnused();
    if (!material) ERROR("Unable to create material \"%s\"", name.c_str());
    return shared_ptr<Material>(material);
}

template <class T>
shared_ptr<Texture<T>> makeTexture(const string &name, const Transform &tex2world,
                                   const TextureParams &tp)
{
    Texture<T> *tex = nullptr;
    if (name == "constant")
        tex = ConstantTexture<T>::create(tex2world, tp);
    else if (name == "scale")
        tex = ScaleTexture<T, T>::create(tex2world, tp);
    else if (name == "mix")
        tex = MixTexture<T>::create(tex2world, tp);
    else if (name == "bilerp")
        tex = BilerpTexture<T>::create(tex2world, tp);
    else if (name == "imagemap")
        tex = ImageTextureCreator<T>::create(tex2world, tp);
    else if (name == "uv")
        tex = UVTexture<T>::create(tex2world, tp);
    else if (name == "checkerboard")
        tex = CheckerboardTextureCreator<T>::create(tex2world, tp);
    else
        WARNING("Float texture \"%s\" unknown.", name.c_str());
    tp.reportUnused();
    return shared_ptr<Texture<T>>(tex);
}

template
shared_ptr<Texture<float>> makeTexture<float>(const string &name, const Transform &tex2world,
                                              const TextureParams &tp);
template
shared_ptr<Texture<Spectrum>> makeTexture<Spectrum>(const string &name, const Transform &tex2world,
                                                    const TextureParams &tp);

}

STAT_MEMORY_COUNTER("Memory/TransformCache", transformCacheBytes);
STAT_PERCENT("Scene/TransformCache hits", nTransformCacheHits, nTransformCacheLookups);
STAT_INT_DISTRIB("Scene/Probes per TransformCache lookup", transformCacheProbes);

Transform * TransformCache::lookup(const Transform &t) {
    ++nTransformCacheLookups;

    unsigned long offset = hash(t) & (hashTable.size() - 1);
    unsigned step = 1;
    while (true) {
        // Keep looking until we find the Transform or determine that
        // it's not present.
        if (!hashTable[offset] || *hashTable[offset] == t)
            break;
        // Advance using quadratic probing.
        offset = (offset + step * step) & (hashTable.size() - 1);
        ++step;
    }
    REPORT_VALUE(transformCacheProbes, step);
    Transform *tCached = hashTable[offset];
    if (tCached)
        ++nTransformCacheHits;
    else {
        tCached = arena.alloc<Transform>();
        *tCached = t;
        insert(tCached);
    }
    return tCached;
}

void TransformCache::clear() {
    transformCacheBytes += arena.totalAllocated() + hashTable.size() * sizeof(Transform *);
    hashTable.resize(512);
    hashTable.clear();
    hashTableOccupancy = 0;
    arena.reset();
}

void TransformCache::insert(Transform *tNew) {
    if (++hashTableOccupancy == hashTable.size() / 2)
        grow();

    unsigned long offset = hash(*tNew) & (hashTable.size() - 1);
    unsigned step = 1;
    while (true) {
        if (hashTable[offset] == nullptr) {
            hashTable[offset] = tNew;
            return;
        }
        // Advance using quadratic probing.
        offset = (offset + step * step) & (hashTable.size() - 1);
        ++step;
    }
}

void TransformCache::grow() {
    vector<Transform *> newTable(2 * hashTable.size());
    LOG(INFO) << "Growing transform cache hash table to " << newTable.size();

    // Insert current elements into newTable.
    for (auto tEntry : hashTable) {
        if (!tEntry) continue;

        unsigned long offset = hash(*tEntry) & (newTable.size() - 1);
        unsigned step = 1;
        while (true) {
            if (newTable[offset] == nullptr) {
                newTable[offset] = tEntry;
                break;
            }
            // Advance using quadratic probing.
            offset = (offset + step * step) & (hashTable.size() - 1);
            ++step;
        }
    }

    swap(hashTable, newTable);
}

uint64_t TransformCache::hash(const Transform &t) {
    const char *ptr = (const char *)(&t.getMatrix());
    size_t size = sizeof(Matrix4x4);
    uint64_t hash = 14695981039346656037ull;
    while (size > 0) {
        hash ^= *ptr;
        hash *= 1099511628211ull;
        ++ptr;
        --size;
    }
    return hash;
}

GraphicsState::GraphicsState()
    : floatTextures(make_shared<FloatTextureMap>()), spectrumTextures(make_shared<SpectrumTextureMap>()),
      namedMaterials(make_shared<NamedMaterialMap>())
{
    ParamSet empty;
    TextureParams tp(empty, empty, *floatTextures, *spectrumTextures);
    shared_ptr<Material> mtl(MatteMaterial::create(tp));
    currentMaterial = make_shared<MaterialInstance>("matte", mtl, ParamSet());
}

shared_ptr<Material> GraphicsState::getMaterialForShape(const ParamSet &shapeParams) {
    CHECK(currentMaterial);
    if (shapeParams.shapeMaySetMaterialParameters()) {
        // The shape's parameters will provide values for some of the material parameters.
        TextureParams mp(shapeParams, currentMaterial->params, *floatTextures, *spectrumTextures);
        return Renderer::makeMaterial(currentMaterial->name, mp);
    } else
        return currentMaterial->material;
}

MediumInterface GraphicsState::createMediumInterface() {
    MediumInterface m;
    if (currentInsideMedium != "") {
        if (Renderer::renderOptions->namedMedia.find(currentInsideMedium) !=
            Renderer::renderOptions->namedMedia.end())
            m.inside = Renderer::renderOptions->namedMedia[currentInsideMedium].get();
        else
            ERROR("Named medium \"%s\" undefined.", currentInsideMedium.c_str());
    }
    if (currentOutsideMedium != "") {
        if (Renderer::renderOptions->namedMedia.find(currentOutsideMedium) !=
            Renderer::renderOptions->namedMedia.end())
            m.outside = Renderer::renderOptions->namedMedia[currentOutsideMedium].get();
        else
            ERROR("Named medium \"%s\" undefined.", currentOutsideMedium.c_str());
    }
    return m;
}

Scene * RenderOptions::makeScene() {
    shared_ptr<Primitive> accelerator = Renderer::makeAccelerator(acceleratorName, move(primitives),
                                                                  acceleratorParams);
    if (!accelerator) accelerator = make_shared<BVH>(primitives);
    Scene *scene = new Scene(accelerator, lights);
    // Erase primitives and lights from _RenderOptions_
    primitives.clear();
    lights.clear();
    return scene;
}

Camera * RenderOptions::makeCamera() const {
    unique_ptr<Filter> filter = Renderer::makeFilter(filterName, filterParams);
    Film *film = Renderer::makeFilm(filmName, filmParams, move(filter));
    if (!film) {
        ERROR("Unable to create film.");
        return nullptr;
    }
    Camera *camera = Renderer::makeCamera(cameraName, cameraParams, cameraToWorld,
                                          Renderer::renderOptions->transformStartTime,
                                          Renderer::renderOptions->transformEndTime, film);
    return camera;
}


