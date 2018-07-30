#include "api.h"
#include "renderer.h"
#include "accelerators/bvh.h"
#include "textures/imagemap.h"
#include "stats.h"

using namespace Renderer;

namespace API {

enum class APIState { Uninitialized, OptionsBlock, WorldBlock };
static APIState currentApiState = APIState::Uninitialized;

};

// API Macros
#define SHOULD_PRINT if (options.cat || options.toPly)
#define VERIFY_INITIALIZED(func)                           \
    if (!(options.cat || options.toPly) &&           \
        currentApiState == APIState::Uninitialized) {        \
        ERROR(                                             \
            "API::init() must be before calling \"%s()\". " \
            "Ignoring.",                                   \
            func);                                         \
        return;                                            \
    } else /* swallow trailing semicolon */

#define VERIFY_OPTIONS(func)                             \
    VERIFY_INITIALIZED(func);                            \
    if (!(options.cat || options.toPly) &&       \
        currentApiState == APIState::WorldBlock) {       \
        ERROR(                                           \
            "Options cannot be set inside world block; " \
            "\"%s\" not allowed.  Ignoring.",            \
            func);                                       \
        return;                                          \
    } else /* swallow trailing semicolon */

#define VERIFY_WORLD(func)                                   \
    VERIFY_INITIALIZED(func);                                \
    if (!(options.cat || options.toPly) &&           \
        currentApiState == APIState::OptionsBlock) {         \
        ERROR(                                               \
            "Scene description must be inside world block; " \
            "\"%s\" not allowed. Ignoring.",                 \
            func);                                           \
        return;                                              \
    } else /* swallow trailing semicolon */

#define FOR_ACTIVE_TRANSFORMS(expr)           \
    for (int i = 0; i < TransformSet::MAX_TRANSFORMS; ++i)   \
        if (activeTransformBits & (1 << i)) { \
            expr                              \
        }

#define WARN_IF_ANIMATED_TRANSFORM(func)                             \
    do {                                                             \
        if (curTransform.isAnimated())                               \
            WARNING(                                                 \
                "Animated transformations set; ignoring for \"%s\" " \
                "and using the start transform only",                \
                func);                                               \
    } while (false) /* swallow trailing semicolon */

// API Methods
void API::init(const Options &opt) {
    options = opt;
    // API Initialization
    if (currentApiState != APIState::Uninitialized)
        ERROR("API::init() has already been called.");
    currentApiState = APIState::OptionsBlock;
    renderOptions.reset(new RenderOptions);
    graphicsState = GraphicsState();
    catIndentCount = 0;

    SampledSpectrum::init();
    Parallel::init();  // Threads must be launched before the profiler is initialized.
    Profiler::init();
}

void API::cleanup() {
    if (currentApiState == APIState::Uninitialized)
        ERROR("pbrtCleanup() called without pbrtInit().");
    else if (currentApiState == APIState::WorldBlock)
        ERROR("pbrtCleanup() called while inside world block.");
    currentApiState = APIState::Uninitialized;
    Parallel::cleanup();
    Profiler::cleanup();
}

void API::identity() {
    VERIFY_INITIALIZED("Identity");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = Transform();)
    if (options.cat || options.toPly)
        printf("%*sIdentity\n", catIndentCount, "");
}

void API::transform(float tr[16]) {
    VERIFY_INITIALIZED("Transform");
    FOR_ACTIVE_TRANSFORMS(
        curTransform[i] = Transform(Matrix4x4(
            tr[0], tr[4], tr[8], tr[12],
            tr[1], tr[5], tr[9], tr[13],
            tr[2],tr[6], tr[10], tr[14],
            tr[3], tr[7], tr[11], tr[15]));)
    if (options.cat || options.toPly) {
        printf("%*sTransform [ ", catIndentCount, "");
        for (int i = 0; i < 16; ++i) printf("%.9g ", tr[i]);
        printf(" ]\n");
    }
}

void API::concatTransform(float tr[16]) {
    VERIFY_INITIALIZED("ConcatTransform");
    FOR_ACTIVE_TRANSFORMS(
        curTransform[i] =
            curTransform[i] *
            Transform(Matrix4x4(tr[0], tr[4], tr[8], tr[12], tr[1], tr[5],
                                tr[9], tr[13], tr[2], tr[6], tr[10], tr[14],
                                tr[3], tr[7], tr[11], tr[15]));)
    if (options.cat || options.toPly) {
        printf("%*sConcatTransform [ ", catIndentCount, "");
        for (int i = 0; i < 16; ++i) printf("%.9g ", tr[i]);
        printf(" ]\n");
    }
}

void API::translate(float dx, float dy, float dz) {
    VERIFY_INITIALIZED("Translate");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = curTransform[i] *
                                            Transform::translate(Vector3f(dx, dy, dz));)
    if (options.cat || options.toPly)
        printf("%*sTranslate %.9g %.9g %.9g\n", catIndentCount, "", dx, dy, dz);
}

void API::rotate(float angle, float dx, float dy, float dz) {
    VERIFY_INITIALIZED("Rotate");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = curTransform[i] *
                                                      Transform::rotate(angle, Vector3f(dx, dy, dz));)
    if (options.cat || options.toPly)
        printf("%*sRotate %.9g %.9g %.9g %.9g\n", catIndentCount, "", angle,
               dx, dy, dz);
}

void API::scale(float sx, float sy, float sz) {
    VERIFY_INITIALIZED("Scale");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = curTransform[i] *
                                                      Transform::scale(sx, sy, sz);)
    if (options.cat || options.toPly)
        printf("%*sScale %.9g %.9g %.9g\n", catIndentCount, "", sx, sy, sz);
}

void API::lookAt(float ex, float ey, float ez, float lx, float ly, float lz,
                float ux, float uy, float uz) {
    VERIFY_INITIALIZED("LookAt");
    auto lookAt = Transform::lookAt(Point3f(ex, ey, ez), Point3f(lx, ly, lz), Vector3f(ux, uy, uz));
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = curTransform[i] * lookAt;);
    if (options.cat || options.toPly)
        printf(
            "%*sLookAt %.9g %.9g %.9g\n%*s%.9g %.9g %.9g\n"
            "%*s%.9g %.9g %.9g\n",
            catIndentCount, "", ex, ey, ez, catIndentCount + 8, "", lx, ly, lz,
            catIndentCount + 8, "", ux, uy, uz);
}

void API::coordinateSystem(const string &name) {
    VERIFY_INITIALIZED("CoordinateSystem");
    namedCoordinateSystems[name] = curTransform;
    if (options.cat || options.toPly)
        printf("%*sCoordinateSystem \"%s\"\n", catIndentCount, "",
               name.c_str());
}

void API::coordSysTransform(const string &name) {
    VERIFY_INITIALIZED("CoordSysTransform");
    if (namedCoordinateSystems.find(name) != namedCoordinateSystems.end())
        curTransform = namedCoordinateSystems[name];
    else
        WARNING("Couldn't find named coordinate system \"%s\"", name.c_str());
    if (options.cat || options.toPly)
        printf("%*sCoordSysTransform \"%s\"\n", catIndentCount, "",
               name.c_str());
}

void API::activeTransformAll() {
    activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
    if (options.cat || options.toPly)
        printf("%*sActiveTransform All\n", catIndentCount, "");
}

void API::activeTransformEndTime() {
    activeTransformBits = TransformSet::END_TRANSFORM_BITS;
    if (options.cat || options.toPly)
        printf("%*sActiveTransform EndTime\n", catIndentCount, "");
}

void API::activeTransformStartTime() {
    activeTransformBits = TransformSet::START_TRANSFORM_BITS;
    if (options.cat || options.toPly)
        printf("%*sActiveTransform StartTime\n", catIndentCount, "");
}

void API::transformTimes(float start, float end) {
    VERIFY_OPTIONS("TransformTimes");
    renderOptions->transformStartTime = start;
    renderOptions->transformEndTime = end;
    if (options.cat || options.toPly)
        printf("%*sTransformTimes %.9g %.9g\n", catIndentCount, "", start,
               end);
}

void API::pixelFilter(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("PixelFilter");
    renderOptions->filterName = name;
    renderOptions->filterParams = params;
    if (options.cat || options.toPly) {
        printf("%*sPixelFilter \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::film(const string &type, const ParamSet &params) {
    VERIFY_OPTIONS("Film");
    renderOptions->filmParams = params;
    renderOptions->filmName = type;
    if (options.cat || options.toPly) {
        printf("%*sFilm \"%s\" ", catIndentCount, "", type.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::sampler(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Sampler");
    renderOptions->samplerName = name;
    renderOptions->samplerParams = params;
    if (options.cat || options.toPly) {
        printf("%*sSampler \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::accelerator(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Accelerator");
    renderOptions->acceleratorName = name;
    renderOptions->acceleratorParams = params;
    if (options.cat || options.toPly) {
        printf("%*sAccelerator \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::integrator(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Integrator");
    renderOptions->integratorName = name;
    renderOptions->integratorParams = params;
    if (options.cat || options.toPly) {
        printf("%*sIntegrator \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::camera(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Camera");
    renderOptions->cameraName = name;
    renderOptions->cameraParams = params;
    renderOptions->cameraToWorld = curTransform.inverse();
    namedCoordinateSystems["camera"] = renderOptions->cameraToWorld;
    if (options.cat || options.toPly) {
        printf("%*sCamera \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::makeNamedMedium(const string &name, const ParamSet &params) {
    VERIFY_INITIALIZED("MakeNamedMedium");
    WARN_IF_ANIMATED_TRANSFORM("MakeNamedMedium");
    string type = params.findOneString("type", "");
    if (type == "")
        ERROR("No parameter string \"type\" found in MakeNamedMedium");
    else {
        shared_ptr<Medium> medium = makeMedium(type, params, curTransform[0]);
        if (medium) renderOptions->namedMedia[name] = medium;
    }
    if (options.cat || options.toPly) {
        printf("%*sMakeNamedMedium \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::mediumInterface(const string &insideName, const string &outsideName) {
    VERIFY_INITIALIZED("MediumInterface");
    graphicsState.currentInsideMedium = insideName;
    graphicsState.currentOutsideMedium = outsideName;
    renderOptions->haveScatteringMedia = true;
    if (options.cat || options.toPly)
        printf("%*sMediumInterface \"%s\" \"%s\"\n", catIndentCount, "", insideName.c_str(),
               outsideName.c_str());
}

void API::worldBegin() {
    VERIFY_OPTIONS("WorldBegin");
    currentApiState = APIState::WorldBlock;
    for (int i = 0; i < TransformSet::MAX_TRANSFORMS; ++i)
        curTransform[i] = Transform();
    activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
    namedCoordinateSystems["world"] = curTransform;
    if (options.cat || options.toPly)
        printf("\n\nWorldBegin\n\n");
}

void API::attributeBegin() {
    VERIFY_WORLD("AttributeBegin");
    pushedGraphicsStates.push_back(graphicsState);
    graphicsState.floatTexturesShared = graphicsState.spectrumTexturesShared =
        graphicsState.namedMaterialsShared = true;
    pushedTransforms.push_back(curTransform);
    pushedActiveTransformBits.push_back(activeTransformBits);
    if (options.cat || options.toPly) {
        printf("\n%*sAttributeBegin\n", catIndentCount, "");
        catIndentCount += 4;
    }
}

void API::attributeEnd() {
    VERIFY_WORLD("AttributeEnd");
    if (!pushedGraphicsStates.size()) {
        ERROR(
            "Unmatched pbrtAttributeEnd() encountered. "
            "Ignoring it.");
        return;
    }
    graphicsState = move(pushedGraphicsStates.back());
    pushedGraphicsStates.pop_back();
    curTransform = pushedTransforms.back();
    pushedTransforms.pop_back();
    activeTransformBits = pushedActiveTransformBits.back();
    pushedActiveTransformBits.pop_back();
    if (options.cat || options.toPly) {
        catIndentCount -= 4;
        printf("%*sAttributeEnd\n", catIndentCount, "");
    }
}

void API::transformBegin() {
    VERIFY_WORLD("TransformBegin");
    pushedTransforms.push_back(curTransform);
    pushedActiveTransformBits.push_back(activeTransformBits);
    if (options.cat || options.toPly) {
        printf("%*sTransformBegin\n", catIndentCount, "");
        catIndentCount += 4;
    }
}

void API::transformEnd() {
    VERIFY_WORLD("TransformEnd");
    if (!pushedTransforms.size()) {
        ERROR(
            "Unmatched pbrtTransformEnd() encountered. "
            "Ignoring it.");
        return;
    }
    curTransform = pushedTransforms.back();
    pushedTransforms.pop_back();
    activeTransformBits = pushedActiveTransformBits.back();
    pushedActiveTransformBits.pop_back();
    if (options.cat || options.toPly) {
        catIndentCount -= 4;
        printf("%*sTransformEnd\n", catIndentCount, "");
    }
}

void API::texture(const string &name, const string &type,
                 const string &texname, const ParamSet &params) {
    VERIFY_WORLD("Texture");
    if (options.cat || options.toPly) {
        printf("%*sTexture \"%s\" \"%s\" \"%s\" ", catIndentCount, "",
               name.c_str(), type.c_str(), texname.c_str());
        params.print(catIndentCount);
        printf("\n");
        return;
    }

    TextureParams tp(params, params, *graphicsState.floatTextures,
                     *graphicsState.spectrumTextures);
    if (type == "float") {
        // Create _Float_ texture and store in _floatTextures_
        if (graphicsState.floatTextures->find(name) !=
            graphicsState.floatTextures->end())
            WARNING("Texture \"%s\" being redefined", name.c_str());
        WARN_IF_ANIMATED_TRANSFORM("Texture");
        shared_ptr<Texture<float>> ft = makeTexture<float>(texname, curTransform[0], tp);
        if (ft) {
            // TODO: move this to be a GraphicsState method, also don't
            // provide direct floatTextures access?
            if (graphicsState.floatTexturesShared) {
                graphicsState.floatTextures =
                    make_shared<GraphicsState::FloatTextureMap>(*graphicsState.floatTextures);
                graphicsState.floatTexturesShared = false;
            }
            (*graphicsState.floatTextures)[name] = ft;
        }
    } else if (type == "color" || type == "spectrum") {
        // Create _color_ texture and store in _spectrumTextures_
        if (graphicsState.spectrumTextures->find(name) !=
            graphicsState.spectrumTextures->end())
            WARNING("Texture \"%s\" being redefined", name.c_str());
        WARN_IF_ANIMATED_TRANSFORM("Texture");
        shared_ptr<Texture<Spectrum>> st = makeTexture<Spectrum>(texname, curTransform[0], tp);
        if (st) {
            if (graphicsState.spectrumTexturesShared) {
                graphicsState.spectrumTextures =
                    make_shared<GraphicsState::SpectrumTextureMap>(*graphicsState.spectrumTextures);
                graphicsState.spectrumTexturesShared = false;
            }
            (*graphicsState.spectrumTextures)[name] = st;
        }
    } else
        ERROR("Texture type \"%s\" unknown.", type.c_str());
}

void API::material(const string &name, const ParamSet &params) {
    VERIFY_WORLD("Material");
    ParamSet emptyParams;
    TextureParams mp(params, emptyParams, *graphicsState.floatTextures, *graphicsState.spectrumTextures);
    shared_ptr<Material> mtl = makeMaterial(name, mp);
    graphicsState.currentMaterial = make_shared<MaterialInstance>(name, mtl, params);

    if (options.cat || options.toPly) {
        printf("%*sMaterial \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::makeNamedMaterial(const string &name, const ParamSet &params) {
    VERIFY_WORLD("MakeNamedMaterial");
    // error checking, warning if replace, what to use for transform?
    ParamSet emptyParams;
    TextureParams mp(params, emptyParams, *graphicsState.floatTextures, *graphicsState.spectrumTextures);
    string matName = mp.findString("type");
    WARN_IF_ANIMATED_TRANSFORM("MakeNamedMaterial");
    if (matName == "")
        ERROR("No parameter string \"type\" found in MakeNamedMaterial");

    if (options.cat || options.toPly) {
        printf("%*sMakeNamedMaterial \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    } else {
        shared_ptr<Material> mtl = makeMaterial(matName, mp);
        if (graphicsState.namedMaterials->find(name) != graphicsState.namedMaterials->end())
            WARNING("Named material \"%s\" redefined.", name.c_str());
        if (graphicsState.namedMaterialsShared) {
            graphicsState.namedMaterials =
                    make_shared<GraphicsState::NamedMaterialMap>(*graphicsState.namedMaterials);
            graphicsState.namedMaterialsShared = false;
        }
        (*graphicsState.namedMaterials)[name] = make_shared<MaterialInstance>(matName, mtl, params);
    }
}

void API::namedMaterial(const string &name) {
    VERIFY_WORLD("NamedMaterial");
    if (options.cat || options.toPly) {
        printf("%*sNamedMaterial \"%s\"\n", catIndentCount, "", name.c_str());
        return;
    }

    auto iter = graphicsState.namedMaterials->find(name);
    if (iter == graphicsState.namedMaterials->end()) {
        ERROR("NamedMaterial \"%s\" unknown.", name.c_str());
        return;
    }
    graphicsState.currentMaterial = iter->second;
}

void API::lightSource(const string &name, const ParamSet &params) {
    VERIFY_WORLD("LightSource");
    WARN_IF_ANIMATED_TRANSFORM("LightSource");
    MediumInterface mi = graphicsState.createMediumInterface();
    shared_ptr<Light> lt = makeLight(name, params, curTransform[0], mi);
    if (!lt)
        ERROR("LightSource: light type \"%s\" unknown.", name.c_str());
    else
        renderOptions->lights.push_back(lt);
    if (options.cat || options.toPly) {
        printf("%*sLightSource \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::areaLightSource(const string &name, const ParamSet &params) {
    VERIFY_WORLD("AreaLightSource");
    graphicsState.areaLight = name;
    graphicsState.areaLightParams = params;
    if (options.cat || options.toPly) {
        printf("%*sAreaLightSource \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }
}

void API::shape(const string &name, const ParamSet &params) {
    VERIFY_WORLD("Shape");
    vector<shared_ptr<Primitive>> prims;
    vector<shared_ptr<AreaLight>> areaLights;
    if (options.cat || (options.toPly && name != "trianglemesh")) {
        printf("%*sShape \"%s\" ", catIndentCount, "", name.c_str());
        params.print(catIndentCount);
        printf("\n");
    }

    if (!curTransform.isAnimated()) {
        // Initialize _prims_ and _areaLights_ for static shape

        // Create shapes for shape _name_
        Transform *ObjToWorld = transformCache.lookup(curTransform[0]);
        Transform *WorldToObj = transformCache.lookup(curTransform[0].inverse());
        vector<shared_ptr<Shape>> shapes = makeShapes(name, ObjToWorld, WorldToObj,
                                                      graphicsState.reverseOrientation, params);
        if (shapes.empty()) return;
        shared_ptr<Material> mtl = graphicsState.getMaterialForShape(params);
        params.reportUnused();
        MediumInterface mi = graphicsState.createMediumInterface();
        prims.reserve(shapes.size());
        for (auto s : shapes) {
            // Possibly create area light for shape
            shared_ptr<AreaLight> area;
            if (graphicsState.areaLight != "") {
                area = makeAreaLight(graphicsState.areaLight, curTransform[0], mi,
                        graphicsState.areaLightParams, s);
                if (area) areaLights.push_back(area);
            }
            prims.push_back(make_shared<GeometricPrimitive>(s, mtl, area, mi));
        }
    } else {
        // Initialize _prims_ and _areaLights_ for animated shape

        // Create initial shape or shapes for animated shape
        if (graphicsState.areaLight != "")
            WARNING(
                "Ignoring currently set area light when creating "
                "animated shape");
        Transform *identity = transformCache.lookup(Transform());
        auto shapes = makeShapes(name, identity, identity, graphicsState.reverseOrientation, params);
        if (shapes.empty()) return;

        // Create _GeometricPrimitive_(s) for animated shape
        shared_ptr<Material> mtl = graphicsState.getMaterialForShape(params);
        params.reportUnused();
        MediumInterface mi = graphicsState.createMediumInterface();
        prims.reserve(shapes.size());
        for (auto s : shapes)
            prims.push_back(make_shared<GeometricPrimitive>(s, mtl, nullptr, mi));

        // Create single _TransformedPrimitive_ for _prims_

        // Get _animatedObjectToWorld_ transform for shape
        static_assert(TransformSet::MAX_TRANSFORMS == 2,
                      "TransformCache assumes only two transforms");
        Transform *ObjToWorld[2] = {
            transformCache.lookup(curTransform[0]),
            transformCache.lookup(curTransform[1])
        };
        AnimatedTransform animatedObjectToWorld(
            ObjToWorld[0], renderOptions->transformStartTime, ObjToWorld[1],
            renderOptions->transformEndTime);
        if (prims.size() > 1) {
            shared_ptr<Primitive> bvh = make_shared<BVH>(prims);
            prims.clear();
            prims.push_back(bvh);
        }
        prims[0] = make_shared<TransformedPrimitive>(prims[0], animatedObjectToWorld);
    }
    // Add _prims_ and _areaLights_ to scene or current instance
    if (renderOptions->currentInstance) {
        if (areaLights.size())
            WARNING("Area lights not supported with object instancing");
        renderOptions->currentInstance->insert(
            renderOptions->currentInstance->end(), prims.begin(), prims.end());
    } else {
        renderOptions->primitives.insert(renderOptions->primitives.end(), prims.begin(), prims.end());
        if (areaLights.size())
            renderOptions->lights.insert(renderOptions->lights.end(), areaLights.begin(), areaLights.end());
    }
}

void API::reverseOrientation() {
    VERIFY_WORLD("ReverseOrientation");
    graphicsState.reverseOrientation = !graphicsState.reverseOrientation;
    if (options.cat || options.toPly)
        printf("%*sReverseOrientation\n", catIndentCount, "");
}

void API::objectBegin(const string &name) {
    VERIFY_WORLD("ObjectBegin");
    attributeBegin();
    if (renderOptions->currentInstance)
        ERROR("ObjectBegin called inside of instance definition");
    renderOptions->instances[name] = vector<shared_ptr<Primitive>>();
    renderOptions->currentInstance = &renderOptions->instances[name];
    if (options.cat || options.toPly)
        printf("%*sObjectBegin \"%s\"\n", catIndentCount, "", name.c_str());
}

STAT_COUNTER("Scene/Object instances created", nObjectInstancesCreated);

void API::objectEnd() {
    VERIFY_WORLD("ObjectEnd");
    if (!renderOptions->currentInstance)
        ERROR("ObjectEnd called outside of instance definition");
    renderOptions->currentInstance = nullptr;
    attributeEnd();
    ++nObjectInstancesCreated;
    if (options.cat || options.toPly)
        printf("%*sObjectEnd\n", catIndentCount, "");
}

STAT_COUNTER("Scene/Object instances used", nObjectInstancesUsed);

void API::objectInstance(const string &name) {
    VERIFY_WORLD("ObjectInstance");
    if (options.cat || options.toPly) {
        printf("%*sObjectInstance \"%s\"\n", catIndentCount, "", name.c_str());
        return;
    }

    // Perform object instance error checking
    if (renderOptions->currentInstance) {
        ERROR("ObjectInstance can't be called inside instance definition");
        return;
    }
    if (renderOptions->instances.find(name) == renderOptions->instances.end()) {
        ERROR("Unable to find instance named \"%s\"", name.c_str());
        return;
    }
    vector<shared_ptr<Primitive>> &in =
        renderOptions->instances[name];
    if (in.empty()) return;
    ++nObjectInstancesUsed;
    if (in.size() > 1) {
        // Create aggregate for instance _Primitive_s
        shared_ptr<Primitive> accel(
            makeAccelerator(renderOptions->acceleratorName, move(in),
                                      renderOptions->acceleratorParams));
        if (!accel) accel = make_shared<BVH>(in);
        in.clear();
        in.push_back(accel);
    }
    static_assert(TransformSet::MAX_TRANSFORMS == 2,
                  "TransformCache assumes only two transforms");
    // Create _animatedInstanceToWorld_ transform for instance
    Transform *InstanceToWorld[2] = {
        transformCache.lookup(curTransform[0]),
        transformCache.lookup(curTransform[1])
    };
    AnimatedTransform animatedInstanceToWorld(
        InstanceToWorld[0], renderOptions->transformStartTime,
        InstanceToWorld[1], renderOptions->transformEndTime);
    shared_ptr<Primitive> prim(make_shared<TransformedPrimitive>(in[0], animatedInstanceToWorld));
    renderOptions->primitives.push_back(prim);
}

void API::worldEnd() {
    VERIFY_WORLD("WorldEnd");
    // Ensure there are no pushed graphics states
    while (pushedGraphicsStates.size()) {
        WARNING("Missing end to API::attributeBegin()");
        pushedGraphicsStates.pop_back();
        pushedTransforms.pop_back();
    }
    while (pushedTransforms.size()) {
        WARNING("Missing end to API::transformBegin()");
        pushedTransforms.pop_back();
    }

    // Create scene and render
    if (options.cat || options.toPly) {
        printf("%*sWorldEnd\n", catIndentCount, "");
    } else {
        unique_ptr<Integrator> integrator(renderOptions->makeIntegrator());
        unique_ptr<Scene> scene(renderOptions->makeScene());

        CHECK_EQ(Profiler::state, Profiler::stageToBits(Stage::SceneConstruction));
        Profiler::state = Profiler::stageToBits(Stage::IntegratorRender);

        if (scene && integrator) integrator->render(*scene);

        CHECK_EQ(Profiler::state, Profiler::stageToBits(Stage::IntegratorRender));
        Profiler::state = Profiler::stageToBits(Stage::SceneConstruction);
    }

    // Clean up after rendering. Do this before reporting stats so that
    // destructors can run and update stats as needed.
    graphicsState = GraphicsState();
    transformCache.clear();
    currentApiState = APIState::OptionsBlock;
    ImageTexture<float, float>::clearCache(); 
    ImageTexture<RGBSpectrum, Spectrum>::clearCache();
    renderOptions.reset(new RenderOptions);

    if (!options.cat && !options.toPly) {
        Parallel::mergeWorkerThreadStats();
        StatsRegisterer::reportThread();
        if (!options.quiet) {
            StatsRegisterer::print(stdout);
            Profiler::reportResults(stdout);
            StatsRegisterer::clear();
            Profiler::clear();
        }
    }

    for (int i = 0; i < TransformSet::MAX_TRANSFORMS; ++i)
        curTransform[i] = Transform();
    activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
    namedCoordinateSystems.erase(namedCoordinateSystems.begin(), namedCoordinateSystems.end());
}

