#include "api.h"
#include "renderer.h"
#include "paramset.h"

#include "accelerators/bvh.h"

Options Renderer::options;
APIState Renderer::currentApiState = APIState::Uninitialized;
uint32_t Renderer::activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
TransformSet Renderer::curTransform;
map<string, TransformSet> Renderer::namedCoordinateSystems;
unique_ptr<RenderOptions> Renderer::renderOptions;
GraphicsState Renderer::graphicsState;
vector<GraphicsState> Renderer::pushedGraphicsStates;
vector<TransformSet> Renderer::pushedTransforms;
vector<uint32_t> Renderer::pushedActiveTransformBits;
TransformCache Renderer::transformCache;
int Renderer::catIndentCount = 0;

// API Macros
#define VERIFY_INITIALIZED(func)                           \
    if (!(Renderer::options.cat || Renderer::options.toPly) &&           \
        Renderer::currentApiState == APIState::Uninitialized) {        \
        ERROR(                                             \
            "pbrtInit() must be before calling \"%s()\". " \
            "Ignoring.",                                   \
            func);                                         \
        return;                                            \
    } else /* swallow trailing semicolon */

#define VERIFY_OPTIONS(func)                             \
    VERIFY_INITIALIZED(func);                            \
    if (!(Renderer::options.cat || Renderer::options.toPly) &&       \
        Renderer::currentApiState == APIState::WorldBlock) {       \
        ERROR(                                           \
            "Options cannot be set inside world block; " \
            "\"%s\" not allowed.  Ignoring.",            \
            func);                                       \
        return;                                          \
    } else /* swallow trailing semicolon */

#define VERIFY_WORLD(func)                                   \
    VERIFY_INITIALIZED(func);                                \
    if (!(Renderer::options.cat || Renderer::options.toPly) &&           \
        Renderer::currentApiState == APIState::OptionsBlock) {         \
        ERROR(                                               \
            "Scene description must be inside world block; " \
            "\"%s\" not allowed. Ignoring.",                 \
            func);                                           \
        return;                                              \
    } else /* swallow trailing semicolon */

#define FOR_ACTIVE_TRANSFORMS(expr)           \
    for (int i = 0; i < TransformSet::MAX_TRANSFORMS; ++i)   \
        if (Renderer::activeTransformBits & (1 << i)) { \
            expr                              \
        }

#define WARN_IF_ANIMATED_TRANSFORM(func)                             \
    do {                                                             \
        if (Renderer::curTransform.isAnimated())                               \
            WARNING(                                                 \
                "Animated transformations set; ignoring for \"%s\" " \
                "and using the start transform only",                \
                func);                                               \
    } while (false) /* swallow trailing semicolon */

// API Methods
void API::init(const Options &opt) {
    Renderer::options = opt;
    // API Initialization
    if (Renderer::currentApiState != APIState::Uninitialized)
        ERROR("pbrtInit() has already been called.");
    Renderer::currentApiState = APIState::OptionsBlock;
    Renderer::renderOptions.reset(new RenderOptions);
    Renderer::graphicsState = GraphicsState();
    Renderer::catIndentCount = 0;

    // SampledSpectrum::init(); // Uncomment when using #SampleSpectrum
    Parallel::init();  // Threads must be launched before the profiler is initialized.
    Profiler::init();
}

void API::cleanup() {
    if (Renderer::currentApiState == APIState::Uninitialized)
        ERROR("pbrtCleanup() called without pbrtInit().");
    else if (Renderer::currentApiState == APIState::WorldBlock)
        ERROR("pbrtCleanup() called while inside world block.");
    Renderer::currentApiState = APIState::Uninitialized;
    Parallel::cleanup();
    Profiler::cleanup();
}

void API::identity() {
    VERIFY_INITIALIZED("Identity");
    FOR_ACTIVE_TRANSFORMS(Renderer::curTransform[i] = Transform();)
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sIdentity\n", Renderer::catIndentCount, "");
}

void API::transform(Float tr[16]) {
    VERIFY_INITIALIZED("Transform");
    FOR_ACTIVE_TRANSFORMS(
        Renderer::curTransform[i] = Transform(Matrix4x4(
                                                  tr[0], tr[4], tr[8], tr[12],
            tr[1], tr[5], tr[9], tr[13],
            tr[2],tr[6], tr[10], tr[14],
            tr[3], tr[7], tr[11], tr[15]));)
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sTransform [ ", Renderer::catIndentCount, "");
        for (int i = 0; i < 16; ++i) printf("%.9g ", tr[i]);
        printf(" ]\n");
    }
}

void API::concatTransform(Float tr[16]) {
    VERIFY_INITIALIZED("ConcatTransform");
    FOR_ACTIVE_TRANSFORMS(
        Renderer::curTransform[i] =
            Renderer::curTransform[i] *
            Transform(Matrix4x4(tr[0], tr[4], tr[8], tr[12], tr[1], tr[5],
                                tr[9], tr[13], tr[2], tr[6], tr[10], tr[14],
                                tr[3], tr[7], tr[11], tr[15]));)
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sConcatTransform [ ", Renderer::catIndentCount, "");
        for (int i = 0; i < 16; ++i) printf("%.9g ", tr[i]);
        printf(" ]\n");
    }
}

void API::rotate(Float angle, Float dx, Float dy, Float dz) {
    VERIFY_INITIALIZED("Rotate");
    FOR_ACTIVE_TRANSFORMS(Renderer::curTransform[i] = Renderer::curTransform[i] *
                                                      Transform::rotate(angle, Vector3f(dx, dy, dz));)
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sRotate %.9g %.9g %.9g %.9g\n", Renderer::catIndentCount, "", angle,
               dx, dy, dz);
}

void API::scale(Float sx, Float sy, Float sz) {
    VERIFY_INITIALIZED("Scale");
    FOR_ACTIVE_TRANSFORMS(Renderer::curTransform[i] = Renderer::curTransform[i] *
                                                      Transform::scale(sx, sy, sz);)
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sScale %.9g %.9g %.9g\n", Renderer::catIndentCount, "", sx, sy, sz);
}

void API::lookAt(Float ex, Float ey, Float ez, Float lx, Float ly, Float lz,
                Float ux, Float uy, Float uz) {
    VERIFY_INITIALIZED("LookAt");
    auto lookAt = Transform::lookAt(Point3f(ex, ey, ez), Point3f(lx, ly, lz), Vector3f(ux, uy, uz));
    FOR_ACTIVE_TRANSFORMS(Renderer::curTransform[i] = Renderer::curTransform[i] * lookAt;);
    if (Renderer::options.cat || Renderer::options.toPly)
        printf(
            "%*sLookAt %.9g %.9g %.9g\n%*s%.9g %.9g %.9g\n"
            "%*s%.9g %.9g %.9g\n",
            Renderer::catIndentCount, "", ex, ey, ez, Renderer::catIndentCount + 8, "", lx, ly, lz,
            Renderer::catIndentCount + 8, "", ux, uy, uz);
}

void API::coordinateSystem(const string &name) {
    VERIFY_INITIALIZED("CoordinateSystem");
    Renderer::namedCoordinateSystems[name] = Renderer::curTransform;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sCoordinateSystem \"%s\"\n", Renderer::catIndentCount, "",
               name.c_str());
}

void API::coordSysTransform(const string &name) {
    VERIFY_INITIALIZED("CoordSysTransform");
    if (Renderer::namedCoordinateSystems.find(name) != Renderer::namedCoordinateSystems.end())
        Renderer::curTransform = Renderer::namedCoordinateSystems[name];
    else
        WARNING("Couldn't find named coordinate system \"%s\"", name.c_str());
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sCoordSysTransform \"%s\"\n", Renderer::catIndentCount, "",
               name.c_str());
}

void API::activeTransformAll() {
    Renderer::activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sActiveTransform All\n", Renderer::catIndentCount, "");
}

void API::activeTransformEndTime() {
    Renderer::activeTransformBits = TransformSet::END_TRANSFORM_BITS;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sActiveTransform EndTime\n", Renderer::catIndentCount, "");
}

void API::activeTransformStartTime() {
    Renderer::activeTransformBits = TransformSet::START_TRANSFORM_BITS;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sActiveTransform StartTime\n", Renderer::catIndentCount, "");
}

void API::transformTimes(Float start, Float end) {
    VERIFY_OPTIONS("TransformTimes");
    Renderer::renderOptions->transformStartTime = start;
    Renderer::renderOptions->transformEndTime = end;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sTransformTimes %.9g %.9g\n", Renderer::catIndentCount, "", start,
               end);
}

void API::pixelFilter(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("PixelFilter");
    Renderer::renderOptions->filterName = name;
    Renderer::renderOptions->filterParams = params;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sPixelFilter \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::film(const string &type, const ParamSet &params) {
    VERIFY_OPTIONS("Film");
    Renderer::renderOptions->filmParams = params;
    Renderer::renderOptions->filmName = type;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sFilm \"%s\" ", Renderer::catIndentCount, "", type.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::sampler(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Sampler");
    Renderer::renderOptions->samplerName = name;
    Renderer::renderOptions->samplerParams = params;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sSampler \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::accelerator(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Accelerator");
    Renderer::renderOptions->acceleratorName = name;
    Renderer::renderOptions->acceleratorParams = params;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sAccelerator \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::integrator(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Integrator");
    Renderer::renderOptions->integratorName = name;
    Renderer::renderOptions->integratorParams = params;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sIntegrator \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::camera(const string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Camera");
    Renderer::renderOptions->cameraName = name;
    Renderer::renderOptions->cameraParams = params;
    Renderer::renderOptions->cameraToWorld = Renderer::curTransform.inverse();
    Renderer::namedCoordinateSystems["camera"] = Renderer::renderOptions->cameraToWorld;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sCamera \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
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
        shared_ptr<Medium> medium =
            Renderer::makeMedium(type, params, Renderer::curTransform[0]);
        if (medium) Renderer::renderOptions->namedMedia[name] = medium;
    }
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sMakeNamedMedium \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::mediumInterface(const string &insideName, const string &outsideName) {
    VERIFY_INITIALIZED("MediumInterface");
    Renderer::graphicsState.currentInsideMedium = insideName;
    Renderer::graphicsState.currentOutsideMedium = outsideName;
    Renderer::renderOptions->haveScatteringMedia = true;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sMediumInterface \"%s\" \"%s\"\n", Renderer::catIndentCount, "",
               insideName.c_str(), outsideName.c_str());
}

void API::worldBegin() {
    VERIFY_OPTIONS("WorldBegin");
    Renderer::currentApiState = APIState::WorldBlock;
    for (int i = 0; i < TransformSet::MAX_TRANSFORMS; ++i)
        Renderer::curTransform[i] = Transform();
    Renderer::activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
    Renderer::namedCoordinateSystems["world"] = Renderer::curTransform;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("\n\nWorldBegin\n\n");
}

void API::attributeBegin() {
    VERIFY_WORLD("AttributeBegin");
    Renderer::pushedGraphicsStates.push_back(Renderer::graphicsState);
    Renderer::graphicsState.floatTexturesShared = Renderer::graphicsState.spectrumTexturesShared =
        Renderer::graphicsState.namedMaterialsShared = true;
    Renderer::pushedTransforms.push_back(Renderer::curTransform);
    Renderer::pushedActiveTransformBits.push_back(Renderer::activeTransformBits);
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("\n%*sAttributeBegin\n", Renderer::catIndentCount, "");
        Renderer::catIndentCount += 4;
    }
}

void API::attributeEnd() {
    VERIFY_WORLD("AttributeEnd");
    if (!Renderer::pushedGraphicsStates.size()) {
        ERROR(
            "Unmatched pbrtAttributeEnd() encountered. "
            "Ignoring it.");
        return;
    }
    Renderer::graphicsState = move(Renderer::pushedGraphicsStates.back());
    Renderer::pushedGraphicsStates.pop_back();
    Renderer::curTransform = Renderer::pushedTransforms.back();
    Renderer::pushedTransforms.pop_back();
    Renderer::activeTransformBits = Renderer::pushedActiveTransformBits.back();
    Renderer::pushedActiveTransformBits.pop_back();
    if (Renderer::options.cat || Renderer::options.toPly) {
        Renderer::catIndentCount -= 4;
        printf("%*sAttributeEnd\n", Renderer::catIndentCount, "");
    }
}

void API::transformBegin() {
    VERIFY_WORLD("TransformBegin");
    Renderer::pushedTransforms.push_back(Renderer::curTransform);
    Renderer::pushedActiveTransformBits.push_back(Renderer::activeTransformBits);
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sTransformBegin\n", Renderer::catIndentCount, "");
        Renderer::catIndentCount += 4;
    }
}

void API::transformEnd() {
    VERIFY_WORLD("TransformEnd");
    if (!Renderer::pushedTransforms.size()) {
        ERROR(
            "Unmatched pbrtTransformEnd() encountered. "
            "Ignoring it.");
        return;
    }
    Renderer::curTransform = Renderer::pushedTransforms.back();
    Renderer::pushedTransforms.pop_back();
    Renderer::activeTransformBits = Renderer::pushedActiveTransformBits.back();
    Renderer::pushedActiveTransformBits.pop_back();
    if (Renderer::options.cat || Renderer::options.toPly) {
        Renderer::catIndentCount -= 4;
        printf("%*sTransformEnd\n", Renderer::catIndentCount, "");
    }
}

void API::texture(const string &name, const string &type,
                 const string &texname, const ParamSet &params) {
    VERIFY_WORLD("Texture");
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sTexture \"%s\" \"%s\" \"%s\" ", Renderer::catIndentCount, "",
               name.c_str(), type.c_str(), texname.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
        return;
    }

    TextureParams tp(params, params, *Renderer::graphicsState.floatTextures,
                     *Renderer::graphicsState.spectrumTextures);
    if (type == "float") {
        // Create _Float_ texture and store in _floatTextures_
        if (Renderer::graphicsState.floatTextures->find(name) !=
            Renderer::graphicsState.floatTextures->end())
            WARNING("Texture \"%s\" being redefined", name.c_str());
        WARN_IF_ANIMATED_TRANSFORM("Texture");
        shared_ptr<Texture<Float>> ft =
            Renderer::makeFloatTexture(texname, Renderer::curTransform[0], tp);
        if (ft) {
            // TODO: move this to be a GraphicsState method, also don't
            // provide direct floatTextures access?
            if (Renderer::graphicsState.floatTexturesShared) {
                Renderer::graphicsState.floatTextures =
                    make_shared<GraphicsState::FloatTextureMap>(*Renderer::graphicsState.floatTextures);
                Renderer::graphicsState.floatTexturesShared = false;
            }
            (*Renderer::graphicsState.floatTextures)[name] = ft;
        }
    } else if (type == "color" || type == "spectrum") {
        // Create _color_ texture and store in _spectrumTextures_
        if (Renderer::graphicsState.spectrumTextures->find(name) !=
            Renderer::graphicsState.spectrumTextures->end())
            WARNING("Texture \"%s\" being redefined", name.c_str());
        WARN_IF_ANIMATED_TRANSFORM("Texture");
        shared_ptr<Texture<Spectrum>> st =
            Renderer::makeSpectrumTexture(texname, Renderer::curTransform[0], tp);
        if (st) {
            if (Renderer::graphicsState.spectrumTexturesShared) {
                Renderer::graphicsState.spectrumTextures =
                    make_shared<GraphicsState::SpectrumTextureMap>(*Renderer::graphicsState.spectrumTextures);
                Renderer::graphicsState.spectrumTexturesShared = false;
            }
            (*Renderer::graphicsState.spectrumTextures)[name] = st;
        }
    } else
        ERROR("Texture type \"%s\" unknown.", type.c_str());
}

void API::material(const string &name, const ParamSet &params) {
    VERIFY_WORLD("Material");
    ParamSet emptyParams;
    TextureParams mp(params, emptyParams, *Renderer::graphicsState.floatTextures,
                     *Renderer::graphicsState.spectrumTextures);
    shared_ptr<Material> mtl = Renderer::makeMaterial(name, mp);
    Renderer::graphicsState.currentMaterial = make_shared<MaterialInstance>(name, mtl, params);

    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sMaterial \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::makeNamedMaterial(const string &name, const ParamSet &params) {
    VERIFY_WORLD("MakeNamedMaterial");
    // error checking, warning if replace, what to use for transform?
    ParamSet emptyParams;
    TextureParams mp(params, emptyParams, *Renderer::graphicsState.floatTextures,
                     *Renderer::graphicsState.spectrumTextures);
    string matName = mp.findString("type");
    WARN_IF_ANIMATED_TRANSFORM("MakeNamedMaterial");
    if (matName == "")
        ERROR("No parameter string \"type\" found in MakeNamedMaterial");

    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sMakeNamedMaterial \"%s\" ", Renderer::catIndentCount, "",
               name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    } else {
        shared_ptr<Material> mtl = Renderer::makeMaterial(matName, mp);
        if (Renderer::graphicsState.namedMaterials->find(name) !=
            Renderer::graphicsState.namedMaterials->end())
            WARNING("Named material \"%s\" redefined.", name.c_str());
        if (Renderer::graphicsState.namedMaterialsShared) {
            Renderer::graphicsState.namedMaterials =
                make_shared<GraphicsState::NamedMaterialMap>(*Renderer::graphicsState.namedMaterials);
            Renderer::graphicsState.namedMaterialsShared = false;
        }
        (*Renderer::graphicsState.namedMaterials)[name] =
            make_shared<MaterialInstance>(matName, mtl, params);
    }
}

void API::namedMaterial(const string &name) {
    VERIFY_WORLD("NamedMaterial");
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sNamedMaterial \"%s\"\n", Renderer::catIndentCount, "", name.c_str());
        return;
    }

    auto iter = Renderer::graphicsState.namedMaterials->find(name);
    if (iter == Renderer::graphicsState.namedMaterials->end()) {
        ERROR("NamedMaterial \"%s\" unknown.", name.c_str());
        return;
    }
    Renderer::graphicsState.currentMaterial = iter->second;
}

void API::lightSource(const string &name, const ParamSet &params) {
    VERIFY_WORLD("LightSource");
    WARN_IF_ANIMATED_TRANSFORM("LightSource");
    MediumInterface mi = Renderer::graphicsState.createMediumInterface();
    shared_ptr<Light> lt = Renderer::makeLight(name, params, Renderer::curTransform[0], mi);
    if (!lt)
        ERROR("LightSource: light type \"%s\" unknown.", name.c_str());
    else
        Renderer::renderOptions->lights.push_back(lt);
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sLightSource \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::areaLightSource(const string &name, const ParamSet &params) {
    VERIFY_WORLD("AreaLightSource");
    Renderer::graphicsState.areaLight = name;
    Renderer::graphicsState.areaLightParams = params;
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sAreaLightSource \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }
}

void API::shape(const string &name, const ParamSet &params) {
    VERIFY_WORLD("Shape");
    vector<shared_ptr<Primitive>> prims;
    vector<shared_ptr<AreaLight>> areaLights;
    if (Renderer::options.cat || (Renderer::options.toPly && name != "trianglemesh")) {
        printf("%*sShape \"%s\" ", Renderer::catIndentCount, "", name.c_str());
        params.print(Renderer::catIndentCount);
        printf("\n");
    }

    if (!Renderer::curTransform.isAnimated()) {
        // Initialize _prims_ and _areaLights_ for static shape

        // Create shapes for shape _name_
        Transform *ObjToWorld = Renderer::transformCache.lookup(Renderer::curTransform[0]);
        Transform *WorldToObj = Renderer::transformCache.lookup(Renderer::curTransform[0].inverse());
        vector<shared_ptr<Shape>> shapes = Renderer::makeShapes(name, ObjToWorld, WorldToObj,
                                                                Renderer::graphicsState.reverseOrientation,
                                                                params);
        if (shapes.empty()) return;
        shared_ptr<Material> mtl = Renderer::graphicsState.getMaterialForShape(params);
        params.reportUnused();
        MediumInterface mi = Renderer::graphicsState.createMediumInterface();
        prims.reserve(shapes.size());
        for (auto s : shapes) {
            // Possibly create area light for shape
            shared_ptr<AreaLight> area;
            if (Renderer::graphicsState.areaLight != "") {
                area = Renderer::makeAreaLight(Renderer::graphicsState.areaLight, Renderer::curTransform[0],
                        mi, Renderer::graphicsState.areaLightParams, s);
                if (area) areaLights.push_back(area);
            }
            prims.push_back(
                make_shared<GeometricPrimitive>(s, mtl, area, mi));
        }
    } else {
        // Initialize _prims_ and _areaLights_ for animated shape

        // Create initial shape or shapes for animated shape
        if (Renderer::graphicsState.areaLight != "")
            WARNING(
                "Ignoring currently set area light when creating "
                "animated shape");
        Transform *identity = Renderer::transformCache.lookup(Transform());
        auto shapes = Renderer::makeShapes(name, identity, identity,
                                           Renderer::graphicsState.reverseOrientation, params);
        if (shapes.empty()) return;

        // Create _GeometricPrimitive_(s) for animated shape
        shared_ptr<Material> mtl = Renderer::graphicsState.getMaterialForShape(params);
        params.reportUnused();
        MediumInterface mi = Renderer::graphicsState.createMediumInterface();
        prims.reserve(shapes.size());
        for (auto s : shapes)
            prims.push_back(make_shared<GeometricPrimitive>(s, mtl, nullptr, mi));

        // Create single _TransformedPrimitive_ for _prims_

        // Get _animatedObjectToWorld_ transform for shape
        static_assert(TransformSet::MAX_TRANSFORMS == 2,
                      "TransformCache assumes only two transforms");
        Transform *ObjToWorld[2] = {
            Renderer::transformCache.lookup(Renderer::curTransform[0]),
            Renderer::transformCache.lookup(Renderer::curTransform[1])
        };
        AnimatedTransform animatedObjectToWorld(
            ObjToWorld[0], Renderer::renderOptions->transformStartTime, ObjToWorld[1],
            Renderer::renderOptions->transformEndTime);
        if (prims.size() > 1) {
            shared_ptr<Primitive> bvh = make_shared<BVH>(prims);
            prims.clear();
            prims.push_back(bvh);
        }
        prims[0] = make_shared<TransformedPrimitive>(prims[0], animatedObjectToWorld);
    }
    // Add _prims_ and _areaLights_ to scene or current instance
    if (Renderer::renderOptions->currentInstance) {
        if (areaLights.size())
            WARNING("Area lights not supported with object instancing");
        Renderer::renderOptions->currentInstance->insert(
            Renderer::renderOptions->currentInstance->end(), prims.begin(), prims.end());
    } else {
        Renderer::renderOptions->primitives.insert(Renderer::renderOptions->primitives.end(),
                                                   prims.begin(), prims.end());
        if (areaLights.size())
            Renderer::renderOptions->lights.insert(Renderer::renderOptions->lights.end(),
                                                   areaLights.begin(), areaLights.end());
    }
}

void API::reverseOrientation() {
    VERIFY_WORLD("ReverseOrientation");
    Renderer::graphicsState.reverseOrientation = !Renderer::graphicsState.reverseOrientation;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sReverseOrientation\n", Renderer::catIndentCount, "");
}

void API::objectBegin(const string &name) {
    VERIFY_WORLD("ObjectBegin");
    attributeBegin();
    if (Renderer::renderOptions->currentInstance)
        ERROR("ObjectBegin called inside of instance definition");
    Renderer::renderOptions->instances[name] = vector<shared_ptr<Primitive>>();
    Renderer::renderOptions->currentInstance = &Renderer::renderOptions->instances[name];
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sObjectBegin \"%s\"\n", Renderer::catIndentCount, "", name.c_str());
}

STAT_COUNTER("Scene/Object instances created", nObjectInstancesCreated);

void API::objectEnd() {
    VERIFY_WORLD("ObjectEnd");
    if (!Renderer::renderOptions->currentInstance)
        ERROR("ObjectEnd called outside of instance definition");
    Renderer::renderOptions->currentInstance = nullptr;
    attributeEnd();
    ++nObjectInstancesCreated;
    if (Renderer::options.cat || Renderer::options.toPly)
        printf("%*sObjectEnd\n", Renderer::catIndentCount, "");
}

STAT_COUNTER("Scene/Object instances used", nObjectInstancesUsed);

void API::objectInstance(const string &name) {
    VERIFY_WORLD("ObjectInstance");
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sObjectInstance \"%s\"\n", Renderer::catIndentCount, "", name.c_str());
        return;
    }

    // Perform object instance error checking
    if (Renderer::renderOptions->currentInstance) {
        ERROR("ObjectInstance can't be called inside instance definition");
        return;
    }
    if (Renderer::renderOptions->instances.find(name) == Renderer::renderOptions->instances.end()) {
        ERROR("Unable to find instance named \"%s\"", name.c_str());
        return;
    }
    vector<shared_ptr<Primitive>> &in =
        Renderer::renderOptions->instances[name];
    if (in.empty()) return;
    ++nObjectInstancesUsed;
    if (in.size() > 1) {
        // Create aggregate for instance _Primitive_s
        shared_ptr<Primitive> accel(
            Renderer::makeAccelerator(Renderer::renderOptions->acceleratorName, move(in),
                                      Renderer::renderOptions->acceleratorParams));
        if (!accel) accel = make_shared<BVH>(in);
        in.clear();
        in.push_back(accel);
    }
    static_assert(TransformSet::MAX_TRANSFORMS == 2,
                  "TransformCache assumes only two transforms");
    // Create _animatedInstanceToWorld_ transform for instance
    Transform *InstanceToWorld[2] = {
        Renderer::transformCache.lookup(Renderer::curTransform[0]),
        Renderer::transformCache.lookup(Renderer::curTransform[1])
    };
    AnimatedTransform animatedInstanceToWorld(
        InstanceToWorld[0], Renderer::renderOptions->transformStartTime,
        InstanceToWorld[1], Renderer::renderOptions->transformEndTime);
    shared_ptr<Primitive> prim(
        make_shared<TransformedPrimitive>(in[0], animatedInstanceToWorld));
    Renderer::renderOptions->primitives.push_back(prim);
}

void API::worldEnd() {
    VERIFY_WORLD("WorldEnd");
    // Ensure there are no pushed graphics states
    while (Renderer::pushedGraphicsStates.size()) {
        WARNING("Missing end to pbrtAttributeBegin()");
        Renderer::pushedGraphicsStates.pop_back();
        Renderer::pushedTransforms.pop_back();
    }
    while (Renderer::pushedTransforms.size()) {
        WARNING("Missing end to pbrtTransformBegin()");
        Renderer::pushedTransforms.pop_back();
    }

    // Create scene and render
    if (Renderer::options.cat || Renderer::options.toPly) {
        printf("%*sWorldEnd\n", Renderer::catIndentCount, "");
    } else {
        unique_ptr<Integrator> integrator(Renderer::renderOptions->makeIntegrator());
        unique_ptr<Scene> scene(Renderer::renderOptions->makeScene());

        // This is kind of ugly; we directly override the current profiler
        // state to switch from parsing/scene construction related stuff to
        // rendering stuff and then switch it back below. The underlying
        // issue is that all the rest of the profiling system assumes
        // hierarchical inheritance of profiling state; this is the only
        // place where that isn't the case.
        CHECK_EQ(Profiler::state, Profiler::profToBits(Profiler::Stage::SceneConstruction));
        Profiler::state = Profiler::profToBits(Profiler::Stage::IntegratorRender);

        if (scene && integrator) integrator->render(*scene);

        CHECK_EQ(Profiler::state, Profiler::profToBits(Profiler::Stage::IntegratorRender));
        Profiler::state = Profiler::profToBits(Profiler::Stage::SceneConstruction);
    }

    // Clean up after rendering. Do this before reporting stats so that
    // destructors can run and update stats as needed.
    Renderer::graphicsState = GraphicsState();
    Renderer::transformCache.clear();
    Renderer::currentApiState = APIState::OptionsBlock;
    // ImageTexture<Float, Float>::ClearCache();
    // ImageTexture<RGBSpectrum, Spectrum>::ClearCache();
    Renderer::renderOptions.reset(new RenderOptions);

    if (!Renderer::options.cat && !Renderer::options.toPly) {
        Parallel::mergeWorkerThreadStats();
        Statistics::reportThread();
        if (!Renderer::options.quiet) {
            Statistics::print(stdout);
            Profiler::reportResults(stdout);
            Statistics::clear();
            Profiler::clear();
        }
    }

    for (int i = 0; i < TransformSet::MAX_TRANSFORMS; ++i)
        Renderer::curTransform[i] = Transform();
    Renderer::activeTransformBits = TransformSet::ALL_TRANSFORM_BITS;
    Renderer::namedCoordinateSystems.erase(Renderer::namedCoordinateSystems.begin(),
                                           Renderer::namedCoordinateSystems.end());
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
