#ifndef RENDERER_H
#define RENDERER_H

#include "api.h"

struct TransformSet;
struct TransformCache {};
struct RenderOptions;
struct MaterialInstance;
struct GraphicsState {};

enum class APIState { Uninitialized, OptionsBlock, WorldBlock };

class Renderer {
public:
    static Options options;
    static int catIndentCount;

    static constexpr int MAX_TRANSFORMS = 2;
    static constexpr int START_TRANSFORM_BITS = 1 << 0;
    static constexpr int END_TRANSFORM_BITS = 1 << 1;
    static constexpr int ALL_TRANSFORM_BITS = (1 << MAX_TRANSFORMS) - 1;

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

#endif // RENDERER_H
