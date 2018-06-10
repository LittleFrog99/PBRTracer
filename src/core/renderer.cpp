#include "renderer.h"

Options Renderer::options;
APIState Renderer::currentApiState = APIState::Uninitialized;
uint32_t Renderer::activeTransformBits = ALL_TRANSFORM_BITS;
map<string, TransformSet> Renderer::namedCoordinateSystems;
unique_ptr<RenderOptions> renderOptions;
GraphicsState Renderer::graphicsState;
vector<GraphicsState> Renderer::pushedGraphicsStates;
vector<TransformSet> Renderer::pushedTransforms;
vector<uint32_t> Renderer::pushedActiveTransformBits;
TransformCache Renderer::transformCache;
int Renderer::catIndentCount = 0;
