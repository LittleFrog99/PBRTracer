CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(src/utilities)

PROJECT(PBRTracer)

SET(SOURCES_CORE
    src/core/renderer.cpp  src/core/texture.cpp src/core/integrator.cpp
    src/core/interaction.cpp src/core/spectrum.cpp src/core/transform.cpp
    src/core/shape.cpp src/core/primitive.cpp src/core/camera.cpp
    src/core/api.cpp src/core/sampling.cpp src/core/film.cpp
    src/core/bsdf.cpp src/core/scene.cpp src/core/material.cpp
    src/core/texture.cpp src/core/light.cpp)

SET(SOURCES_UTILITIES
    src/utilities/random.cpp src/utilities/quaternion.cpp src/utilities/log.cpp
    src/utilities/matrix.cpp src/utilities/memory.cpp src/utilities/file.cpp
    src/utilities/imageio.cpp src/utilities/parallel.cpp src/utilities/stats.cpp
    src/utilities/paramset.cpp src/utilities/parser.cpp src/utilities/mipmap.cpp)

SET(SOURCES_SHAPES
    src/shapes/sphere.cpp src/shapes/cylinder.cpp src/shapes/disk.cpp
    src/shapes/triangle.cpp src/shapes/curve.cpp)

SET(SOURCES_ACCELERATORS
    src/accelerators/bvh.cpp src/accelerators/kdtree.cpp)

SET(SOURCES_INTEGRATORS
    src/integrators/whitted.cpp)

SET(SOURCES_CAMERAS
    src/cameras/ortho.cpp src/cameras/perspective.cpp src/cameras/environment.cpp)

SET(SOURCES_SAMPLERS
    src/samplers/stratified.cpp src/samplers/halton.cpp)

SET(SOURCES_BSDFS
    src/bsdfs/fresnel.cpp src/bsdfs/microfacet.cpp)

SET(SOURCES_MATERIALS
    src/materials/matte.cpp src/materials/plastic.cpp src/materials/mix.cpp)

SET(SOURCES_TEXTURES
    src/textures/imagemap.cpp src/textures/checkerboard.cpp src/textures/dots.cpp)

SET(SOURCES_MEDIA
    src/media/grid.cpp)

SET(SOURCES_LIGHTS
    src/lights/point.cpp src/lights/spot.cpp src/lights/projection.cpp
    src/lights/distant.cpp src/lights/gonio.cpp src/lights/diffuse.cpp
    src/lights/infinite.cpp)

SET(SOURCES_EXTENSIONS
    src/ext/lodepng.cpp src/ext/targa.cpp)

SET(ALL_LIBS
    ${SOURCES_CORE} ${SOURCES_UTILITIES} ${SOURCES_SHAPES} ${SOURCES_ACCELERATORS}
    ${SOURCES_INTEGRATORS} ${SOURCES_CAMERAS} ${SOURCES_SAMPLERS} ${SOURCES_BSDFS}
    ${SOURCES_MATERIALS}  ${SOURCES_TEXTURES} ${SOURCES_MEDIA} ${SOURCES_LIGHTS}
    ${SOURCES_EXTENSIONS})

SET(GTEST_SOURCES
    src/tests/gtest/gtest_main.cc src/tests/gtest/gtest-all.cc)

SET(SOURCES_EXECUTABLE src/main.cpp ${ALL_LIBS})

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES_EXECUTABLE})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} z glog IlmImf Iex IlmThread Half)
