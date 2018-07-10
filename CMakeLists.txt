CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(src/utilities)

PROJECT(pbr-tracer)


SET(SOURCES_CORE
    src/core/renderer.cpp src/core/paramset.cpp src/core/texture.cpp
    src/core/integrator.cpp src/core/interaction.cpp src/core/spectrum.cpp
    src/core/transform.cpp src/core/parser.cpp src/core/shape.cpp
    src/core/primitive.cpp src/core/parallel.cpp src/core/stats.cpp
    src/core/camera.cpp src/core/api.cpp)

SET(SOURCES_UTILITIES
    src/utilities/random.cpp src/utilities/quaternion.cpp src/utilities/log.cpp
    src/utilities/matrix.cpp src/utilities/memory.cpp src/utilities/file.cpp)

SET(SOURCES_SHAPES
    src/shapes/sphere.cpp src/shapes/cylinder.cpp src/shapes/disk.cpp
    src/shapes/triangle.cpp src/shapes/curve.cpp)

SET(SOURCES_ACCELERATORS
    src/accelerators/bvh.cpp src/accelerators/kdtree.cpp)

SET(SOURCES_INTEGRATORS
    src/integrators/whitted.cpp)

SET(SOURCES_CAMERAS
    src/cameras/ortho.cpp src/cameras/perspective.cpp src/cameras/environment.cpp)

SET(ALL_LIBS
    ${SOURCES_CORE} ${SOURCES_UTILITIES} ${SOURCES_SHAPES} ${SOURCES_ACCELERATORS}
    ${SOURCES_INTEGRATORS} ${SOURCES_CAMERAS})

SET(GTEST_SOURCES
    src/tests/gtest/gtest_main.cc src/tests/gtest/gtest-all.cc)

SET(SOURCES_EXECUTABLE src/main.cpp ${ALL_LIBS})


FIND_LIBRARY(GLOG_LIB glog)

LINK_LIBRARIES(${GLOG_LIB})

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES_EXECUTABLE})
