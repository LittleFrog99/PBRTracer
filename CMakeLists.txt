CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(src/utilities)

PROJECT(pbr-tracer)

SET(SOURCES_CORE
    src/core/renderer.cpp src/core/paramset.cpp src/core/texture.cpp
    src/core/integrator.cpp src/core/interaction.cpp src/core/spectrum.cpp
    src/core/transform.cpp src/core/parser.cpp src/core/shape.cpp)
SET(SOURCES_UTILITIES
    src/utilities/random.cpp src/utilities/math.cpp src/utilities/report.cpp
    src/utilities/matrix.cpp src/utilities/memory.cpp src/utilities/file.cpp
    src/utilities/parallel.cpp src/utilities/stats.cpp )
SET(SOURCES_SHAPES
    src/shapes/sphere.cpp src/shapes/cylinder.cpp src/shapes/disk.cpp
    src/shapes/triangle.cpp src/shapes/curve.cpp)
SET(SOURCE_ACCELERATORS)
SET(SOURCES src/main.cpp ${SOURCES_CORE} ${SOURCES_UTILITIES} ${SOURCES_SHAPES}
    ${SOURCES_ACCELERATORS})

FIND_LIBRARY(GLOG_LIB glog)

LINK_LIBRARIES(${GLOG_LIB})

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})
