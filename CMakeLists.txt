CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(src/utilities)

PROJECT(pbr-tracer)

SET(SOURCES_CORE src/core/renderer.cpp src/core/paramset.cpp src/core/texture.cpp
    src/core/integrator.cpp)
SET(SOURCES_UTILITIES src/utilities/random.cpp src/utilities/math.cpp src/utilities/report.cpp
    src/utilities/matrix.cpp src/utilities/memory.cpp src/utilities/parser.cpp
    src/utilities/parallel.cpp src/utilities/stats.cpp src/utilities/file.cpp
    src/utilities/spectrum.cpp )
SET(SOURCES src/main.cpp ${SOURCES_CORE} ${SOURCES_UTILITIES})

LINK_LIBRARIES(glog)

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})
