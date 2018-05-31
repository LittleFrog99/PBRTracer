CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(src/utilities)

PROJECT(pbr-tracer)

SET(SOURCES_CORE src/main.cpp)
SET(SOURCES_UTILITIES src/utilities/random.cpp src/utilities/math.cpp
    src/utilities/error.cpp src/utilities/matrix.cpp)
SET(SOURCES ${SOURCES_CORE} ${SOURCES_UTILITIES})

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})
