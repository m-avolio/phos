cmake_minimum_required(VERSION 3.16)
project(phos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Dependencies ───────────────────────────────────────────────
find_package(embree 4 REQUIRED)

set(BUILD_TESTING OFF CACHE BOOL "Disable all rkcommon CTest targets" FORCE)  # Do not build rkcommon tests
add_subdirectory(external/rkcommon)

add_library(stb INTERFACE)
target_include_directories(stb INTERFACE
    ${CMAKE_SOURCE_DIR}/external/stb
)

add_library(tinyobj INTERFACE)
target_include_directories(tinyobj INTERFACE 
    ${CMAKE_SOURCE_DIR}/external/tinyobj
)

add_library(core STATIC
    src/camera.cpp
    src/sensor.cpp
    src/samplers.cpp
)

target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(core
    PUBLIC
        embree
        rkcommon
        stb
        tinyobj
)

add_executable(phos src/main.cpp)
target_link_libraries(phos PRIVATE core)