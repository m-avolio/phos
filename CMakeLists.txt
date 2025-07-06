cmake_minimum_required(VERSION 3.16)
project(phos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Dependencies ───────────────────────────────────────────────
find_package(embree 4 REQUIRED)
find_package(ZLIB REQUIRED)

set(BUILD_TESTING OFF CACHE BOOL "Disable all rkcommon CTest targets" FORCE)  # Do not build rkcommon tests
add_subdirectory(external/rkcommon)

add_library(tinyexr INTERFACE)
target_include_directories(tinyexr INTERFACE external/tinyexr)
# Disable miniz
target_compile_definitions(tinyexr INTERFACE
    TINYEXR_USE_MINIZ=0)

# ── Executable ─────────────────────────────────────────────────
add_library(core STATIC
    src/camera.cpp
    src/sensor.cpp
)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(core
    PUBLIC
        embree
        rkcommon
        tinyexr
        ZLIB::ZLIB
)

add_executable(phos src/main.cpp)
target_link_libraries(phos PRIVATE core)