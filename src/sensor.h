#pragma once
#include "rkcommon/math/vec.h"
#include "rkcommon/memory/malloc.h"

struct Framebuffer {
    std::vector<float> pixels;
};

struct SensorSamples {
    rkcommon::math::vec2f *points = nullptr;
    size_t count = 0;
};

inline void allocSensorSamples(SensorSamples& s, size_t count) {
    constexpr size_t ALIGN = 64;
    s.points = static_cast<rkcommon::math::vec2f*>(
        rkcommon::memory::alignedMalloc(count * sizeof(rkcommon::math::vec2f),
                                        ALIGN));
    s.count = count;
}

inline void freeSensorSamples(SensorSamples& s) {
    rkcommon::memory::alignedFree(s.points);
    s = {};
}

void generateSensorSamples(std::uint32_t width_px,
                           std::uint32_t height_px,
                           float width,
                           float height,
                           SensorSamples& samples);

bool writePNG(const Framebuffer& fb, uint32_t W, uint32_t H, const char* path);
