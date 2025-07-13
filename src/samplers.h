#pragma once
#include <stdint.h>
#include "rkcommon/math/vec.h"
#include "rkcommon/memory/malloc.h"

struct CMJBuffer {
    rkcommon::math::vec2f* buf = nullptr;
    uint32_t depth = 0;
    uint32_t spp = 0;
    uint32_t count = 0;
    uint32_t size = 0;
};

inline void allocCMJBuffer(CMJBuffer& c,
                           uint32_t depth,
                           uint32_t spp,
                           uint32_t count) {
    constexpr size_t ALIGN = 64;
    c.depth = depth;
    c.spp = spp;
    c.count = count;
    c.size = depth * spp * count;
    c.buf = static_cast<rkcommon::math::vec2f*>(
                        rkcommon::memory::alignedMalloc(
                        depth * spp * count * sizeof(rkcommon::math::vec2f),
                        ALIGN));
}

inline void freeCMJBuffer(CMJBuffer& c) {
    rkcommon::memory::alignedFree(c.buf);
    c = {};
}

inline rkcommon::math::vec2f& CMJSample(
                                CMJBuffer& c, uint32_t depth, 
                                uint32_t s, uint32_t k) {
    return c.buf[(depth * c.spp + s) * c.count + k];
}


// Robert Jenkins' 32 bit integer hash function
inline uint32_t hash( uint32_t a) {
   a = (a+0x7ed55d16) + (a<<12);
   a = (a^0xc761c23c) ^ (a>>19);
   a = (a+0x165667b1) + (a<<5);
   a = (a+0xd3a2646c) ^ (a<<9);
   a = (a+0xfd7046c5) + (a<<3);
   a = (a^0xb55a4f09) ^ (a>>16);
   return a;
}

// Andrew Kensler 2013
rkcommon::math::vec2f cmj(int s, int N, int p, float a = 1.0f);

inline void fillCMJBuffer(CMJBuffer &c, const uint32_t pixelSeed) {
    float size = c.depth * c.spp * c.count;
    for (size_t i = 0; i < size; ++i) { //possible overflow? run out of memory?
        rkcommon::math::vec2f u = cmj(i, size, pixelSeed);
        c.buf[i] = u;
    }
}
