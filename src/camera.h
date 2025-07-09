#pragma once
#include <embree4/rtcore.h>
#include "rkcommon/math/vec.h"
#include "rkcommon/memory/malloc.h"
#include "sensor.h"

struct Basis {
    rkcommon::math::vec3f x{1,0,0}, y{0,1,0}, z{0,0,1};
};

struct Camera {
    float focal_length  = 0.050f;
    float width         = 0.036f;
    float height        = 0.024f;
    rkcommon::math::vec3f position{0.f,0.f,1.5f};
    uint32_t width_px  = 600;
    uint32_t height_px = 400;
};

// Move eventually if I make a ray file
struct RayBuffer {
    RTCRayHit* rh {};
    uint32_t*  pixel {};
    size_t     count {};
};

inline void allocRayBuffer(RayBuffer& r, size_t N) {
    constexpr size_t ALIGN = 64;
    r.rh    = static_cast<RTCRayHit*>( rkcommon::memory::alignedMalloc(
                                          N * sizeof(RTCRayHit), ALIGN));
    r.pixel = static_cast<uint32_t*>(  rkcommon::memory::alignedMalloc(
                                          N * sizeof(uint32_t), ALIGN));
    r.count = N;
}

inline void freeRayBuffer(RayBuffer& r) {
    rkcommon::memory::alignedFree(r.rh);
    rkcommon::memory::alignedFree(r.pixel);
    r = {};
}

inline RTCRayHit makeRayHit(const rkcommon::math::vec3f& org,
                            const rkcommon::math::vec3f& dir,
                            float tnear = 1e-3f,
                            float tfar  = std::numeric_limits<float>::infinity(),
                            unsigned mask = 0xFFFFFFFFu,
                            float time = 0.f) {
    RTCRayHit rh;
    rh.ray.org_x = org.x;
    rh.ray.org_y = org.y;
    rh.ray.org_z = org.z;

    rh.ray.dir_x = dir.x;
    rh.ray.dir_y = dir.y;
    rh.ray.dir_z = dir.z;

    rh.ray.tnear = tnear;
    rh.ray.tfar  = tfar;
    rh.ray.mask  = mask;
    rh.ray.time  = time;
    rh.ray.id    = 0;
    rh.ray.flags = 0;

    rh.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
    rh.hit.primID    = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    return rh;
}

void generateEyeRays(const Camera& cam,
                     const SensorSamples& samples,
                     const Basis& basis,
                     RayBuffer& rs);
