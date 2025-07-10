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
};

inline void allocRayBuffer(RayBuffer& r, size_t N) {
    constexpr size_t ALIGN = 64;
    r.rh    = static_cast<RTCRayHit*>( rkcommon::memory::alignedMalloc(
                                          N * sizeof(RTCRayHit), ALIGN));
}

inline void freeRayBuffer(RayBuffer& r) {
    rkcommon::memory::alignedFree(r.rh);
    r = {};
}

inline RTCRay makeRay(const rkcommon::math::vec3f& org,
                            const rkcommon::math::vec3f& dir,
                            float tnear = 1e-3f,
                            float tfar  = std::numeric_limits<float>::infinity(),
                            unsigned mask = 0xFFFFFFFFu,
                            float time = 0.f) {
    RTCRay ray;
    ray.org_x = org.x;
    ray.org_y = org.y;
    ray.org_z = org.z;

    ray.dir_x = dir.x;
    ray.dir_y = dir.y;
    ray.dir_z = dir.z;

    ray.tnear = tnear;
    ray.tfar  = tfar;
    ray.mask  = mask;
    ray.time  = time;
    ray.id    = 0;
    ray.flags = 0;

    return ray;
}


// Move somewhere else
inline RTCRayHit makeRayHit(const rkcommon::math::vec3f& org,
                            const rkcommon::math::vec3f& dir,
                            float tnear = 1e-3f,
                            float tfar  = std::numeric_limits<float>::infinity(),
                            unsigned mask = 0xFFFFFFFFu,
                            float time = 0.f) {
    RTCRayHit rh;
    rh.ray = makeRay(org, dir, tnear, tfar, mask, time);

    rh.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
    rh.hit.primID    = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    return rh;
}

void generateRaysForPixel(const Camera& cam,
                          const Basis&  bas,
                          const SensorSamples& samp,
                          uint32_t spp,
                          RayBuffer& rb); 

// void generateEyeRays(const Camera& cam,
//                      const SensorSamples& samples,
//                      const Basis& basis,
//                      RayBuffer& rb,
//                      uint32_t spp);