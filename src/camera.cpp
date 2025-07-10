#include "camera.h"
#include "sensor.h"
#include <limits>

using rkcommon::math::vec3f;

inline RTCRayHit makeEyeRay(const Camera& cam, const Basis& basis, const rkcommon::math::vec2f sensorPoint) {
    RTCRayHit rh;
    const float fl = cam.focal_length;

    vec3f d_cam = normalize(vec3f(sensorPoint.x, sensorPoint.y, -fl));
    vec3f d = basis.x * d_cam.x +
                basis.y * d_cam.y +
                basis.z * d_cam.z;

    rh = makeRayHit(cam.position, d, 0.f);
    return rh;
}

void generateRaysForPixel(const Camera& cam,
                                 const Basis&  bas,
                                 const SensorSamples& samp,
                                 uint32_t spp,
                                 RayBuffer& rb) {
    for (uint32_t s = 0; s < spp; ++s)
        rb.rh[s] = makeEyeRay(cam, bas, samp.points[s]);
}

// void generateEyeRays(const Camera& cam,
//                      const SensorSamples& samples,
//                      const Basis& basis,
//                      RayBuffer& rb,
//                      uint32_t spp) {
//     const float fl = cam.focal_length;

//     for (size_t i = 0; i < rb.count; ++i) {
//         vec3f d_cam = normalize(vec3f(samples.points[i].x,
//                                       samples.points[i].y,
//                                      -fl));
//         vec3f d = basis.x * d_cam.x +
//                   basis.y * d_cam.y +
//                   basis.z * d_cam.z;

//         rb.rh[i] = makeRayHit(cam.position, d, 0.f);

//         uint32_t pixel_index = static_cast<uint32_t>(i) / spp; // correct pixel index
//         rb.pixel[i] = pixel_index;
//     }
// }