#include "camera.h"
#include "sensor.h"
#include <limits>

using rkcommon::math::vec3f;


void generateEyeRays(const Camera& cam,
                     const SensorSamples& samples,
                     const Basis& basis,
                     RayBuffer& rb,
                     uint32_t spp) {
    const float fl = cam.focal_length;

    for (size_t i = 0; i < rb.count; ++i) {
        vec3f d_cam = normalize(vec3f(samples.points[i].x,
                                      samples.points[i].y,
                                     -fl));
        vec3f d = basis.x * d_cam.x +
                  basis.y * d_cam.y +
                  basis.z * d_cam.z;

        rb.rh[i] = makeRayHit(cam.position, d, 0.f);

        uint32_t pixel_index = static_cast<uint32_t>(i) / spp; // correct pixel index
        rb.pixel[i] = pixel_index;
    }
}