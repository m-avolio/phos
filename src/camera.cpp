#include "camera.h"
#include "sensor.h"
#include <limits>

using rkcommon::math::vec3f;

void generateEyeRays(const Camera& cam,
                     const SensorSamples& samples,
                     const Basis& basis,
                     RayBuffer& rs)
{
    const float fl = cam.focal_length;

    for (size_t i = 0; i < rs.count; ++i)
    {
        vec3f d_cam = normalize(vec3f(samples.points[i].x,
                                      samples.points[i].y,
                                     -fl));
        vec3f d = basis.x * d_cam.x +
                  basis.y * d_cam.y +
                  basis.z * d_cam.z;

        rs.rh[i] = makeRayHit(cam.position, d, 0.f);
        rs.pixel[i] = static_cast<uint32_t>(i);
    }
}