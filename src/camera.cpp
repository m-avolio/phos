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

        RTCRayHit &r = rs.rh[i];
        r.ray.org_x = cam.position.x;
        r.ray.org_y = cam.position.y;
        r.ray.org_z = cam.position.z;
        r.ray.dir_x = d.x;
        r.ray.dir_y = d.y;
        r.ray.dir_z = d.z;
        r.ray.tnear = 0.f;
        r.ray.tfar  = std::numeric_limits<float>::infinity();
        r.ray.time  = 0.f;
        r.ray.mask  = 0xFFFFFFFF;
        r.ray.id    = 0;
        r.ray.flags = 0;
        r.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
        r.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        rs.pixel[i] = static_cast<uint32_t>(i);
    }
}