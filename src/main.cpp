#include <embree4/rtcore.h>
#include <stdio.h>
#include <math.h>
#include <limits>

#include "rkcommon/math/vec.h"
#include "rkcommon/memory/malloc.h"

#include "camera.h"
#include "sensor.h"

/*
 * A minimal tutorial. 
 *
 * It demonstrates how to intersect a ray with a single triangle. It is
 * meant to get you started as quickly as possible, and does not output
 * an image. 
 *
 * For more complex examples, see the other tutorials.
 *
 * Compile this file using
 *   
 *   gcc -std=c99 \
 *       -I<PATH>/<TO>/<EMBREE>/include \
 *       -o minimal \
 *       minimal.c \
 *       -L<PATH>/<TO>/<EMBREE>/lib \
 *       -lembree4 
 *
 * You should be able to compile this using a C or C++ compiler.
 */

/* 
 * This is only required to make the tutorial compile even when
 * a custom namespace is set.
 */
#if defined(RTC_NAMESPACE_USE)
RTC_NAMESPACE_USE
#endif

/*
 * We will register this error handler with the device in initializeDevice(),
 * so that we are automatically informed on errors.
 * This is extremely helpful for finding bugs in your code, prevents you
 * from having to add explicit error checking to each Embree API call.
 */
void errorFunction(void* userPtr, enum RTCError error, const char* str)
{
  printf("error %d: %s\n", error, str);
}

/*
 * Embree has a notion of devices, which are entities that can run 
 * raytracing kernels.
 * We initialize our device here, and then register the error handler so that 
 * we don't miss any errors.
 *
 * rtcNewDevice() takes a configuration string as an argument. See the API docs
 * for more information.
 *
 * Note that RTCDevice is reference-counted.
 */
RTCDevice initializeDevice()
{
  RTCDevice device = rtcNewDevice(NULL);

  if (!device)
    printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));

  rtcSetDeviceErrorFunction(device, errorFunction, NULL);
  return device;
}

/*
 * Create a scene, which is a collection of geometry objects. Scenes are 
 * what the intersect / occluded functions work on. You can think of a 
 * scene as an acceleration structure, e.g. a bounding-volume hierarchy.
 *
 * Scenes, like devices, are reference-counted.
 */
RTCScene initializeScene(RTCDevice device) {
  RTCScene scene = rtcNewScene(device);

  /* 
   * Create a triangle mesh geometry, and initialize a single triangle.
   * You can look up geometry types in the API documentation to
   * find out which type expects which buffers.
   *
   * We create buffers directly on the device, but you can also use
   * shared buffers. For shared buffers, special care must be taken
   * to ensure proper alignment and padding. This is described in
   * more detail in the API documentation.
   */
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  float* vertices = (float*) rtcSetNewGeometryBuffer(geom,
                                                     RTC_BUFFER_TYPE_VERTEX,
                                                     0,
                                                     RTC_FORMAT_FLOAT3,
                                                     3*sizeof(float),
                                                     3);

  unsigned* indices = (unsigned*) rtcSetNewGeometryBuffer(geom,
                                                          RTC_BUFFER_TYPE_INDEX,
                                                          0,
                                                          RTC_FORMAT_UINT3,
                                                          3*sizeof(unsigned),
                                                          1);

  if (vertices && indices)
  {
    vertices[0] = -1.f; vertices[1] = 0.f; vertices[2] = 0.f;
    vertices[3] = 1.f; vertices[4] = 0.f; vertices[5] = 0.f;
    vertices[6] = 0.f; vertices[7] = 1.f; vertices[8] = 0.f;

    indices[0] = 0; indices[1] = 1; indices[2] = 2;
  }

  /*
   * You must commit geometry objects when you are done setting them up,
   * or you will not get any intersections.
   */
  rtcCommitGeometry(geom);

  /*
   * In rtcAttachGeometry(...), the scene takes ownership of the geom
   * by increasing its reference count. This means that we don't have
   * to hold on to the geom handle, and may release it. The geom object
   * will be released automatically when the scene is destroyed.
   *
   * rtcAttachGeometry() returns a geometry ID. We could use this to
   * identify intersected objects later on.
   */
  rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);

  /*
   * Like geometry objects, scenes must be committed. This lets
   * Embree know that it may start building an acceleration structure.
   */
  rtcCommitScene(scene);

  return scene;
}

void traceAndShade(RTCScene scene, RayBuffer& rb, Framebuffer& fb)
{
    RTCRayQueryContext qctx;
    rtcInitRayQueryContext(&qctx);

    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.context = &qctx;
    iargs.flags   = RTC_RAY_QUERY_FLAG_COHERENT;

    for (size_t i = 0; i < rb.count; ++i) {
        rtcIntersect1(scene, &rb.rh[i], &iargs);

        const RTCRayHit& rh = rb.rh[i];
        float* px = &fb.pixels[rb.pixel[i] * 3];

        if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            px[0] = 1;  px[1] = 0;  px[2] = 0;
        } else {
            px[0] = px[1] = px[2] = 0.f;
        }
    }
}
/* -------------------------------------------------------------------------- */
using namespace rkcommon::math;

int main() {
    /* Initialization. All of this may fail, but we will be notified by
    * our errorFunction. */
    RTCDevice device = initializeDevice();
    RTCScene scene = initializeScene(device);

    // Camera
    Camera cam;
    cam.focal_length     = 0.010f;
    cam.width            = 0.036f;
    cam.height           = 0.024;
    cam.position         = vec3f(0.f, 0.f,  1.0f);
    cam.width_px  = 600;
    cam.height_px = 400;

    const uint32_t W = cam.width_px;
    const uint32_t H = cam.height_px;
    const size_t   N = size_t(W) * H;

    // Film samples
    SensorSamples samples;            
    allocSensorSamples(samples, N);
    generateSensorSamples(cam.width_px, cam.height_px, cam.width, cam.height, samples);

    // Eye rays
    RayBuffer rb;
    allocRayBuffer(rb, N);
    generateEyeRays(cam, samples, Basis{}, rb);

    // Framebuffer
    Framebuffer fb;
    fb.pixels.resize(N * 3);

    // Trace
    traceAndShade(scene, rb, fb);

    // write EXR
    writeEXR(fb, W, H, "render.exr");
    printf("Wrote render.exr (%ux%u)\n", W, H);

    freeRayBuffer(rb);
    freeSensorSamples(samples);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return 0;
}