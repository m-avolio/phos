#include <embree4/rtcore.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include <random>

#include "rkcommon/math/vec.h"
#include "rkcommon/memory/malloc.h"

#define TINYOBJLOADER_IMPLEMENTATION 
#include "tiny_obj_loader.h"

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

// Needs a redo
static RTCScene buildSceneFromOBJ(RTCDevice device,
                                  const std::string& objPath,
                                  std::vector<uint32_t>& matID,
                                  const std::vector<std::string>& mtlSearchPaths = {})
{
    // 1. Load the OBJ file (vertices, shapes and materials)
    tinyobj::attrib_t                attrib;
    std::vector<tinyobj::shape_t>    shapes;
    std::vector<tinyobj::material_t> materials;
    std::string                      warn, err;
    bool ret = tinyobj::LoadObj(&attrib,
                                &shapes,
                                &materials,
                                &warn,
                                &err,
                                objPath.c_str(),
                                /*mtl_basepath=*/mtlSearchPaths.empty() ? nullptr : mtlSearchPaths[0].c_str(),
                                /*triangulate=*/true);

    if (!warn.empty())  std::cerr << "OBJ warning: " << warn << "\n";
    if (!err.empty())   throw std::runtime_error("OBJ error: " + err);
    if (!ret)           throw std::runtime_error("Failed to load/parse OBJ");

    // 2. Create a new Embree scene
    RTCScene scene = rtcNewScene(device);

    // 3. For each shape in the OBJ, push its triangles into Embree
    for (size_t s = 0; s < shapes.size(); ++s) {
        const auto& mesh = shapes[s].mesh;
        if (mesh.indices.empty()) continue;  // skip empty shapes

        // Allocate a new triangle geometry
        RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        // -- Vertex buffer: use the global attrib.vertices (flat float list [x,y,z,x,y,z,...])
        size_t    numVerts = attrib.vertices.size() / 3;
        float*    vBuf     = static_cast<float*>(
            rtcSetNewGeometryBuffer(geom,
                                    RTC_BUFFER_TYPE_VERTEX,
                                    /*slot=*/0,
                                    RTC_FORMAT_FLOAT3,
                                    /*byteStride=*/3 * sizeof(float),
                                    /*numVertices=*/numVerts));
        std::memcpy(vBuf,
                    attrib.vertices.data(),
                    attrib.vertices.size() * sizeof(float));

        // -- Index buffer: every 3 indices (must be triangles)
        size_t numIndices = mesh.indices.size();
        if (numIndices % 3 != 0) {
            std::cerr << "Warning: non-triangle faces found, skipping remainder.\n";
        }
        uint32_t* iBuf = static_cast<uint32_t*>(
            rtcSetNewGeometryBuffer(geom,
                                    RTC_BUFFER_TYPE_INDEX,
                                    /*slot=*/0,
                                    RTC_FORMAT_UINT3,
                                    /*byteStride=*/3 * sizeof(uint32_t),
                                    /*numTriangles=*/numIndices / 3));
        for (size_t f = 0; f + 2 < numIndices; f += 3) {
            iBuf[f + 0] = mesh.indices[f + 0].vertex_index;
            iBuf[f + 1] = mesh.indices[f + 1].vertex_index;
            iBuf[f + 2] = mesh.indices[f + 2].vertex_index;
        }

        rtcCommitGeometry(geom);
        uint32_t geomID = rtcAttachGeometry(scene, geom);
        std::cout << geomID << std::endl;

        // 4. Record material ID for this geometry
        //    (we just take the first face's material, you could split by material if needed)
        if (geomID >= matID.size()) matID.resize(geomID + 1);
        matID[geomID] = mesh.material_ids.empty()
                        ? 0u
                        : static_cast<uint32_t>(mesh.material_ids[0]);

        rtcReleaseGeometry(geom);
    }

    // 5. Commit and return
    rtcCommitScene(scene);
    return scene;
}

void traceAndShade(RTCScene scene, const std::vector<uint32_t>& matID, RayBuffer& rb, Framebuffer& fb) {
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
            // retrieve the hit distance
            float dist = rh.ray.tfar;  

            // map [0 .. maxT] → [1 .. 0], clamped, so near = white, far = black
            float shade = 1.0f - std::min(dist / 8.0f, 1.0f);

            // write a grayscale
            px[0] = shade;
            px[1] = shade;
            px[2] = shade;
        } else {
            px[0] = px[1] = px[2] = 0.f;
        }
    }
}
/* -------------------------------------------------------------------------- */
using namespace rkcommon::math;

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s scene.usd\n", argv[0]);
        return 0;
    }
    const std::string objPath = argv[1];

    std::vector<uint32_t> matID;
    /* Setup device + scene */
    RTCDevice device = initializeDevice();
    // RTCScene  scene  = buildSceneFromUSD(device, usdPath, matID);
    RTCScene  scene  = buildSceneFromOBJ(device, objPath, matID);

    // Camera
    Camera cam;
    cam.focal_length     = 0.050f;
    cam.width            = 0.036f;
    cam.height           = 0.024;
    cam.position         = vec3f(0.f, 0.0f,  4.5f);
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
    Basis bas;
    bas.x = vec3f(1, 0, 0);
    bas.y = vec3f(0, 1, 0);
    bas.z = vec3f(0, 0, 1);

    generateEyeRays(cam, samples, bas, rb);

    // Framebuffer
    Framebuffer fb;
    fb.pixels.resize(N * 3);

    // Trace
    traceAndShade(scene, matID, rb, fb);

    // write PNG
    writePNG(fb, W, H, "render.png");
    printf("Wrote render.png (%ux%u)\n", W, H);

    freeRayBuffer(rb);
    freeSensorSamples(samples);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return 0;
}