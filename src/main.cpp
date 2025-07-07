#include <embree4/rtcore.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include <random>
#include <filesystem>
#include <chrono>
#include <complex>

#include "rkcommon/math/vec.h"
#include "rkcommon/memory/malloc.h"

#define TINYOBJLOADER_IMPLEMENTATION 
#include "tiny_obj_loader.h"

#include "camera.h"
#include "sensor.h"

static constexpr float EPSILON = 1e-5;
static constexpr int MAX_DEPTH = 5;

using namespace rkcommon::math;

void errorFunction(void* userPtr, enum RTCError error, const char* str) {
  printf("error %d: %s\n", error, str);
}

RTCDevice initializeDevice() {
  RTCDevice device = rtcNewDevice(NULL);

  if (!device)
    printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));

  rtcSetDeviceErrorFunction(device, errorFunction, NULL);
  return device;
}

struct Material {
    std::vector<vec3f> albedo;
    std::vector<std::complex<float>> ior;
    std::vector<bool> specular;
    std::vector<bool> emissive;
};

struct EmissiveTri {
    uint32_t geomID;
    uint32_t primID;
    float area;
    float power;
    vec3f Le;
};

static RTCScene buildSceneFromOBJ(RTCDevice device,
                                  const std::string& objPath,
                                  Material& mats,
                                  std::vector<EmissiveTri>& emissives) {
    // Get base dir path
    std::filesystem::path baseDir = std::filesystem::path{objPath}.parent_path();

    // Load obj
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
                                baseDir.c_str(), 
                                true);
                               

    // Warnings & Errors
    if (!warn.empty())  std::cerr << "OBJ warning: " << warn << "\n";
    if (!err.empty())   throw std::runtime_error("OBJ error: " + err);
    if (!ret)           throw std::runtime_error("Failed to load/parse OBJ");

    // for (auto m: materials) {
    //     std::cout << m.name << std::endl;
    // }

    // Load Materials
    size_t N = materials.size();
    mats.albedo.resize(N);
    mats.ior.resize(N);
    mats.emissive.resize(N);
    mats.specular.resize(N);
    RTCScene scene = rtcNewScene(device);

    for (size_t s = 0; s < shapes.size(); ++s) {
        const auto& mesh = shapes[s].mesh;
        if (mesh.indices.empty()) continue;
        RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        // Vertices
        size_t numVerts = attrib.vertices.size() / 3;
        float* vBuf = static_cast<float*>(rtcSetNewGeometryBuffer(
                                    geom,
                                    RTC_BUFFER_TYPE_VERTEX,
                                    0,
                                    RTC_FORMAT_FLOAT3,
                                    3 * sizeof(float),
                                    numVerts));
        std::memcpy(vBuf,
                    attrib.vertices.data(),
                    attrib.vertices.size() * sizeof(float));

        rtcSetGeometryVertexAttributeCount(geom, 1);
        // Normals
        float* nBuf = static_cast<float*>(rtcSetNewGeometryBuffer(geom,
            RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,
            0, 
            RTC_FORMAT_FLOAT3,
            3*sizeof(float),
            numVerts));

        std::memcpy(nBuf,
                    attrib.normals.data(),
                    attrib.normals.size()*sizeof(float));

        // Indices
        size_t numIndices = mesh.indices.size();
        if (numIndices % 3 != 0) {
            std::cerr << "Warning: non-triangle faces found, skipping remainder.\n";
        }
        uint32_t* iBuf = static_cast<uint32_t*>(rtcSetNewGeometryBuffer(
                                    geom,
                                    RTC_BUFFER_TYPE_INDEX,
                                    0,
                                    RTC_FORMAT_UINT3,
                                    3 * sizeof(uint32_t),
                                    numIndices / 3));
        // This seems weird, should just be some kind of memcpy?
        for (size_t f = 0; f + 2 < numIndices; f += 3) {
            iBuf[f + 0] = mesh.indices[f + 0].vertex_index;
            iBuf[f + 1] = mesh.indices[f + 1].vertex_index;
            iBuf[f + 2] = mesh.indices[f + 2].vertex_index;
        }


        rtcCommitGeometry(geom);
        uint32_t geomID = rtcAttachGeometry(scene, geom);

        // Assuming one material ID per geom
        auto material = materials[mesh.material_ids[0]];
        mats.albedo[geomID] = vec3f(material.diffuse);
        mats.ior[geomID]    = std::complex(material.ior, 0.0f);  // All dielectric for now
        mats.emissive[geomID] = (material.name == "light") ? true : false;
        mats.specular[geomID] = (material.name == "metal") ? true : false;

        rtcReleaseGeometry(geom);
    }

    rtcCommitScene(scene);
    return scene;
}
vec3f reflect(vec3f wi, vec3f n) {
    return -2 * (dot(wi, n)) * n + wi;
}

// Recursively trace one ray and return its color.
vec3f traceRay(RTCScene scene,
               const Material &mats,
               RTCRayQueryContext &qctx,
               RTCIntersectArguments &iargs,
               RTCRayHit rayhit,
               int depth)
{
    if (depth >= MAX_DEPTH)  
        return vec3f{0.0f,0.0f,0.0f};

    // reset hit and tfar
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.ray.tfar   = std::numeric_limits<float>::infinity();

    // intersect
    rtcIntersect1(scene, &rayhit, &iargs);

    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
        // missed—return black or environment lookup
        return vec3f{0.0f,0.0f,0.0f};
    }

    uint32_t geomID = rayhit.hit.geomID;

    // Compute shading normal (normalized)
    vec3f Ng = normalize(
      vec3f{ rayhit.hit.Ng_x,
             rayhit.hit.Ng_y,
             rayhit.hit.Ng_z }
    );

    if (mats.specular[geomID]) {
        // build incident & reflection
        vec3f I = normalize(vec3f{
            rayhit.ray.dir_x,
            rayhit.ray.dir_y,
            rayhit.ray.dir_z
        });
        vec3f R = reflect(I, Ng);

        // spawn origin just off the surface
        vec3f org = vec3f{
            rayhit.ray.org_x + I.x * rayhit.ray.tfar + Ng.x * EPSILON,
            rayhit.ray.org_y + I.y * rayhit.ray.tfar + Ng.y * EPSILON,
            rayhit.ray.org_z + I.z * rayhit.ray.tfar + Ng.z * EPSILON
        };

        // build new RTCRayHit
        RTCRayHit newRH = {};
        newRH.ray.org_x = org.x;
        newRH.ray.org_y = org.y;
        newRH.ray.org_z = org.z;
        newRH.ray.dir_x = R.x;
        newRH.ray.dir_y = R.y;
        newRH.ray.dir_z = R.z;
        newRH.ray.tnear = 0.0f;
        newRH.ray.tfar  = std::numeric_limits<float>::infinity();
        newRH.ray.time  = rayhit.ray.time;
        newRH.ray.mask  = rayhit.ray.mask;
        // recurse one level deeper
        return mats.albedo[geomID] * traceRay(scene, mats, qctx, iargs, newRH, depth + 1);

    } else {
        // diffuse: just return albedo
        return mats.albedo[geomID];
    }
}
void traceAndShade(RTCScene scene,
                   const Material& mats,
                   RayBuffer& rb,
                   Framebuffer& fb)
{
    RTCRayQueryContext qctx;
    rtcInitRayQueryContext(&qctx);

    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.context = &qctx;
    iargs.flags   = RTC_RAY_QUERY_FLAG_COHERENT;

    for (size_t i = 0; i < rb.count; ++i) {
        // call our recursive helper with depth=0
        vec3f col = traceRay(scene, mats, qctx, iargs, rb.rh[i], 0);

        float* px = &fb.pixels[rb.pixel[i] * 3];
        px[0] = col.x;
        px[1] = col.y;
        px[2] = col.z;
    }
}

/* -------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s scene.usd\n", argv[0]);
        return 0;
    }
    const std::string objPath = argv[1];

    Material mats; 
    std::vector<EmissiveTri> emissives;
    /* Setup device + scene */
    RTCDevice device = initializeDevice();
    RTCScene  scene  = buildSceneFromOBJ(device, objPath, mats, emissives);

    // Camera
    Camera cam;
    cam.focal_length     = 0.050f;
    cam.width            = 0.036f;
    cam.height           = 0.024;
    cam.position         = vec3f(0.f, 0.0f,  2.5f);
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

    // --- TIMING START ---
    auto t0 = std::chrono::high_resolution_clock::now();
    traceAndShade(scene, mats, rb, fb);
    auto t1 = std::chrono::high_resolution_clock::now();
    // --- TIMING END ---

    std::chrono::duration<double> dt = t1 - t0;
    double seconds = dt.count();
    double mrays_per_s = (double)N / seconds / 1e6;

    std::printf("Rendered %zu rays in %.3f s → %.2f Mrays/s\n",
                N, seconds, mrays_per_s);

    // write PNG
    writePNG(fb, W, H, "render.png");
    printf("Wrote render.png (%ux%u)\n", W, H);

    freeRayBuffer(rb);
    freeSensorSamples(samples);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return 0;
}