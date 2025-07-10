#include <embree4/rtcore.h>
#include <embree4/rtcore_ray.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include <random>
#include <atomic>
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
static constexpr int LIGHT_SAMPLES = 1;

inline RTCDevice device = nullptr;
inline RTCScene scene = nullptr;

// DEBUG Purposes, remove later or find a better way
#define DEBUG
#ifdef DEBUG
inline std::atomic<uint64_t> rayTraversals{0};
inline void rtcIntersect1Count(
        RTCScene                 scene,
        RTCRayHit*               rayhit,
        RTCIntersectArguments*   args)
{
    rayTraversals.fetch_add(1, std::memory_order_relaxed);
    rtcIntersect1(scene, rayhit, args);
}
inline void rtcOccluded1Count(
        RTCScene                 scene,
        RTCRay*                  ray,
        RTCOccludedArguments*    args)
{
    rayTraversals.fetch_add(1, std::memory_order_relaxed);
    rtcOccluded1(scene, ray, args);
}

#define rtcIntersect1  rtcIntersect1Count
#define rtcOccluded1   rtcOccluded1Count
#endif

using namespace rkcommon::math;

struct Material {
    std::vector<vec3f> albedo;
    std::vector<std::complex<float>> ior;
    std::vector<bool> specular;
    std::vector<bool> emissive;
};

struct EmissiveTri {
    uint32_t geomID;
    uint32_t primID;
    const uint32_t* index;
    const float* vertex;
    float area;
    float power;
    vec3f Le;
};

// Globals
inline Material mats;
inline std::vector<EmissiveTri> emissives;
std::mt19937 rng{ 42 };

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

void buildSceneFromOBJ(const std::string& objPath) {
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

    for (auto m: materials) {
        std::cout << m.name << std::endl;
    }

    // Load Materials
    size_t N = materials.empty() ? 1 : materials.size();
    mats.albedo.resize(N);
    mats.ior.resize(N);
    mats.emissive.resize(N);
    mats.specular.resize(N);

    scene = rtcNewScene(device);

    for (size_t s = 0; s < shapes.size(); ++s) {
        const auto& mesh = shapes[s].mesh;
        if (mesh.indices.empty()) continue;

        RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        // Indices
        size_t numIndices = mesh.indices.size();
        if (numIndices % 3 != 0) {
            std::cerr << "Warning: non-triangle faces found, skipping remainder.\n"; // What is the difference between cerror and runtime_error

        }
        uint32_t* iBuf = static_cast<uint32_t*>(rtcSetNewGeometryBuffer(
                                    geom,
                                    RTC_BUFFER_TYPE_INDEX,
                                    0,
                                    RTC_FORMAT_UINT3,
                                    3 * sizeof(uint32_t),
                                    numIndices / 3));
        for (size_t f = 0; f + 2 < numIndices; f += 3) {
            iBuf[f + 0] = mesh.indices[f + 0].vertex_index;
            iBuf[f + 1] = mesh.indices[f + 1].vertex_index;
            iBuf[f + 2] = mesh.indices[f + 2].vertex_index;
        }

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
                    attrib.vertices.size()*sizeof(float));


        // Normals
        rtcSetGeometryVertexAttributeCount(geom, 1);
        if (!attrib.normals.empty()) {
            float* nBuf = static_cast<float*>(rtcSetNewGeometryBuffer(
                                    geom,
                                    RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,
                                    0, 
                                    RTC_FORMAT_FLOAT3,
                                    3 * sizeof(float),
                                    numVerts));

                for (size_t f = 0; f < mesh.indices.size(); f += 3) {
                    for (int k = 0; k < 3; ++k) {
                        const auto& idx = mesh.indices[f + k];

                        int vi = idx.vertex_index;
                        int ni = idx.normal_index;
                        if (ni < 0) {
                            printf("here"); 
                            continue;
                        }

                        nBuf[3*vi + 0] = attrib.normals[3*ni + 0];
                        nBuf[3*vi + 1] = attrib.normals[3*ni + 1];
                        nBuf[3*vi + 2] = attrib.normals[3*ni + 2];
                    }
                }
        } else {
            static const vec3f zero = vec3f{0};
            rtcSetSharedGeometryBuffer(
                geom,
                RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0,
                RTC_FORMAT_FLOAT3,
                zero,
                0,
                0,
                1);
        }

        rtcCommitGeometry(geom);
        uint32_t geomID = rtcAttachGeometry(scene, geom);

        // Assuming one material ID per geom
        if (!materials.empty()) {
            auto material = materials[mesh.material_ids[0]];
            mats.albedo[geomID] = vec3f(material.diffuse);
            mats.ior[geomID]    = std::complex(material.ior, 0.0f);  // All dielectric for now
            mats.specular[geomID] = (material.name == "metal") ? true : false;
            mats.emissive[geomID] = false; // This might not be a good idea
            bool isLight = (material.emission[0] > 0.0f || material.emission[1] > 0.0f || material.emission[2] > 0.0f); // could cause crash
            if (isLight) {
                printf("islight");
                mats.emissive[geomID] = true;
                const size_t numTris = numIndices / 3;
                vec3f Le = vec3f(material.emission); 
                for (uint32_t t = 0; t < numTris; ++t) {

                    // Calculate Area
                    uint32_t i0 = iBuf[3*t + 0];
                    uint32_t i1 = iBuf[3*t + 1];
                    uint32_t i2 = iBuf[3*t + 2];

                    vec3f p0{vBuf[3*i0 + 0], vBuf[3*i0 + 1], vBuf[3*i0 + 2]};
                    vec3f p1{vBuf[3*i1 + 0], vBuf[3*i1 + 1], vBuf[3*i1 + 2]};
                    vec3f p2{vBuf[3*i2 + 0], vBuf[3*i2 + 1], vBuf[3*i2 + 2]};

                    float area = 0.5f * length(cross(p1 - p0, p2 - p0));
                    emissives.push_back(EmissiveTri{geomID, t, iBuf, vBuf, area, area * dot(Le, vec3f(0.3333f)), Le/(float(pi)*area)}); //TODO: Should I divide by pi*area?
                }
            }
        } else { // default material
            mats.albedo[geomID] = vec3f(0.9f, 0.0f, 0.95f);
            mats.ior[geomID]    = std::complex(1.0f, 0.0f);
            mats.emissive[geomID] = false;
            mats.specular[geomID] = false;
        }
        rtcReleaseGeometry(geom);
    }

    rtcCommitScene(scene);
}

vec3f reflect(vec3f wi, vec3f n) {
    return -2 * (dot(wi, n)) * n + wi;
}

// PBRTv3
vec2f UniformSampleTriangle(const vec2f &u) {
    float su0 = std::sqrt(u[0]);
    return vec2f(1 - su0, u[1] * su0);
}

inline void getTriangleVerts(const EmissiveTri& tri,
                             vec3f& p0, vec3f& p1, vec3f& p2)
{
    const uint32_t* i = tri.index;
    const float*    v = tri.vertex;

    uint32_t i0 = i[3*tri.primID + 0];
    uint32_t i1 = i[3*tri.primID + 1];
    uint32_t i2 = i[3*tri.primID + 2];

    p0 = vec3f{ v[3*i0 + 0], v[3*i0 + 1], v[3*i0 + 2] };
    p1 = vec3f{ v[3*i1 + 0], v[3*i1 + 1], v[3*i1 + 2] };
    p2 = vec3f{ v[3*i2 + 0], v[3*i2 + 1], v[3*i2 + 2] };
}

struct LightSample {
    vec3f point;
    vec3f Ng;
    float pdf;
};

LightSample EmissiveTriSample(const EmissiveTri &tri, const vec2f &u) {
    // get triangle points
    vec3f p0, p1, p2;
    getTriangleVerts(tri, p0, p1, p2);
    // sample triangle
    vec2f b   = UniformSampleTriangle(u);
    vec3f point = b[0]*p0 + b[1]*p1 + (1.f-b[0]-b[1])*p2;
    vec3f Ng = normalize(cross(p1 - p0, p2 - p0));
    float pdf = 1.f / tri.area;  // should double check to make sure this is correct
    return LightSample{point, Ng, pdf}; 
}

inline vec2f randomVec2f() {
    static std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    return vec2f{uni(rng), uni(rng)};
}

vec3f traceRay(RTCRayQueryContext &qctx,
               RTCIntersectArguments &iargs,
               RTCRayHit rayhit,
               int depth) {

    if (depth >= MAX_DEPTH)  
        return vec3f{0.0f,0.0f,0.0f};

    // reset hit and tfar
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.ray.tfar   = std::numeric_limits<float>::infinity();

    // intersect
    rtcIntersect1(scene, &rayhit, &iargs);

    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
        // missed
        return vec3f{0.0f,0.0f,0.0f};
    }

    uint32_t geomID = rayhit.hit.geomID;

    // Geometric Normal
    vec3f Ng = normalize(
      vec3f{ rayhit.hit.Ng_x,
             rayhit.hit.Ng_y,
             rayhit.hit.Ng_z }
    );

    // Shading Normal
    vec3f Ns = Ng;
    vec3f NsBuf = vec3f{0.0f};
    RTCGeometry geometry = rtcGetGeometry(scene, geomID);
    rtcInterpolate0(geometry, rayhit.hit.primID, rayhit.hit.u, rayhit.hit.v, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, NsBuf, 3);

    if (NsBuf[0] || NsBuf[1] || NsBuf[2]) { Ns = normalize(vec3f{NsBuf[0], NsBuf[1], NsBuf[2]}); }

    if (dot(Ns, Ng) < 0.0f) { Ns = -Ns; }

    vec3f wi = normalize(vec3f{
        rayhit.ray.dir_x,
        rayhit.ray.dir_y,
        rayhit.ray.dir_z
    });

    // Move position in direction of geom normal 
    vec3f org = vec3f{
        rayhit.ray.org_x + wi.x * rayhit.ray.tfar + Ng.x * EPSILON,
        rayhit.ray.org_y + wi.y * rayhit.ray.tfar + Ng.y * EPSILON,
        rayhit.ray.org_z + wi.z * rayhit.ray.tfar + Ng.z * EPSILON
    };

    if (mats.specular[geomID]) {
        vec3f wr = reflect(wi, Ns);
        // build new RTCRayHit
        RTCRayHit newRH = makeRayHit(org, wr, 0);
        return mats.albedo[geomID] * traceRay(qctx, iargs, newRH, depth + 1);
    } else if (mats.emissive[geomID]) {
        return mats.albedo[geomID];
    } else {
        // sample every light
        vec3f Li = vec3f{0};
        for (auto light: emissives) {
            for (int i = 0; i < LIGHT_SAMPLES; ++i) {
                LightSample sample = EmissiveTriSample(light, randomVec2f());
                vec3f wi = sample.point - org;
                float dist = length(wi);
                RTCRay shadowRay = makeRay(org, normalize(wi), 0, dist-EPSILON);  //do not intersect with light
                rtcOccluded1(scene, &shadowRay, NULL);
                if (shadowRay.tfar < EPSILON) {continue;} // shadowed
                float cosSurface = std::max(0.f, dot(Ns,  wi));
                float cosLight   = std::max(0.f, dot(sample.Ng, -wi)); // TODO: look into the normal for the light, not sure if this is good
                if (cosSurface == 0.f || cosLight == 0.f) {continue;}     
                float G = (cosSurface * cosLight) / (dist*dist);
                vec3f contrib = light.Le * G / sample.pdf;
                Li += contrib / LIGHT_SAMPLES;
            }
        }
        return mats.albedo[geomID] * Li * float(one_over_pi);
    }
}

void traceAndShade(RayBuffer& rb, Framebuffer& fb, uint32_t spp) {
    RTCRayQueryContext qctx;
    rtcInitRayQueryContext(&qctx);

    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.context = &qctx;
    iargs.flags   = RTC_RAY_QUERY_FLAG_COHERENT;

    for (size_t i = 0; i < rb.count; ++i) {
        // call our recursive helper with depth=0
        vec3f col = traceRay(qctx, iargs, rb.rh[i], 0);


        float inv_spp = 1.0f/spp; 
        float* px = &fb.pixels[rb.pixel[i] * 3];
        px[0] += col.x * inv_spp;
        px[1] += col.y * inv_spp;
        px[2] += col.z * inv_spp;
    }
}

/* -------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s scene.obj\n", argv[0]);
        return 0;
    }
    const std::string objPath = argv[1];

    /* Setup device + scene */
    device = initializeDevice();
    buildSceneFromOBJ(objPath);

    // Camera
    Camera cam;
    cam.focal_length     = 0.050f;
    cam.width            = 0.036f;
    cam.height           = 0.024;
    cam.position         = vec3f(0.f, 0.0f,  2.5f);
    cam.width_px  = 600;
    cam.height_px = 400;

    const uint32_t SPP = 128;
    const uint32_t W = cam.width_px;
    const uint32_t H = cam.height_px;
    const size_t   N = size_t(W) * H * SPP;

    // Film samples
    SensorSamples samples;            
    allocSensorSamples(samples, N);
    generateSensorSamples(cam.width_px, cam.height_px, cam.width, cam.height, SPP, samples);

    // Eye rays
    RayBuffer rb;
    allocRayBuffer(rb, N);
    Basis bas;
    bas.x = vec3f(1, 0, 0);
    bas.y = vec3f(0, 1, 0);
    bas.z = vec3f(0, 0, 1);

    generateEyeRays(cam, samples, bas, rb, SPP);

    // Framebuffer
    Framebuffer fb;
    fb.pixels.resize(N * 3);

    #ifdef DEBUG
    auto t0 = std::chrono::high_resolution_clock::now();
    traceAndShade(rb, fb, SPP);
    auto t1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> dt = t1 - t0;
    double seconds = dt.count();
    double mrays_per_s = (double)rayTraversals / seconds / 1e6;

    std::printf("Rendered %zu rays in %.3f s → %.2f Mrays/s\n",
                N, seconds, mrays_per_s);
    #elif
    traceAndShade(rb, fb, SPP);
    #endif

    // write PNG
    writePNG(fb, W, H, "render.png");
    printf("Wrote render.png (%ux%u)\n", W, H);

    freeRayBuffer(rb);
    freeSensorSamples(samples);
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return 0;
}