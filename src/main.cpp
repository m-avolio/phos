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
#include <thread>

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
inline thread_local uint64_t tlsRays = 0;

inline void rtcIntersect1Count(RTCScene s, RTCRayHit* rh, RTCIntersectArguments* a)
{
    ++tlsRays;
    rtcIntersect1(s, rh, a);
}
inline void rtcOccluded1Count(RTCScene s, RTCRay* r, RTCOccludedArguments* a)
{
    ++tlsRays;
    rtcOccluded1(s, r, a);
}

#define rtcIntersect1 rtcIntersect1Count
#define rtcOccluded1  rtcOccluded1Count
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
                    emissives.push_back(EmissiveTri{geomID, t, iBuf, vBuf, area, area * dot(Le, vec3f(0.3333f)), Le}); //TODO: Should I divide by pi*area?
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
    vec3f wi;
    float dist;
    float pdf;
};

LightSample EmissiveTriSample(const EmissiveTri &tri, const vec3f &origin, const vec2f &u) {
    // get triangle points
    vec3f p0, p1, p2;
    getTriangleVerts(tri, p0, p1, p2);
    // sample triangle
    vec2f b   = UniformSampleTriangle(u);
    vec3f point = b[0]*p0 + b[1]*p1 + (1.f-b[0]-b[1])*p2;
    vec3f Ng = normalize(cross(p1 - p0, p2 - p0));
    vec3f wi = point - origin;
    float dist = length(wi);
    wi = normalize(wi);
    float cos = std::max(0.f, dot(Ng,  -wi));
    if (cos == 0) return LightSample{point, Ng, wi, dist, 0};
    float pdf = (dist*dist) / (tri.area * cos);
    return LightSample{point, Ng, wi, dist, pdf}; 
}

inline vec2f randomVec2f() {
    static std::uniform_real_distribution<float> uni(0.0f, 1.0f);
    return vec2f{uni(rng), uni(rng)};
}


// Revall Frisvad 2012
inline void frisvad(const vec3f& n, vec3f& b1, vec3f& b2) {
    if(n.z < -0.9999999f) { // Handle the singularity
        b1 = vec3f( 0.0f, -1.0f, 0.0f); b2 = vec3f(-1.0f, 0.0f, 0.0f); return;
    }
    const float a = 1.0f/(1.0f + n.z); const float b = -n.x*n.y*a;
    b1 = vec3f(1.0f - n.x*n.x*a, b, -n.x); b2 = vec3f(b, 1.0f - n.y*n.y*a, -n.y);
}

// PBRT 
vec2f concentricSampleDisk(const vec2f &u) {
    vec2f uOffset = 2.f * u - vec2f(1, 1);
    if (uOffset.x == 0 && uOffset.y == 0) { return vec2f(0, 0); }
    float theta, r;
    if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
    r = uOffset.x;
    theta = float(quarter_pi) * (uOffset.y / uOffset.x);
    } else {
        r = uOffset.y;
        theta = float(half_pi) - float(quarter_pi) * (uOffset.x / uOffset.y);
    }
    return r * vec2f(std::cos(theta), std::sin(theta));
}

// PBRT 
inline vec3f cosineSampleHemisphere(const vec2f &u) {
    vec2f d = concentricSampleDisk(u);
    float z = std::sqrt(std::max((float)0, 1 - d.x * d.x - d.y * d.y));
    return vec3f(d.x, d.y, z);
}

//PBRT
inline float powerHeuristic(int nf, float fPdf, int ng, float gPdf) {
    float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
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

    vec3f wo = normalize(vec3f{
        rayhit.ray.dir_x,
        rayhit.ray.dir_y,
        rayhit.ray.dir_z
    });

    // Move position in direction of geom normal 
    vec3f org = vec3f{
        rayhit.ray.org_x + wo.x * rayhit.ray.tfar + Ng.x * EPSILON,
        rayhit.ray.org_y + wo.y * rayhit.ray.tfar + Ng.y * EPSILON,
        rayhit.ray.org_z + wo.z * rayhit.ray.tfar + Ng.z * EPSILON
    };

    if (mats.specular[geomID]) {
        vec3f wi = reflect(wo, Ns);
        // build new RTCRayHit
        RTCRayHit newRH = makeRayHit(org, wi, 0);
        return mats.albedo[geomID] * traceRay(qctx, iargs, newRH, depth + 1);
    } else if (mats.emissive[geomID]) {
        return mats.albedo[geomID];
    } else {
        // This is terrible, but I think it works thanks chatGPT
        //  ─────────────────────── 1. LIGHT-SAMPLED PATH ───────────────────────
        vec3f Lo_light{0};
        {
            std::uniform_int_distribution<size_t> pick(0, emissives.size() - 1);
            const EmissiveTri &light = emissives[pick(rng)];

            LightSample s = EmissiveTriSample(light, org, randomVec2f());

            // probability of selecting this exact direction through the “pick one light” strategy
            float pdfLight = s.pdf / float(emissives.size());

            if (pdfLight > 0.f) {
                RTCRay shadowRay = makeRay(org, s.wi, 0, s.dist - EPSILON);
                rtcOccluded1(scene, &shadowRay, NULL);

                if (shadowRay.tfar > 0.f) {                       // not shadowed
                    float cosIn = dot(Ns, s.wi);
                    if (cosIn > 0.f) {
                        float pdfBSDF = cosIn * float(one_over_pi);   // competing pdf
                        float w       = powerHeuristic(1, pdfLight, 1, pdfBSDF);

                        Lo_light = light.Le * cosIn / pdfLight * w;
                    }
                }
            }
        }

        //  ─────────────────────── 2. BSDF-SAMPLED PATH ────────────────────────
        vec3f Lo_bsdf{0};
        {
            // cosine-weighted hemisphere sample in local frame
            vec3f h  = cosineSampleHemisphere(randomVec2f());
            vec3f b1, b2;
            frisvad(Ns, b1, b2);

            vec3f wi        = h.x * b1 + h.y * b2 + h.z * Ns;
            float cosIn     = h.z;
            float pdfBSDF   = cosIn * float(one_over_pi);

            if (pdfBSDF > 0.f) {
                // trace the BSDF ray
                RTCRayHit rh = makeRayHit(org, wi, 0);
                vec3f      Li = traceRay(qctx, iargs, rh, depth + 1);

                // competing pdf from the light sampler for *this* direction
                float pdfLight = 0.f;
                rh = makeRayHit(org, wi, 0);

                rtcIntersect1(scene, &rh, &iargs);
                if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) { 
                    for (const EmissiveTri &e : emissives) {
                        if (e.geomID == rh.hit.geomID && e.primID == rh.hit.primID) { // TODO: need a better way of checking if it hit an emissive
                            // we hit an emissive triangle: convert its area pdf to solid angle
                            vec3f p0, p1, p2;
                            getTriangleVerts(e, p0, p1, p2);
                            vec3f Ng   = normalize(cross(p1 - p0, p2 - p0));
                            float dist = rh.ray.tfar;
                            float cosL = dot(Ng, -wi);

                            if (cosL > 0.f) {
                                pdfLight = (dist * dist) / (e.area * cosL);
                                pdfLight /= float(emissives.size());   // account for random-light pick
                            }
                            break;
                        }
                    }
                }

                float w = powerHeuristic(1, pdfBSDF, 1, pdfLight);
                Lo_bsdf = Li * cosIn / pdfBSDF * w;
            }
        }

        //  ─────────────────────── 3. COMBINE & RETURN ────────────────────────
        vec3f Lo = Lo_light + Lo_bsdf;

        // Lambertian BRDF: f_r = albedo / π
        return mats.albedo[geomID] * Lo * float(one_over_pi);
    }
}

inline void addSample(Framebuffer& fb,
                      size_t pixel, const vec3f& c, uint32_t spp) {
    float* px = &fb.pixels[pixel * 3];
    px[0] += c.x / float(spp);
    px[1] += c.y / float(spp);
    px[2] += c.z / float(spp);
}

/* -------------------------------------------------------------------------- */
int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s scene.obj\n", argv[0]);
        return 0;
    }
    const std::string objPath = argv[1];
    /* TODO: - Change random number generation
             - Make robust cosine
    */


             

    /* Setup device + scene */
    device = initializeDevice();
    buildSceneFromOBJ(objPath);

    // Camera
    Camera cam;
    cam.focal_length     = 0.050f;
    cam.width            = 0.024;
    cam.height           = 0.024;
    cam.position         = vec3f(0.f, 0.0f,  2.5f);
    cam.width_px  = 1200;
    cam.height_px = 1200;

    const uint32_t SPP = 100*128;
    const uint32_t W = cam.width_px;
    const uint32_t H = cam.height_px;
    const size_t   N = size_t(W) * H * SPP;

    static_assert((SPP & 3) == 0, "SPP must be divisible by 4 for rtcIntersect4"); //TODO
    // Eye rays
    Basis bas;
    bas.x = vec3f(1, 0, 0);
    bas.y = vec3f(0, 1, 0);
    bas.z = vec3f(0, 0, 1);

    // Framebuffer
    Framebuffer fb;
    fb.pixels.resize(W * H * 3);

    struct Tile { int x0, y0, x1, y1; };

    auto renderTile = [&](const Tile& T) {
        RTCRayQueryContext qctx;
        rtcInitRayQueryContext(&qctx);

        RTCIntersectArguments iargs;
        rtcInitIntersectArguments(&iargs);
        iargs.context = &qctx;
        iargs.flags   = RTC_RAY_QUERY_FLAG_COHERENT;

        SensorSamples pixSamples;   allocSensorSamples(pixSamples, SPP);
        RayBuffer     pixRays;      allocRayBuffer    (pixRays,    SPP);
        for (int y = T.y0; y < T.y1; ++y) {
            for (int x = T.x0; x < T.x1; ++x) {
                generateSamplesForPixel(x, y,
                                        cam.width_px, cam.height_px,
                                        cam.width,     cam.height,
                                        SPP, pixSamples);

                generateRaysForPixel(cam, bas, pixSamples, SPP, pixRays);

                size_t pixelId = size_t(y) * W + x;
                for (uint32_t s = 0; s < SPP; ++s) {
                    vec3f col = traceRay(qctx, iargs, pixRays.rh[s], 0);
                    addSample(fb, pixelId, col, SPP);
                }
            }
        }
        freeRayBuffer(pixRays);
        freeSensorSamples(pixSamples);
    };

    auto t0 = std::chrono::high_resolution_clock::now();
    const int TILE = 32;
    std::vector<Tile> tiles;
    for (int y = 0; y < int(H); y += TILE)
        for (int x = 0; x < int(W); x += TILE)
            tiles.push_back({ x, y,
                            std::min(x+TILE, int(W)),
                            std::min(y+TILE, int(H)) });

    std::atomic<size_t> next{0};
    unsigned hw = std::thread::hardware_concurrency();
    std::vector<std::thread> pool;

    for (unsigned t = 0; t < hw; ++t)
        pool.emplace_back([&] {
            while (true) {
                size_t id = next.fetch_add(1, std::memory_order_relaxed);
                if (id >= tiles.size()) break;
                renderTile(tiles[id]);
            }
        #ifdef DEBUG
        rayTraversals.fetch_add(tlsRays, std::memory_order_relaxed);
        #endif
        });

    for (auto& th : pool) th.join();
    auto t1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> dt = t1 - t0;
    double seconds = dt.count();
    #ifdef DEBUG
    double mrays_per_s = (double)rayTraversals / seconds / 1e6;

    std::printf("Rendered %zu rays in %.3f s → %.2f Mrays/s\n",
                (size_t)rayTraversals, seconds, mrays_per_s);
    #else
    std::printf("Rendered in %.3f s\n", seconds);
    #endif

    // write PNG
    writePNG(fb, W, H, "render.png");
    printf("Wrote render.png (%ux%u)\n", W, H);

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return 0;
}