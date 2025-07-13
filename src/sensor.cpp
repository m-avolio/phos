#include "sensor.h"
#include "samplers.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "rkcommon/math/vec.h"
#include <cstdint>

void generateSensorSamples(std::uint32_t width_px,
                           std::uint32_t height_px,
                           float width,
                           float height,
                           SensorSamples& samples) {

    uint32_t W = width_px;
    uint32_t H = height_px;

    const float dx   = width  / float(W);
    const float dy   = height / float(H);
    const float left = -0.5f * width;
    const float top  =  0.5f * height;

    std::size_t i = 0;
    for (uint32_t y = 0; y < H; ++y)
        for (uint32_t x = 0; x < W; ++x, ++i)
            samples.points[i] = { left + (x + 0.5f) * dx,
                                  top  - (y + 0.5f) * dy };
}

void generateSensorSamples(std::uint32_t width_px,
                           std::uint32_t height_px,
                           float width,
                           float height,
                           std::uint32_t spp,
                           SensorSamples& samples) {

    const float dx   = width  / float(width_px);
    const float dy   = height / float(height_px);
    const float left = -0.5f * width;
    const float top  =  0.5f * height;

    std::size_t i = 0;
    for (uint32_t y = 0; y < height_px; ++y) {
        for (uint32_t x = 0; x < width_px; ++x) {
            const uint32_t pixelIndex = y * width_px + x;
            const int pixelSeed = pixelIndex * 0x9e3779b9; // or any scrambling constant

            for (uint32_t s = 0; s < spp; ++s, ++i) {
                rkcommon::math::vec2f u = cmj(s, spp, pixelSeed);

                float px = float(x) + u.x;
                float py = float(y) + u.y;

                samples.points[i] = {
                    left + px * dx,
                    top  - py * dy
                };
            }
        }
    }
}

void generateSamplesForPixel(uint32_t px, uint32_t py,
                                    uint32_t imageW, uint32_t imageH,
                                    float sensorW,  float sensorH,
                                    uint32_t spp,
                                    SensorSamples& samples) {
    const float dx   = sensorW  / float(imageW);
    const float dy   = sensorH  / float(imageH);
    const float left = -0.5f * sensorW;
    const float top  =  0.5f * sensorH;

    uint32_t pixelSeed = (px * 0x1f1f1f1f) ^ (py * 0x5f356495); // Magic numbers that I don't understand

    for (uint32_t s = 0; s < spp; ++s) {
        rkcommon::math::vec2f u = cmj(s, spp, pixelSeed);

        float sx = (float(px) + u.x) * dx;
        float sy = (float(py) + u.y) * dy;

        samples.points[s] = {
            left + sx,
            top  - sy
        };
    }
}

static float toneMapping(const float r) {
    // ACES Narkowicz 2016
    const float a = 2.51f;
    const float b = 0.03f;
    const float c = 2.43f;
    const float d = 0.59f;
    const float e = 0.14f;
    return std::clamp((r*(a*r+b)) / (r*(c*r+d)+e), 0.f, 1.f);
}

static float gammaCorrection(const float r, const float gamma = 1.0f) {
    // assumes r is within 0 to 1
    // gamma is typically 2.2, but the default is 1.0 to make it linear
    return pow(r, 1.0f / gamma);
}

bool writePNG(const Framebuffer& fb, uint32_t W, uint32_t H, const char* path) {
    std::vector<unsigned char> img;
    img.resize(size_t(W) * H * 3);
    for (size_t i = 0, n = img.size(); i < n; ++i) {
        float p = gammaCorrection(toneMapping(fb.pixels[i]), 2.2);
        img[i] = static_cast<uint8_t>(p * 255.0f + 0.5f);
    }

    int stride = W * 3;
    return stbi_write_png(path, W, H, 3, img.data(), stride) != 0;
}