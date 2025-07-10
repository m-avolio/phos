#include "sensor.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "rkcommon/math/vec.h"
#include <cstdint>

// Andrew Kensler 2013
unsigned permute(unsigned i, unsigned l, unsigned p) {
    unsigned w = l - 1;
    w |= w >> 1;
    w |= w >> 2;
    w |= w >> 4;
    w |= w >> 8;
    w |= w >> 16;

    do {
        i ^= p;
        i *= 0xe170893d;
        i ^= p >> 16;
        i ^= (i & w) >> 4;
        i ^= p >> 8;
        i *= 0x0929eb3f;
        i ^= p >> 23;
        i ^= (i & w) >> 1;
        i *= (1 | p >> 27);
        i *= 0x6935fa69;
        i ^= (i & w) >> 11;
        i *= 0x74dcb303;
        i ^= (i & w) >> 2;
        i *= 0x9e501cc3;
        i ^= (i & w) >> 2;
        i *= 0xc860a3df;
        i &= w;
        i ^= i >> 5;
    } while (i >= l);

    return (i + p) % l;
}

// Correlated Multi-Jittered Sampling
// Andrew Kensler 2013
float randfloat(unsigned i, unsigned p) {
    i ^= p;
    i ^= i >> 17;
    i ^= i >> 10;  i *= 0xb36534e5;
    i ^= i >> 12;  i *= 0x93fc4795;
    i ^= i >> 21;  i ^= 0xdf6e307f;
    i ^= i >> 17;  i *= 1 | (p >> 18);
    return i * (1.0f / 4294967808.0f);
}

// Andrew Kensler 2013
rkcommon::math::vec2f cmj(int s, int N, int p, float a = 1.0f) {
    int m = static_cast<int>(sqrtf(N * a));
    int n = (N + m - 1) / m;

    s = permute(s, N, p * 0x51633e2d);
    int sx = permute(s % m, m, p * 0x68bc21eb);
    int sy = permute(s / m, n, p * 0x02e5be93);

    float jx = randfloat(s, p * 0x967a889b);
    float jy = randfloat(s, p * 0x368cc8b7);

    rkcommon::math::vec2f r = rkcommon::math::vec2f(
        (sx + (sy + jx) / n) / m,
        (s + jy) / N
    );

    return r;
}

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