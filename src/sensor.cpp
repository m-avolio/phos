#include "sensor.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

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