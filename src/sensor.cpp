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

bool writePNG(const Framebuffer& fb, uint32_t W, uint32_t H, const char* path) {
    std::vector<unsigned char> img;
    img.resize(size_t(W) * H * 3);
    for (size_t i = 0, n = img.size(); i < n; ++i) {
        float p = fb.pixels[i] * 255.0f;
        float v = std::clamp(p, 0.0f, 255.0f);
        img[i] = static_cast<unsigned char>(v + 0.5f);
    }

    int stride = W * 3;
    return stbi_write_png(path, W, H, 3, img.data(), stride) != 0;
}