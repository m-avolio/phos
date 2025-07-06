#include "sensor.h"

#define TINYEXR_USE_MINIZ 0
#ifndef TINYEXR_IMPLEMENTATION
#define TINYEXR_IMPLEMENTATION
#endif

#include "zlib.h"
#include "tinyexr.h"


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

bool writeEXR(const Framebuffer& fb, uint32_t W, uint32_t H, const char* path) {
    EXRHeader hdr;  InitEXRHeader(&hdr);
    EXRImage  img;  InitEXRImage (&img);

    img.num_channels = 3;
    std::vector<float> r(W*H), g(W*H), b(W*H);
    for (size_t i = 0; i < W*H; ++i) {
        r[i] = fb.pixels[i*3+0];
        g[i] = fb.pixels[i*3+1];
        b[i] = fb.pixels[i*3+2];
    }
    float* chan[3] = { b.data(), g.data(), r.data() };
    img.images = reinterpret_cast<unsigned char**>(chan);
    img.width  = W;
    img.height = H;

    hdr.num_channels = 3;
    hdr.channels = (EXRChannelInfo*)malloc(sizeof(EXRChannelInfo)*3);
    strncpy(hdr.channels[0].name, "B", 255);
    strncpy(hdr.channels[1].name, "G", 255);
    strncpy(hdr.channels[2].name, "R", 255);

    hdr.pixel_types           = (int*)malloc(sizeof(int)*3);
    hdr.requested_pixel_types = (int*)malloc(sizeof(int)*3);
    for (int i=0;i<3;++i)
        hdr.pixel_types[i] = hdr.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;

    const char* err;
    int ret = SaveEXRImageToFile(&img, &hdr, path, &err);
    if (ret != TINYEXR_SUCCESS) {
        fprintf(stderr,"TinyEXR: %s\n", err);
        FreeEXRErrorMessage(err);
        return false;
    }
    free(hdr.channels); free(hdr.pixel_types); free(hdr.requested_pixel_types);
    return true;
}

