// NOLINTBEGIN
#include "jpeg_generator.hpp"

#include <cmath>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

static void jpg_write_cb(void *ctx, void *data, int size) {
    auto *out = static_cast<std::string *>(ctx);
    out->append(static_cast<const char *>(data), size);
}

JpegGenerator::JpegGenerator(int w, int h)
    : width(w), height(h), pixels(w * h * 3, 0) {
    // Pre-encode a single black frame; pixels are already zeroed
    cached_black_frame_.reserve(width * height / 4);
    stbi_write_jpg_to_func(jpg_write_cb, &cached_black_frame_, width, height, 3,
                           pixels.data(), 80);
}

// std::string FrameGenerator::next(int /*quality*/) {
//     return cached_black_frame_;
// }

std::string JpegGenerator::next_frame(int quality) {
    frame_num++;
    float t = frame_num * 0.05f;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int i = (y * width + x) * 3;

            float fx = (float)x / width;
            float fy = (float)y / height;
            float wave = sinf(fx * 6.0f + t) * 0.5f + 0.5f;
            float wave2 = cosf(fy * 4.0f - t * 0.7f) * 0.5f + 0.5f;

            pixels[i + 0] = (uint8_t)(wave * 200 + 30);
            pixels[i + 1] = (uint8_t)(wave2 * 180 + 40);
            pixels[i + 2] = (uint8_t)((1.0f - fx) * 200 + 30);
        }
    }

    // Moving vertical bar (white)
    int bar_x = (int)(((sinf(t * 0.8f) + 1.0f) / 2.0f) * (width - 20));
    for (int y = 0; y < height; y++) {
        for (int dx = 0; dx < 20 && bar_x + dx < width; dx++) {
            int i = (y * width + (bar_x + dx)) * 3;
            pixels[i + 0] = 255;
            pixels[i + 1] = 255;
            pixels[i + 2] = 255;
        }
    }

    // Frame counter bar at the top
    for (int y = 0; y < 30; y++) {
        int fill_w = (frame_num % 200) * width / 200;
        for (int x = 0; x < fill_w; x++) {
            int i = (y * width + x) * 3;
            pixels[i + 0] = 30;
            pixels[i + 1] = 200;
            pixels[i + 2] = 100;
        }
    }

    std::string jpeg_data;
    jpeg_data.reserve(width * height / 4);
    stbi_write_jpg_to_func(jpg_write_cb, &jpeg_data, width, height, 3,
                           pixels.data(), quality);
    return jpeg_data;
}
// NOLINTEND
