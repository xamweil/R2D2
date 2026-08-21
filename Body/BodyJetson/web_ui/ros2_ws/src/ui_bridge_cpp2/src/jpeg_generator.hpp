// NOLINTBEGIN
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ui_bridge_cpp {

struct JpegGenerator {
    int width = 650;
    int height = 480;
    std::vector<uint8_t> pixels{};
    int frame_num = 0;
    std::string cached_black_frame_{};

    JpegGenerator();
    JpegGenerator(int w, int h);
    std::string next_frame(int quality = 80);
};
// NOLINTEND

} // namespace ui_bridge_cpp
