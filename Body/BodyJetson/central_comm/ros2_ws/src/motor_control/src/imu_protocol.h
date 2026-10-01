#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <cstdio>

namespace imu_protocol {

constexpr uint8_t SOF = 0xAB;
constexpr size_t IMU_COUNT = 6;
constexpr size_t IMU_SAMPLE_SIZE_BYTES = 10;
constexpr size_t FRAME_SIZE_BYTES = IMU_COUNT * IMU_SAMPLE_SIZE_BYTES;

enum ImuIndex : uint8_t {
    IMU_head = 0,
    IMU_body = 1,
    IMU_leg_l_foot = 2,
    IMU_leg_l_leg = 3,
    IMU_leg_r_foot = 4,
    IMU_leg_r_leg = 5,
};

struct ImuSample {
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    uint32_t ts_ms = 0;   // ts_ms == 0 => invalid / missing / stale
};

struct Frame {
    std::array<ImuSample, IMU_COUNT> imus{};

    bool serialize(uint8_t *out, size_t len) const {
        if (len < FRAME_SIZE_BYTES) return false;

        for (size_t i = 0; i < IMU_COUNT; ++i) {
            const auto &s = imus[i];
            const size_t off = i * IMU_SAMPLE_SIZE_BYTES;

            out[off + 0] = static_cast<uint8_t>((static_cast<uint16_t>(s.accel_x) >> 0) & 0xFF);
            out[off + 1] = static_cast<uint8_t>((static_cast<uint16_t>(s.accel_x) >> 8) & 0xFF);

            out[off + 2] = static_cast<uint8_t>((static_cast<uint16_t>(s.accel_y) >> 0) & 0xFF);
            out[off + 3] = static_cast<uint8_t>((static_cast<uint16_t>(s.accel_y) >> 8) & 0xFF);

            out[off + 4] = static_cast<uint8_t>((static_cast<uint16_t>(s.accel_z) >> 0) & 0xFF);
            out[off + 5] = static_cast<uint8_t>((static_cast<uint16_t>(s.accel_z) >> 8) & 0xFF);

            out[off + 6] = static_cast<uint8_t>((s.ts_ms >> 0) & 0xFF);
            out[off + 7] = static_cast<uint8_t>((s.ts_ms >> 8) & 0xFF);
            out[off + 8] = static_cast<uint8_t>((s.ts_ms >> 16) & 0xFF);
            out[off + 9] = static_cast<uint8_t>((s.ts_ms >> 24) & 0xFF);
        }

        return true;
    }

    bool serialize(std::array<uint8_t, FRAME_SIZE_BYTES> &out) const {
        return serialize(out.data(), out.size());
    }

    bool deserialize(const uint8_t *in, size_t len) {
        if (len < FRAME_SIZE_BYTES) return false;

        for (size_t i = 0; i < IMU_COUNT; ++i) {
            auto &s = imus[i];
            const size_t off = i * IMU_SAMPLE_SIZE_BYTES;

            s.accel_x = static_cast<int16_t>(
                static_cast<uint16_t>(in[off + 0]) |
                (static_cast<uint16_t>(in[off + 1]) << 8));

            s.accel_y = static_cast<int16_t>(
                static_cast<uint16_t>(in[off + 2]) |
                (static_cast<uint16_t>(in[off + 3]) << 8));

            s.accel_z = static_cast<int16_t>(
                static_cast<uint16_t>(in[off + 4]) |
                (static_cast<uint16_t>(in[off + 5]) << 8));

            s.ts_ms = static_cast<uint32_t>(in[off + 6]) |
                      (static_cast<uint32_t>(in[off + 7]) << 8) |
                      (static_cast<uint32_t>(in[off + 8]) << 16) |
                      (static_cast<uint32_t>(in[off + 9]) << 24);
        }

        return true;
    }

    bool deserialize(const std::array<uint8_t, FRAME_SIZE_BYTES> &in) {
        return deserialize(in.data(), in.size());
    }

    ImuSample &operator[](ImuIndex idx) {
        return imus[static_cast<size_t>(idx)];
    }

    const ImuSample &operator[](ImuIndex idx) const {
        return imus[static_cast<size_t>(idx)];
    }

    std::string to_string() const {
        static constexpr const char *names[IMU_COUNT] = {
            "head      ",
            "body      ",
            "leg_l_foot",
            "leg_l_leg ",
            "leg_r_foot",
            "leg_r_leg "
        };

        std::string out;
        char buf[128];

        for (size_t i = 0; i < IMU_COUNT; ++i) {
            const auto &s = imus[i];
            int n = std::snprintf(
                buf, sizeof(buf),
                "[%zu] %s ax:%6d ay:%6d az:%6d ts:%10u\n",
                i, names[i],
                static_cast<int>(s.accel_x),
                static_cast<int>(s.accel_y),
                static_cast<int>(s.accel_z),
                static_cast<unsigned>(s.ts_ms));
            if (n > 0) {
                out.append(buf, static_cast<size_t>(n) < sizeof(buf)
                                    ? static_cast<size_t>(n)
                                    : sizeof(buf) - 1);
            }
        }

        return out;
    }
};

} // namespace imu_protocol

#endif