// =============================================================================
// Motor Control Protocol, 34-byte frame, little-endian
//
// Motors (index):
//   0: M_mid  1: M_head  2: M_shoulder_l  3: M_shoulder_r  4: M_drive_l
//   5: M_drive_r
//
// Control word [bytes 0–3], u32, 4 bits per motor (motor 0 in LSB):
//   bit 0: enable, bit 1: direction, bit 2: angle set, bit 3: velocity set
//
// Value groups [bytes 4–33], 5 bytes per motor, ordered by index:
//   bytes 0–3: angle value (float)
//   byte  4:   velocity value (u8)
//
//  0      4      9      14     19     24     29
// ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┐
// │ ctrl │ mid  │ head │ sh_l │ sh_r │ dr_l │ dr_r │
// │ u32  │ f+v  │ f+v  │ f+v  │ f+v  │ f+v  │ f+v  │
// └──────┴──────┴──────┴──────┴──────┴──────┴──────┘
// =============================================================================

#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace motor_protocol {

constexpr size_t MOTOR_COUNT = 6;

enum MotorIndex : uint8_t {
    M_mid = 0,
    M_head = 1,
    M_shoulder_l = 2,
    M_shoulder_r = 3,
    M_drive_l = 4,
    M_drive_r = 5,
};

struct Motor {
    bool enable = false;
    bool direction = false;
    bool angle_set = false;
    bool velocity_set = false;
    float angle_value = 0.0F;
    uint8_t velocity_value = 0;
};

// Size of frame when serialized to byte array
// 4 bytes (control word u32) + MOTOR_COUNT * 5 (angle float + velocity u8)
constexpr size_t FRAME_SIZE_BYTES = 4 + (MOTOR_COUNT * 5);

struct Frame {
    std::array<Motor, MOTOR_COUNT> motors{};

    // Serialize the frame into a byte buffer of size FRAME_SIZE_BYTES.
    // Returns false if len < FRAME_SIZE_BYTES, true on success.
    bool serialize(uint8_t *out, size_t len) const {
        if (len < FRAME_SIZE_BYTES) {
            return false;
        }

        // Control word
        uint32_t control = 0;
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            const auto &m = motors[i];
            uint32_t bits = 0;
            if (m.enable)
                bits |= (1U << 0);
            if (m.direction)
                bits |= (1U << 1);
            if (m.angle_set)
                bits |= (1U << 2);
            if (m.velocity_set)
                bits |= (1U << 3);
            control |= (bits << (i * 4));
        }

        out[0] = static_cast<uint8_t>((control >> 0) & 0xFF);
        out[1] = static_cast<uint8_t>((control >> 8) & 0xFF);
        out[2] = static_cast<uint8_t>((control >> 16) & 0xFF);
        out[3] = static_cast<uint8_t>((control >> 24) & 0xFF);

        // Value groups: 5 bytes per motor (angle float + velocity u8)
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            size_t off = 4 + (i * 5);
            uint32_t a = 0;
            std::memcpy(&a, &motors[i].angle_value, sizeof(a));
            out[off + 0] = static_cast<uint8_t>((a >> 0) & 0xFF);
            out[off + 1] = static_cast<uint8_t>((a >> 8) & 0xFF);
            out[off + 2] = static_cast<uint8_t>((a >> 16) & 0xFF);
            out[off + 3] = static_cast<uint8_t>((a >> 24) & 0xFF);
            out[off + 4] = motors[i].velocity_value;
        }

        return true;
    }

    bool serialize(std::array<uint8_t, FRAME_SIZE_BYTES> &out) const {
        return serialize(out.data(), out.size());
    }

    // Deserialize a byte buffer into this frame, overwriting all motor fields.
    // Returns false if len < FRAME_SIZE_BYTES, true on success.
    bool deserialize(const uint8_t *in, size_t len) {
        if (len < FRAME_SIZE_BYTES) {
            return false;
        }
        uint32_t control = static_cast<uint32_t>(in[0]) |
                           (static_cast<uint32_t>(in[1]) << 8) |
                           (static_cast<uint32_t>(in[2]) << 16) |
                           (static_cast<uint32_t>(in[3]) << 24);

        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            uint32_t bits = (control >> (i * 4)) & 0x0F;
            motors[i].enable = ((bits & (1U << 0)) != 0U);
            motors[i].direction = ((bits & (1U << 1)) != 0U);
            motors[i].angle_set = ((bits & (1U << 2)) != 0U);
            motors[i].velocity_set = ((bits & (1U << 3)) != 0U);

            size_t off = 4 + (i * 5);
            uint32_t a = static_cast<uint32_t>(in[off + 0]) |
                         (static_cast<uint32_t>(in[off + 1]) << 8) |
                         (static_cast<uint32_t>(in[off + 2]) << 16) |
                         (static_cast<uint32_t>(in[off + 3]) << 24);
            std::memcpy(&motors[i].angle_value, &a, sizeof(a));
            motors[i].velocity_value = in[off + 4];
        }

        return true;
    }

    bool deserialize(const std::array<uint8_t, FRAME_SIZE_BYTES> &in) {
        return deserialize(in.data(), in.size());
    }

    Motor &operator[](MotorIndex idx) {
        return motors[static_cast<size_t>(idx)];
    }
    const Motor &operator[](MotorIndex idx) const {
        return motors[static_cast<size_t>(idx)];
    }

    bool operator==(const Frame &other) const {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            const auto &a = motors[i];
            const auto &b = other.motors[i];
            if (a.enable != b.enable || a.direction != b.direction ||
                a.angle_set != b.angle_set ||
                a.velocity_set != b.velocity_set ||
                a.angle_value != b.angle_value ||
                a.velocity_value != b.velocity_value)
                return false;
        }
        return true;
    }

    bool operator!=(const Frame &other) const {
        return !(*this == other);
    }

    std::string to_string() const {
        static constexpr const char *names[MOTOR_COUNT] = {
            "mid      ",  "head     ", "shoulder_l",
            "shoulder_r", "drive_l  ", "drive_r  "};
        std::string out;
        out.reserve(256);
        char buf[80];
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            const auto &m = motors[i];
            int n = std::snprintf(buf, sizeof(buf),
                                  "[%zu] %s  en:%d dir:%d ang_set:%d "
                                  "vel_set:%d  angle:%7.2f  vel:%3u\n",
                                  i, names[i], static_cast<int>(m.enable),
                                  static_cast<int>(m.direction),
                                  static_cast<int>(m.angle_set),
                                  static_cast<int>(m.velocity_set),
                                  static_cast<double>(m.angle_value),
                                  static_cast<unsigned>(m.velocity_value));
            if (n > 0)
                out.append(buf, static_cast<size_t>(n) < sizeof(buf)
                                    ? static_cast<size_t>(n)
                                    : sizeof(buf) - 1);
        }
        return out;
    }
};

} // namespace motor_protocol

#endif // MOTOR_PROTOCOL_H
