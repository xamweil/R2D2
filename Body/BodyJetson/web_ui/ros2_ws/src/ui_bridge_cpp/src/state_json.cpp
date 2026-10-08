#include "state_json.hpp"

#include <nlohmann/json.hpp>

#include <chrono>

namespace ui_bridge {

namespace {

using json = nlohmann::json;
using Clock = std::chrono::system_clock;

double to_sec(Clock::time_point tp) {
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

json to_json(const std_msgs::msg::Header &h) {
    return {{"stamp", {{"sec", h.stamp.sec}, {"nanosec", h.stamp.nanosec}}},
            {"frame_id", h.frame_id}};
}

json to_json(const tcp_msg::msg::MPU6500Sample &m) {
    return {{"accel", m.accel}, {"gyro", m.gyro}, {"ts_ms", m.ts_ms}};
}

json to_json(const sensor_msgs::msg::CameraInfo &m) {
    return {{"header", to_json(m.header)},
            {"height", m.height},
            {"width", m.width},
            {"distortion_model", m.distortion_model},
            {"d", m.d},
            {"k", m.k},
            {"r", m.r},
            {"p", m.p}};
}

json to_json(const sensor_msgs::msg::CompressedImage &m) {
    // Raw image bytes are not sent over the websocket, only their size.
    return {{"header", to_json(m.header)},
            {"format", m.format},
            {"data", {{"_bytes_len", m.data.size()}}}};
}

json to_json(const vision_msgs::msg::Detection2DArray &m) {
    json detections = json::array();
    for (const auto &det : m.detections) {
        json results = json::array();
        for (const auto &res : det.results) {
            results.push_back({{"hypothesis",
                                {{"class_id", res.hypothesis.class_id},
                                 {"score", res.hypothesis.score}}}});
        }
        detections.push_back(
            {{"id", det.id},
             {"bbox",
              {{"center",
                {{"position",
                  {{"x", det.bbox.center.position.x},
                   {"y", det.bbox.center.position.y}}},
                 {"theta", det.bbox.center.theta}}},
               {"size_x", det.bbox.size_x},
               {"size_y", det.bbox.size_y}}},
             {"results", std::move(results)}});
    }
    return {{"header", to_json(m.header)},
            {"detections", std::move(detections)}};
}

template <class T>
json entry_json(const Slot<T> &slot, Clock::time_point now, double stale_sec) {
    auto snap = slot.load();
    if (!snap.msg) {
        return {{"present", false},
                {"stale", true},
                {"t", nullptr},
                {"data", nullptr}};
    }
    double age = std::chrono::duration<double>(now - snap.recv_time).count();
    return {{"present", true},
            {"stale", age > stale_sec},
            {"age", age},
            {"t", to_sec(snap.recv_time)},
            {"data", to_json(*snap.msg)}};
}

} // namespace

std::string build_robot_state_json(const TelemetryStore &store,
                                   double stale_sec) {
    auto now = Clock::now();
    auto entry = [&](const auto &slot) {
        return entry_json(slot, now, stale_sec);
    };

    json entries = {
        {"imu_left_foot", entry(store.imu_left_foot)},
        {"imu_left_leg", entry(store.imu_left_leg)},
        {"imu_right_foot", entry(store.imu_right_foot)},
        {"imu_right_leg", entry(store.imu_right_leg)},
        {"imu_body", entry(store.imu_body)},
        {"camera_info", entry(store.camera_info)},
        {"image_raw_compressed", entry(store.compressed_image)},
        {"scene_detections", entry(store.scene_detections)},
        {"tracking_tracks", entry(store.tracking_tracks)},
    };

    json msg = {{"type", "robot_state"},
                {"payload",
                 {{"t", to_sec(now)},
                  {"stale_sec", stale_sec},
                  {"entries", std::move(entries)}}}};
    return msg.dump();
}

std::string build_hello_json(double state_hz, double stale_sec) {
    json msg = {{"type", "hello"},
                {"payload",
                 {{"dev_unsafe", false},
                  {"state_hz", state_hz},
                  {"stale_sec", stale_sec}}}};
    return msg.dump();
}

} // namespace ui_bridge
