#include "bridge_node.hpp"

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <tcp_msg/msg/detail/mpu6500_sample__struct.hpp>

namespace ui_bridge {

// static constexpr uint64_t BROADCAST_FREQ = 100;
static constexpr int PORT = 9090;

BridgeNode::BridgeNode(TelemetryStore &store,
                       const rclcpp::NodeOptions &options)
    : Node("ui_bridge_cpp", options),
      store_(store) {
    declare_parameter("port", PORT);
    // declare_parameter("doc_root", std::string("/home/ros/frontend"));
    // declare_parameter("mjpeg_fps", 3);
    //
    // port_ = static_cast<int>(get_parameter("port").as_int());
    // doc_root_ = get_parameter("doc_root").as_string();
    // mjpeg_fps_ = static_cast<int>(get_parameter("mjpeg_fps").as_int());

    // imu subscriptions
    imu_left_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/foot", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_left_foot.store(msg);
        });
    imu_left_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_l/imu/leg", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_left_leg.store(msg);
        });
    imu_right_foot_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/foot", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_right_foot.store(msg);
        });
    imu_right_leg_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/leg_r/imu/leg", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_right_leg.store(msg);
        });
    imu_body_sub_ = create_subscription<tcp_msg::msg::MPU6500Sample>(
        "/Body/mpu", rclcpp::SensorDataQoS(),
        [this](const tcp_msg::msg::MPU6500Sample::ConstSharedPtr &msg) {
            store_.imu_body.store(msg);
        });

    // camera subscriptions
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/relay/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
            store_.camera_info.store(msg);
        });
    compressed_image_sub_ =
        create_subscription<sensor_msgs::msg::CompressedImage>(
            "/relay/camera/image_raw/compressed", rclcpp::SensorDataQoS(),
            [this](
                const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
                store_.compressed_image.store(msg);
            });

    // tracking subscriptions
    scene_detections_sub_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "/scene_understanding/detections", rclcpp::SensorDataQoS(),
            [this](
                const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg) {
                // sensor_state_.scene_detections.msg = msg;
                // sensor_state_.scene_detections.recv_time = now_sec();
                // sensor_state_.scene_detections.received = true;
                store_.scene_detections.store(msg);
            });
    tracking_tracks_sub_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "/tracking/tracks", rclcpp::SensorDataQoS(),
            [this](
                const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg) {
                // sensor_state_.tracking_tracks.msg = msg;
                // sensor_state_.tracking_tracks.recv_time = now_sec();
                // sensor_state_.tracking_tracks.received = true;
                store_.tracking_tracks.store(msg);
            });

    // state_timer_ =
    // create_wall_timer(std::chrono::milliseconds(BROADCAST_FREQ),
    //                                  [this]() { broadcast_state(); });
}

// void BridgeNode::set_broadcast(
//     std::function<void(const std::string &)> broadcast_fn) {
//     broadcast_fn_ = std::move(broadcast_fn);
// }
//
// sensor_msgs::msg::CompressedImage::ConstSharedPtr
// BridgeNode::get_latest_image() {
//     std::lock_guard<std::mutex> lock(image_mutex_);
//     return image_;
// }
//
// void BridgeNode::camera_info_callback(
//     const sensor_msgs::msg::CameraInfo & /*msg*/) {
// }
//
// void BridgeNode::compressed_image_callback(
//     const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
//     {
//         std::lock_guard<std::mutex> lock(image_mutex_);
//         image_ = msg;
//     }
// }
//
// double BridgeNode::now_sec() {
//     using clock = std::chrono::steady_clock;
//     return std::chrono::duration<double>(clock::now().time_since_epoch())
//         .count();
// }
//
// static void write_detection_entry(rapidjson::Writer<rapidjson::StringBuffer>
// &w,
//                                   const char *key, const DetectionEntry &e,
//                                   double now, bool include_id) {
//     w.Key(key);
//     w.StartObject();
//     w.Key("present");
//     w.Bool(e.received);
//     w.Key("stale");
//     w.Bool(!e.received || (now - e.recv_time) > SensorState::STALE_SEC);
//     w.Key("age");
//     w.Double(e.received ? now - e.recv_time : -1.0);
//     w.Key("data");
//     if (e.received) {
//         w.StartArray();
//         for (const auto &det : e.msg.detections) {
//             w.StartArray();
//             w.Double(det.bbox.center.position.x);
//             w.Double(det.bbox.center.position.y);
//             w.Double(det.bbox.size_x);
//             w.Double(det.bbox.size_y);
//             if (!det.results.empty()) {
//                 w.String(det.results[0].hypothesis.class_id.c_str());
//                 w.Double(det.results[0].hypothesis.score);
//             } else {
//                 w.String("");
//                 w.Double(0.0);
//             }
//             if (include_id) {
//                 w.String(det.id.c_str());
//             }
//             w.EndArray();
//         }
//         w.EndArray();
//     } else {
//         w.Null();
//     }
//     w.EndObject();
// }
//
// void BridgeNode::broadcast_state() {
//     if (!broadcast_fn_)
//         return;
//
//     double now = now_sec();
//     rapidjson::StringBuffer buf;
//     rapidjson::Writer<rapidjson::StringBuffer> w(buf);
//
//     w.StartObject();
//     w.Key("type");
//     w.String("robot_state");
//     w.Key("payload");
//     w.StartObject();
//     w.Key("t");
//     w.Double(now);
//     w.Key("stale_sec");
//     w.Double(SensorState::STALE_SEC);
//     w.Key("entries");
//     w.StartObject();
//
//     for (size_t i = 0; i < SensorState::NUM_IMU; ++i) {
//         const auto &e = sensor_state_.imu[i];
//         w.Key(IMU_ALIASES[i]);
//         w.StartObject();
//         w.Key("present");
//         w.Bool(e.received);
//         w.Key("stale");
//         w.Bool(!e.received || (now - e.recv_time) > SensorState::STALE_SEC);
//         w.Key("age");
//         w.Double(e.received ? now - e.recv_time : -1.0);
//         w.Key("data");
//         if (e.received) {
//             w.StartObject();
//             w.Key("accel");
//             w.StartArray();
//             for (int j = 0; j < 3; ++j)
//                 w.Int(e.msg.accel[j]);
//             w.EndArray();
//             w.Key("gyro");
//             w.StartArray();
//             for (int j = 0; j < 3; ++j)
//                 w.Int(e.msg.gyro[j]);
//             w.EndArray();
//             w.Key("ts_ms");
//             w.Uint(e.msg.ts_ms);
//             w.EndObject();
//         } else {
//             w.Null();
//         }
//         w.EndObject();
//     }
//
//     write_detection_entry(w, "scene_detections",
//     sensor_state_.scene_detections,
//                           now, false);
//     write_detection_entry(w, "tracking_tracks",
//     sensor_state_.tracking_tracks,
//                           now, true);
//
//     w.EndObject(); // entries
//     w.EndObject(); // payload
//     w.EndObject();
//
//     broadcast_fn_(std::string(buf.GetString(), buf.GetSize()));
// }
//
// //
// ---------------------------------------------------------------------------
// // Command handling
// //
// ---------------------------------------------------------------------------
//
// std::string BridgeNode::json_ok() {
//     return R"({"ok":true})";
// }
//
// std::string BridgeNode::json_error(const std::string &msg) {
//     rapidjson::StringBuffer buf;
//     rapidjson::Writer<rapidjson::StringBuffer> w(buf);
//     w.StartObject();
//     w.Key("ok");
//     w.Bool(false);
//     w.Key("error");
//     w.String(msg.c_str());
//     w.EndObject();
//     return std::string(buf.GetString(), buf.GetSize());
// }
//
// void BridgeNode::push_command(PendingCommand cmd) {
//     std::lock_guard<std::mutex> lock(cmd_mutex_);
//     cmd_queue_.push_back(std::move(cmd));
// }
//
// void BridgeNode::process_commands() {
//     std::deque<PendingCommand> batch;
//     {
//         std::lock_guard<std::mutex> lock(cmd_mutex_);
//         batch.swap(cmd_queue_);
//     }
//     for (auto &cmd : batch) {
//         try {
//             if (cmd.kind == "publish") {
//                 handle_publish(cmd.body, cmd.respond);
//             } else if (cmd.kind == "call_service") {
//                 handle_call_service(cmd.body, cmd.respond);
//             } else if (cmd.kind == "action_start") {
//                 handle_action_start(cmd.body, cmd.respond);
//             } else if (cmd.kind == "action_cancel") {
//                 handle_action_cancel(cmd.body, cmd.respond);
//             } else {
//                 cmd.respond(json_error("unknown command kind: " + cmd.kind));
//             }
//         } catch (const std::exception &e) {
//             RCLCPP_ERROR(get_logger(), "Command error: %s", e.what());
//             cmd.respond(json_error(e.what()));
//         }
//     }
// }
//
// void BridgeNode::handle_publish(
//     const std::string &body,
//     const std::function<void(const std::string &)> &respond) {
//     rapidjson::Document doc;
//     doc.Parse(body.c_str());
//     if (doc.HasParseError() || !doc.IsObject()) {
//         respond(json_error("invalid JSON"));
//         return;
//     }
//
//     std::string alias = doc.HasMember("alias") && doc["alias"].IsString()
//                             ? doc["alias"].GetString()
//                             : "";
//     if (alias != "motor_command") {
//         respond(json_error("unknown publish alias: " + alias));
//         return;
//     }
//     if (!doc.HasMember("msg") || !doc["msg"].IsObject()) {
//         respond(json_error("missing 'msg' object"));
//         return;
//     }
//     const auto &m = doc["msg"];
//
//     if (!motor_cmd_pub_) {
//         motor_cmd_pub_ = create_publisher<serial_msg::msg::MotorCommand>(
//             "/motor_command", 10);
//     }
//
//     serial_msg::msg::MotorCommand msg;
//     if (m.HasMember("ids") && m["ids"].IsArray())
//         for (auto &v : m["ids"].GetArray())
//             msg.ids.push_back(static_cast<uint8_t>(v.GetInt()));
//     if (m.HasMember("enable") && m["enable"].IsArray())
//         for (auto &v : m["enable"].GetArray())
//             msg.enable.push_back(v.GetBool());
//     if (m.HasMember("direction") && m["direction"].IsArray())
//         for (auto &v : m["direction"].GetArray())
//             msg.direction.push_back(v.GetBool());
//     if (m.HasMember("angle_set") && m["angle_set"].IsArray())
//         for (auto &v : m["angle_set"].GetArray())
//             msg.angle_set.push_back(v.GetBool());
//     if (m.HasMember("velocity_set") && m["velocity_set"].IsArray())
//         for (auto &v : m["velocity_set"].GetArray())
//             msg.velocity_set.push_back(v.GetBool());
//     if (m.HasMember("angle") && m["angle"].IsArray())
//         for (auto &v : m["angle"].GetArray())
//             msg.angle.push_back(static_cast<float>(v.GetDouble()));
//     if (m.HasMember("velocity") && m["velocity"].IsArray())
//         for (auto &v : m["velocity"].GetArray())
//             msg.velocity.push_back(static_cast<uint8_t>(v.GetInt()));
//
//     motor_cmd_pub_->publish(msg);
//     respond(json_ok());
// }
//
// void BridgeNode::handle_call_service(
//     const std::string &body,
//     const std::function<void(const std::string &)> &respond) {
//     rapidjson::Document doc;
//     doc.Parse(body.c_str());
//     if (doc.HasParseError() || !doc.IsObject()) {
//         respond(json_error("invalid JSON"));
//         return;
//     }
//
//     std::string alias = doc.HasMember("alias") && doc["alias"].IsString()
//                             ? doc["alias"].GetString()
//                             : "";
//     if (alias != "device_command") {
//         respond(json_error("unknown service alias: " + alias));
//         return;
//     }
//
//     double timeout_sec = 2.0;
//     if (doc.HasMember("timeout_sec") && doc["timeout_sec"].IsNumber())
//         timeout_sec = doc["timeout_sec"].GetDouble();
//
//     if (!device_cmd_client_) {
//         device_cmd_client_ =
//             create_client<serial_msg::srv::DeviceCommand>("/serial_command");
//     }
//
//     if (!device_cmd_client_->wait_for_service(std::chrono::milliseconds(
//             static_cast<int64_t>(timeout_sec * 1000)))) {
//         respond(json_error("service not available: /serial_command"));
//         return;
//     }
//
//     auto request =
//     std::make_shared<serial_msg::srv::DeviceCommand::Request>(); if
//     (doc.HasMember("request") && doc["request"].IsObject()) {
//         const auto &r = doc["request"];
//         if (r.HasMember("device_name") && r["device_name"].IsString())
//             request->device_name = r["device_name"].GetString();
//         if (r.HasMember("method_name") && r["method_name"].IsString())
//             request->method_name = r["method_name"].GetString();
//         if (r.HasMember("args") && r["args"].IsArray())
//             for (auto &v : r["args"].GetArray())
//                 request->args.push_back(v.GetInt64());
//     }
//
//     auto future = device_cmd_client_->async_send_request(request);
//
//     auto start = std::chrono::steady_clock::now();
//     auto timeout = std::chrono::duration<double>(timeout_sec);
//     while (rclcpp::ok()) {
//         rclcpp::spin_some(shared_from_this());
//         if (future.wait_for(std::chrono::milliseconds(0)) ==
//             std::future_status::ready)
//             break;
//         if (std::chrono::steady_clock::now() - start > timeout) {
//             respond(json_error("service call timed out"));
//             return;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     }
//
//     auto result = future.get();
//     rapidjson::StringBuffer buf;
//     rapidjson::Writer<rapidjson::StringBuffer> w(buf);
//     w.StartObject();
//     w.Key("ok");
//     w.Bool(true);
//     w.Key("response");
//     w.StartObject();
//     w.Key("response");
//     w.String(result->response.c_str());
//     w.EndObject();
//     w.EndObject();
//     respond(std::string(buf.GetString(), buf.GetSize()));
// }
//
// void BridgeNode::handle_action_start(
//     const std::string &body,
//     const std::function<void(const std::string &)> &respond) {
//     rapidjson::Document doc;
//     doc.Parse(body.c_str());
//     if (doc.HasParseError() || !doc.IsObject()) {
//         respond(json_error("invalid JSON"));
//         return;
//     }
//
//     std::string alias = doc.HasMember("alias") && doc["alias"].IsString()
//                             ? doc["alias"].GetString()
//                             : "";
//     if (alias != "follow_track") {
//         respond(json_error("unknown action alias: " + alias));
//         return;
//     }
//     if (!doc.HasMember("goal") || !doc["goal"].IsObject()) {
//         respond(json_error("missing 'goal' object"));
//         return;
//     }
//
//     if (!follow_track_client_) {
//         follow_track_client_ =
//             rclcpp_action::create_client<FollowTrack>(this, "/follow_track");
//     }
//
//     if (!follow_track_client_->wait_for_action_server(
//             std::chrono::seconds(2))) {
//         respond(json_error("action server not available: /follow_track"));
//         return;
//     }
//
//     auto goal_msg = FollowTrack::Goal();
//     const auto &g = doc["goal"];
//     if (g.HasMember("track_id") && g["track_id"].IsInt())
//         goal_msg.track_id = g["track_id"].GetInt();
//
//     auto future = follow_track_client_->async_send_goal(goal_msg);
//
//     auto start = std::chrono::steady_clock::now();
//     while (rclcpp::ok()) {
//         rclcpp::spin_some(shared_from_this());
//         if (future.wait_for(std::chrono::milliseconds(0)) ==
//             std::future_status::ready)
//             break;
//         if (std::chrono::steady_clock::now() - start >
//             std::chrono::seconds(2)) {
//             respond(json_error("action goal send timed out"));
//             return;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     }
//
//     auto goal_handle = future.get();
//     if (!goal_handle) {
//         respond(json_error("action goal rejected"));
//         return;
//     }
//
//     active_follow_goal_ = goal_handle;
//     respond(R"({"ok":true,"result":{"accepted":true}})");
// }
//
// void BridgeNode::handle_action_cancel(
//     const std::string &body,
//     const std::function<void(const std::string &)> &respond) {
//     rapidjson::Document doc;
//     doc.Parse(body.c_str());
//     if (doc.HasParseError() || !doc.IsObject()) {
//         respond(json_error("invalid JSON"));
//         return;
//     }
//
//     std::string alias = doc.HasMember("alias") && doc["alias"].IsString()
//                             ? doc["alias"].GetString()
//                             : "";
//     if (alias != "follow_track") {
//         respond(json_error("unknown action alias: " + alias));
//         return;
//     }
//
//     if (!active_follow_goal_) {
//         respond(
//             R"({"ok":true,"result":{"ok":true,"message":"no_active_goal"}})");
//         return;
//     }
//
//     auto cancel_future =
//         follow_track_client_->async_cancel_goal(active_follow_goal_);
//
//     auto start = std::chrono::steady_clock::now();
//     while (rclcpp::ok()) {
//         rclcpp::spin_some(shared_from_this());
//         if (cancel_future.wait_for(std::chrono::milliseconds(0)) ==
//             std::future_status::ready)
//             break;
//         if (std::chrono::steady_clock::now() - start >
//             std::chrono::seconds(2)) {
//             respond(json_error("cancel action goal timed out"));
//             return;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     }
//
//     active_follow_goal_.reset();
//     respond(R"({"ok":true,"result":{"ok":true}})");
// }
//

} // namespace ui_bridge
