#include "state_json.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>

namespace ui_bridge {

namespace {

using json = nlohmann::json;
using Clock = std::chrono::system_clock;

double to_sec(Clock::time_point tp) {
    return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

void append_double(std::string &out, double v) {
    if (!std::isfinite(v)) {
        out += "null";
        return;
    }
    constexpr size_t DOUBLE_CHARS = 32;
    std::array<char, DOUBLE_CHARS> buf{};
    auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    out.append(buf.data(), res.ptr);
}

template <class Msg>
void append_entry(std::string &out, Topic<Msg> &topic, Clock::time_point now,
                  double stale_sec) {
    out += '"';
    out += topic.spec.key;
    out += "\":";

    auto snap = topic.slot.load();
    if (!snap.msg) {
        out += R"({"present":false,"stale":true,"t":null,"data":null})";
        return;
    }

    if (snap.generation != topic.seen_generation) {
        topic.cached_data.clear();
        topic.spec.serialize(*snap.msg, topic.cached_data);
        topic.seen_generation = snap.generation;
    }

    double age = std::chrono::duration<double>(now - snap.recv_time).count();
    out += R"({"present":true,"stale":)";
    out += age > stale_sec ? "true" : "false";
    out += R"(,"age":)";
    append_double(out, age);
    out += R"(,"t":)";
    append_double(out, to_sec(snap.recv_time));
    out += R"(,"data":)";
    out += topic.cached_data;
    out += '}';
}

} // namespace

void build_robot_state_json(TopicRegistry &topics, double stale_sec,
                            std::string &out) {
    auto now = Clock::now();

    out.clear();
    out += R"({"type":"robot_state","payload":{"t":)";
    append_double(out, to_sec(now));
    out += R"(,"stale_sec":)";
    append_double(out, stale_sec);
    out += R"(,"entries":{)";

    bool first = true;
    for_each_topic(topics, [&](auto &topic) {
        if (!topic.spec.in_state)
            return;
        if (!first)
            out += ',';
        first = false;
        append_entry(out, topic, now, stale_sec);
    });

    out += "}}}";
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
