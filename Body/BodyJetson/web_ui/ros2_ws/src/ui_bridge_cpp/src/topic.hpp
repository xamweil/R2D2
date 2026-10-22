#pragma once

#include "msg_json.hpp"
#include "slot.hpp"

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace ui_bridge {

// Appends the JSON for one message to out.
template <class Msg> using Serializer = void (*)(const Msg &, std::string &out);

// One entry of the topic list. Only the static description; runtime state
// lives in Topic.
template <class Msg> struct TopicSpec {
    const char *key{};       // name in robot_state.payload.entries
    const char *ros_topic{}; // ROS topic to subscribe to
    Serializer<Msg> serialize = &write_json<Msg>;
    bool in_state = true; // false: subscribe only, not sent in robot_state
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
};

template <class Msg> struct Topic {
    using Type = Msg;

    explicit Topic(const TopicSpec<Msg> &s) : spec(s) {
        // Resolve now so a missing typesupport fails at startup instead of
        // in the first state broadcast on the http thread.
        if (spec.serialize == &write_json<Msg>)
            message_members<Msg>();
    }

    TopicSpec<Msg> spec;
    Slot<Msg> slot;
    typename rclcpp::Subscription<Msg>::SharedPtr sub;

    // Serialization cache, only touched by the http thread.
    uint64_t seen_generation = 0;
    std::string cached_data;
};

// Topics are constructed in place (Slot holds a mutex and can't move), so the
// registry must be initialized directly from make_registry's return value.
// https://en.cppreference.com/cpp/language/pack
// https://en.cppreference.com/cpp/language/fold
// https://en.cppreference.com/cpp/language/copy_elision
template <class... Msgs>
std::tuple<Topic<Msgs>...> make_registry(const TopicSpec<Msgs> &...specs) {
    return std::tuple<Topic<Msgs>...>(specs...);
}

// https://en.cppreference.com/cpp/utility/apply
// Calls f(topic) for every topic in the registry, in declaration order. The
// topics have different types, so f must be a generic lambda (auto &t). This
// is a compile-time unrolled loop: std::apply unpacks the tuple into a pack t,
// and the fold (f(t), ...) expands to f(t0), f(t1), ..., f(tN).
template <class Registry, class F> void for_each_topic(Registry &reg, F f) {
    std::apply([&f](auto &...t) { (f(t), ...); }, reg);
}

// Look up a topic by key and message type. Meant for one-time use at startup
// by code that needs direct access to a slot (e.g. the MJPEG stream).
template <class Msg, class Registry>
Topic<Msg> &find_topic(Registry &reg, std::string_view key) {
    Topic<Msg> *found = nullptr;
    for_each_topic(reg, [&](auto &t) {
        using T = std::decay_t<decltype(t)>;
        if constexpr (std::is_same_v<typename T::Type, Msg>) {
            if (key == t.spec.key)
                found = &t;
        }
    });
    if (!found)
        throw std::runtime_error("topic not found: " + std::string(key));
    return *found;
}

} // namespace ui_bridge
