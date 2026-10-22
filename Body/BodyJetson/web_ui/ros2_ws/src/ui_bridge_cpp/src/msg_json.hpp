#pragma once

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <stdexcept>
#include <string>

namespace ui_bridge {

namespace introspection = rosidl_typesupport_introspection_cpp;

// Appends any ROS message as JSON to out, using the type's introspection
// data. Field names and nesting follow the .msg definition. Dynamic uint8 /
// byte arrays (image data, ...) are written as {"_bytes_len": n} instead of
// their content, same as the Python ui_bridge.
void write_message_json(const introspection::MessageMembers &members,
                        const void *msg, std::string &out);

// Throws if the message package's introspection library can't be loaded.
template <class Msg>
const introspection::MessageMembers &message_members() {
    static const introspection::MessageMembers *members = [] {
        const auto *ts = get_message_typesupport_handle(
            rosidl_typesupport_cpp::get_message_type_support_handle<Msg>(),
            introspection::typesupport_identifier);
        if (!ts)
            throw std::runtime_error(
                std::string("no introspection typesupport for ") +
                rosidl_generator_traits::name<Msg>());
        return static_cast<const introspection::MessageMembers *>(ts->data);
    }();
    return *members;
}

template <class Msg> void write_json(const Msg &msg, std::string &out) {
    write_message_json(message_members<Msg>(), &msg, out);
}

} // namespace ui_bridge
