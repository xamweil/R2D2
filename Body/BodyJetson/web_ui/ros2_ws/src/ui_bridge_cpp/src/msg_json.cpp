#include "msg_json.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace ui_bridge {

namespace {

using introspection::MessageMember;

template <class T> void append_number(std::string &out, T v) {
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(v)) {
            out += "null";
            return;
        }
    }
    constexpr size_t NUMBER_CHARS = 32;
    std::array<char, NUMBER_CHARS> buf{};
    auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    out.append(buf.data(), res.ptr);
}

void append_escape(std::string &out, unsigned code) {
    std::array<char, 8> buf{};
    int n = std::snprintf(buf.data(), buf.size(), "\\u%04x", code);
    out.append(buf.data(), static_cast<size_t>(n));
}

void append_string(std::string &out, const std::string &s) {
    out += '"';
    for (char c : s) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20)
                append_escape(out, static_cast<unsigned char>(c));
            else
                out += c;
        }
    }
    out += '"';
}

void append_wstring(std::string &out, const std::u16string &s) {
    // UTF-16 code units map directly to JSON \u escapes.
    out += '"';
    for (char16_t c : s)
        append_escape(out, c);
    out += '"';
}

// Writes one non-array value of the member's type located at field.
void append_value(std::string &out, const MessageMember &m, const void *field) {
    namespace ft = introspection;
    // Type ids and their C++ types: rosidl_typesupport_introspection_cpp/
    // field_types.hpp (values in rosidl_typesupport_introspection_c/
    // field_types.h). The FLOAT32/FLOAT64/BOOL/BYTE constants there are
    // aliases of ids below.
    switch (m.type_id_) {
    case ft::ROS_TYPE_FLOAT: // 1, float
        append_number(out, *static_cast<const float *>(field));
        break;
    case ft::ROS_TYPE_DOUBLE: // 2, double
        append_number(out, *static_cast<const double *>(field));
        break;
    case ft::ROS_TYPE_LONG_DOUBLE: // 3, long double
        append_number(out, static_cast<double>(
                               *static_cast<const long double *>(field)));
        break;
    case ft::ROS_TYPE_CHAR: // 4, unsigned char
    case ft::ROS_TYPE_OCTET: // 7, byte (generated as unsigned char)
    case ft::ROS_TYPE_UINT8: // 8, uint8_t
        append_number(out, *static_cast<const uint8_t *>(field));
        break;
    case ft::ROS_TYPE_WCHAR: // 5, char16_t
        append_number(out, static_cast<uint16_t>(
                               *static_cast<const char16_t *>(field)));
        break;
    case ft::ROS_TYPE_BOOLEAN: // 6, bool
        out += *static_cast<const bool *>(field) ? "true" : "false";
        break;
    case ft::ROS_TYPE_INT8: // 9, int8_t
        append_number(out, *static_cast<const int8_t *>(field));
        break;
    case ft::ROS_TYPE_UINT16: // 10, uint16_t
        append_number(out, *static_cast<const uint16_t *>(field));
        break;
    case ft::ROS_TYPE_INT16: // 11, int16_t
        append_number(out, *static_cast<const int16_t *>(field));
        break;
    case ft::ROS_TYPE_UINT32: // 12, uint32_t
        append_number(out, *static_cast<const uint32_t *>(field));
        break;
    case ft::ROS_TYPE_INT32: // 13, int32_t
        append_number(out, *static_cast<const int32_t *>(field));
        break;
    case ft::ROS_TYPE_UINT64: // 14, uint64_t
        append_number(out, *static_cast<const uint64_t *>(field));
        break;
    case ft::ROS_TYPE_INT64: // 15, int64_t
        append_number(out, *static_cast<const int64_t *>(field));
        break;
    case ft::ROS_TYPE_STRING: // 16, std::string
        append_string(out, *static_cast<const std::string *>(field));
        break;
    case ft::ROS_TYPE_WSTRING: // 17, std::u16string
        append_wstring(out, *static_cast<const std::u16string *>(field));
        break;
    case ft::ROS_TYPE_MESSAGE: // 18, nested msg, members_->data = its members
        write_message_json(
            *static_cast<const introspection::MessageMembers *>(
                m.members_->data),
            field, out);
        break;
    default:
        out += "null";
    }
}

bool is_byte_type(uint8_t type_id) {
    return type_id == introspection::ROS_TYPE_UINT8 ||
           type_id == introspection::ROS_TYPE_OCTET;
}

void append_array(std::string &out, const MessageMember &m,
                  const void *field) {
    size_t size = m.size_function ? m.size_function(field) : m.array_size_;

    // Fixed size arrays are small; dynamic byte arrays are blobs.
    bool is_dynamic = m.array_size_ == 0 || m.is_upper_bound_;
    if (is_dynamic && is_byte_type(m.type_id_)) {
        out += R"({"_bytes_len":)";
        append_number(out, size);
        out += '}';
        return;
    }

    out += '[';
    for (size_t i = 0; i < size; ++i) {
        if (i != 0)
            out += ',';
        if (m.get_const_function) {
            append_value(out, m, m.get_const_function(field, i));
        } else {
            // std::vector<bool> has no element pointers, only fetch.
            bool value = false;
            m.fetch_function(field, i, &value);
            append_value(out, m, &value);
        }
    }
    out += ']';
}

} // namespace

void write_message_json(const introspection::MessageMembers &members,
                        const void *msg, std::string &out) {
    out += '{';
    for (uint32_t i = 0; i < members.member_count_; ++i) {
        const MessageMember &m = members.members_[i];
        if (i != 0)
            out += ',';
        out += '"';
        out += m.name_;
        out += "\":";
        const void *field = static_cast<const uint8_t *>(msg) + m.offset_;
        if (m.is_array_)
            append_array(out, m, field);
        else
            append_value(out, m, field);
    }
    out += '}';
}

} // namespace ui_bridge
