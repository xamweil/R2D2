#pragma once

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <memory>
#include <string>

namespace ui_bridge_ng::json {

inline bool parse(const std::string &input, rapidjson::Document &doc) {
    doc.Parse(input.c_str(), input.size());
    return !doc.HasParseError();
}

inline std::string stringify(const rapidjson::Value &value) {
    rapidjson::StringBuffer buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
    value.Accept(writer);
    return {buf.GetString(), buf.GetSize()};
}

inline std::shared_ptr<const std::string>
stringify_shared(const rapidjson::Value &value) {
    return std::make_shared<const std::string>(stringify(value));
}

} // namespace ui_bridge_ng::json
