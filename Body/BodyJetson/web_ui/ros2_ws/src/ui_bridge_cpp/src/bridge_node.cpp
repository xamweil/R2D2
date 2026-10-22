#include "bridge_node.hpp"

#include <rclcpp/node_options.hpp>

namespace ui_bridge {

BridgeNode::BridgeNode(TopicRegistry &topics,
                       const rclcpp::NodeOptions &options)
    : Node("ui_bridge_cpp", options) {

    for_each_topic(topics, [this](auto &topic) {
        using Msg = typename std::decay_t<decltype(topic)>::Type;
        topic.sub = create_subscription<Msg>(
            topic.spec.ros_topic, topic.spec.qos,
            [&topic](const typename Msg::ConstSharedPtr &msg) {
                topic.slot.store(msg);
            });
    });
}

} // namespace ui_bridge
