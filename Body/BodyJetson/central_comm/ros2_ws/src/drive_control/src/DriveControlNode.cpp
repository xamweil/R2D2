#include "drive_control/DriveControlNode.hpp"

#include <chrono>
#include <stdexcept>
#include <functional>
#include <string>

namespace DriveControl
{

DriveControlNode::DriveControlNode()
    : Node("drive_control_node")
{
    // -------------------------------------------------------------------------
    // Initialize RS485 interface
    // -------------------------------------------------------------------------

    if (!rs485_.init())
    {
        throw std::runtime_error(
            "Failed to initialize RS485 communication");
    }

    // -------------------------------------------------------------------------
    // Create and register installed motors
    // -------------------------------------------------------------------------

    for (const auto& config : DRIVE_MOTOR_CONFIGS)
    {
        auto motor = std::make_unique<DriveMotor>(config);

        motor->init();

        if (!rs485_.registerMotor(&motor->ddsm()))
        {
            throw std::runtime_error(
                "Failed to register motor ID " +
                std::to_string(config.motorId));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Registered motor '%s' (ROS ID: %u, RS485 ID: %u)",
            config.name,
            config.commandId,
            config.motorId);

        motors_.push_back(std::move(motor));
    }

    // -------------------------------------------------------------------------
    // Motor Feedback Publisher
    // -------------------------------------------------------------------------    

    for (auto& motor : motors_)
    {
        const std::string topic =
            "/ddsm115_feedback/" +
            std::string(motor->name()) +
            "_" +
            std::to_string(motor->commandId());

        auto publisher =
            this->create_publisher<
                serial_msg::msg::DDSM115Feedback>(
                    topic,
                    10);

        feedbackPublishers_.push_back(
            {motor.get(), publisher});

        RCLCPP_INFO(
            this->get_logger(),
            "Created feedback publisher: %s",
            topic.c_str());
    }

    // -------------------------------------------------------------------------
    // Motor command subscriber
    // -------------------------------------------------------------------------

    motorCommandSub_ =
        this->create_subscription<serial_msg::msg::MotorCommand>(
            "/motor_command",
            10,
            std::bind(
                &DriveControlNode::motorCommandCallback,
                this,
                std::placeholders::_1));

    // -------------------------------------------------------------------------
    // RS485 communication timer
    //
    // Run faster than the actual bus rate.
    // RS485Com internally limits transmission to the allowed interval.
    // -------------------------------------------------------------------------

    communicationTimer_ = this->create_wall_timer(std::chrono::milliseconds(1),
            std::bind(&DriveControlNode::communicationUpdate, this));
    
    // -------------------------------------------------------------------------
    // Motor feedback publishing timer - 50 Hz
    // -------------------------------------------------------------------------

    feedbackTimer_ = this->create_wall_timer(std::chrono::milliseconds(20),
            std::bind(&DriveControlNode::publishFeedback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "DriveControlNode initialized with %zu motors",
        motors_.size());
}


// =============================================================================
// ROS MotorCommand callback
// =============================================================================

void DriveControlNode::motorCommandCallback(
    const serial_msg::msg::MotorCommand::SharedPtr msg)
{
    if (!validateMotorCommand(*msg))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Received malformed MotorCommand: array sizes do not match");

        return;
    }

    for (std::size_t i = 0; i < msg->ids.size(); ++i)
    {
        DriveMotor* motor =
            findMotorByCommandId(msg->ids[i]);

        if (motor == nullptr)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received command for unknown drive motor ID %u",
                msg->ids[i]);

            continue;
        }

        const bool enable =
            msg->enable[i];

        const bool angleSet =
            msg->angle_set[i];

        const bool velocitySet =
            msg->velocity_set[i];


        // ---------------------------------------------------------------------
        // Disabled motor -> brake
        // ---------------------------------------------------------------------

        if (!enable)
        {
            motor->brakeOn();
            continue;
        }


        // ---------------------------------------------------------------------
        // Enabled, but no target supplied -> release brake
        // ---------------------------------------------------------------------

        if (!angleSet && !velocitySet)
        {
            motor->brakeOff();
            continue;
        }


        // ---------------------------------------------------------------------
        // Velocity command
        // ---------------------------------------------------------------------

        if (velocitySet && !angleSet)
        {
            if (motor->ddsm().mode() != DDSM115::MODE_SPEED_LOOP)
            {
                if (!motor->setMode(DDSM115::MODE_SPEED_LOOP))
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to set motor %u to speed mode",
                        motor->commandId());

                    continue;
                }
            }

            int16_t rpm =
                static_cast<int16_t>(msg->velocity[i]);

            if (msg->direction[i])
            {
                rpm = -rpm;
            }

            if (!motor->setVelocity(rpm))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to set velocity for motor %u",
                    motor->commandId());
            }

            continue;
        }


        // ---------------------------------------------------------------------
        // Position command
        // ---------------------------------------------------------------------

        if (angleSet && !velocitySet)
        {
            if (motor->ddsm().mode() != DDSM115::MODE_POSITION_LOOP)
            {
                if (!motor->setMode(DDSM115::MODE_POSITION_LOOP))
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to set motor %u to position mode",
                        motor->commandId());

                    continue;
                }
            }

            if (!motor->setPosition(msg->angle[i]))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to set position for motor %u",
                    motor->commandId());
            }

            continue;
        }


        // ---------------------------------------------------------------------
        // angle_set + velocity_set
        //
        // Reserved for explicit DDSM115 0x74 feedback request.
        // ---------------------------------------------------------------------

        if (angleSet && velocitySet)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Special feedback request for motor %u is not implemented yet",
                motor->commandId());

            // TODO:
            // Implement DDSM115 CMD_FEEDBACK (0x74) request.
            //
            // This combination is intentionally reserved for it:
            //
            // angle_set    = true
            // velocity_set = true

            continue;
        }
    }
}


// =============================================================================
// RS485 update
// =============================================================================

void DriveControlNode::communicationUpdate()
{
    rs485_.update();
}


// =============================================================================
// Motor lookup
// =============================================================================

DriveMotor* DriveControlNode::findMotorByCommandId(
    uint8_t commandId)
{
    for (auto& motor : motors_)
    {
        if (motor->commandId() == commandId)
        {
            return motor.get();
        }
    }

    return nullptr;
}


// =============================================================================
// MotorCommand validation
// =============================================================================

bool DriveControlNode::validateMotorCommand(
    const serial_msg::msg::MotorCommand& msg) const
{
    const std::size_t size = msg.ids.size();

    return
        msg.enable.size()       == size &&
        msg.direction.size()    == size &&
        msg.angle_set.size()    == size &&
        msg.velocity_set.size() == size &&
        msg.angle.size()        == size &&
        msg.velocity.size()     == size;
}

// =============================================================================
// Publish motor feedback
// =============================================================================

void DriveControlNode::publishFeedback()
{
    for (auto& entry : feedbackPublishers_)
    {
        serial_msg::msg::DDSM115Feedback msg;

        msg.id =
            entry.motor->commandId();

        msg.mode =
            entry.motor->ddsm().mode();

        msg.velocity =
            entry.motor->ddsm().velocity();

        msg.current =
            entry.motor->ddsm().current();

        msg.position =
            entry.motor->ddsm().position();

        msg.error_code =
            entry.motor->ddsm().errorCode();

        entry.publisher->publish(msg);
    }
}

} // namespace DriveControl