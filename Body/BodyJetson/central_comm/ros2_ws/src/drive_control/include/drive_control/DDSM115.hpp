#ifndef DDSM_115_HPP
#define DDSM_115_HPP

#include <cstddef>
#include <cstdint>

class DDSM115
{
public:
    static constexpr std::size_t FRAME_SIZE = 10;

    // Motor operating modes
    static constexpr uint8_t MODE_CURRENT_LOOP  = 0x01;
    static constexpr uint8_t MODE_SPEED_LOOP    = 0x02;
    static constexpr uint8_t MODE_POSITION_LOOP = 0x03;

    // Protocol commands
    static constexpr uint8_t CMD_DRIVE    = 0x64;
    static constexpr uint8_t CMD_MODE     = 0xA0;
    static constexpr uint8_t CMD_FEEDBACK = 0x74;

    // Brake values
    static constexpr uint8_t BRAKE_ON  = 0xFF;
    static constexpr uint8_t BRAKE_OFF = 0x00;

    // Maximum ratings
    static constexpr int16_t MAX_RPM = 330;
    //static constexpr int16_t MAX_CURRENT = ;

    explicit DDSM115(uint8_t id);

    void init();

    bool setVelocity(int16_t rpm);
    bool setPosition(float angleDeg);
    bool setCurrent(int16_t current);

    void brakeOn();
    void brakeOff();
    void stop();

    bool setMode(uint8_t mode);

    // Called by RS485Com when a response for this motor arrives
    bool processFeedback(const uint8_t* data, std::size_t length);
    bool hasPendingModeChange() const;

    const uint8_t* driveFrame() const;
    const uint8_t* modeFrame() const;

    void markModeChangeSent();

    // Motor information
    uint8_t id() const;
    uint8_t mode() const;

    // Latest feedback values
    int16_t velocity() const;
    float current() const;
    float position() const;
    uint8_t errorCode() const;

private:
    uint8_t id_;
    uint8_t mode_;

    uint8_t driveFrame_[FRAME_SIZE];
    uint8_t modeFrame_[FRAME_SIZE];

    bool modeChangePending_;

    int16_t velocity_;
    int16_t current_;
    uint16_t position_;
    uint8_t errorCode_;

    void buildVelocityCommand(int16_t rpm);
    void buildPositionCommand(float angleDeg);
    void buildCurrentCommand(int16_t current);
    void buildBrakeCommand(bool enabled);
    void buildModeCommand(uint8_t mode);

    static uint8_t crc8(const uint8_t* data, std::size_t length);
};

#endif