#ifndef RS485_COM_HPP
#define RS485_COM_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class DDSM115;

class RS485Com
{
public:
    static constexpr std::size_t FRAME_SIZE = 10;

    explicit RS485Com(
        const std::string& device = "/dev/ddsm_rs485",
        int baudRate = 115200);

    ~RS485Com();

    bool init();
    void update();

    bool registerMotor(DDSM115* motor);

private:
    static constexpr auto RESPONSE_TIMEOUT =
        std::chrono::milliseconds(10);

    static constexpr auto MIN_SEND_INTERVAL =
        std::chrono::milliseconds(2);

    // Serial interface
    std::string device_;
    int baudRate_;
    int serialFd_;

    // Registered motors
    std::vector<DDSM115*> motors_;
    std::size_t nextMotorIndex_;

    // Current request/response transaction
    bool waitingForResponse_;
    DDSM115* awaitingMotor_;

    std::chrono::steady_clock::time_point requestSentAt_;
    std::chrono::steady_clock::time_point lastSendAt_;

    // Receive buffer
    uint8_t rxBuffer_[FRAME_SIZE];
    std::size_t rxIndex_;

    void sendNextMotor();

    bool sendFrame(
        DDSM115* motor,
        const uint8_t* frame);

    void receive();
    void dispatchResponse();

    DDSM115* findMotorById(uint8_t id);
};

#endif