#include "drive_control/RS485Com.hpp"
#include "drive_control/DDSM115.hpp"

#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

RS485Com::RS485Com(
    const std::string& device,
    int baudRate)
    : device_(device),
      baudRate_(baudRate),
      serialFd_(-1),
      nextMotorIndex_(0),
      waitingForResponse_(false),
      awaitingMotor_(nullptr),
      rxIndex_(0),
      requestSentAt_(std::chrono::steady_clock::now()),
      lastSendAt_(std::chrono::steady_clock::now() - MIN_SEND_INTERVAL)
{
    std::memset(rxBuffer_, 0, FRAME_SIZE);
}

RS485Com::~RS485Com()
{
    if (serialFd_ >= 0)
    {
        close(serialFd_);
        serialFd_ = -1;
    }
}

bool RS485Com::init()
{
    serialFd_ = open(
        device_.c_str(),
        O_RDWR | O_NOCTTY);

    if (serialFd_ < 0)
    {
        std::cerr
            << "RS485 open failed for "
            << device_
            << ": "
            << std::strerror(errno)
            << std::endl;

        return false;
    }

    termios tty{};

    if (tcgetattr(serialFd_, &tty) != 0)
    {
        std::cerr
            << "RS485 tcgetattr failed: "
            << std::strerror(errno)
            << std::endl;

        close(serialFd_);
        serialFd_ = -1;

        return false;
    }

    speed_t speed;

    switch (baudRate_)
    {
        case 115200:
            speed = B115200;
            break;

        default:
            std::cerr
                << "Unsupported baud rate: "
                << baudRate_
                << std::endl;

            close(serialFd_);
            serialFd_ = -1;

            return false;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serialFd_, TCSANOW, &tty) != 0)
    {
        std::cerr
            << "RS485 tcsetattr failed: "
            << std::strerror(errno)
            << std::endl;

        close(serialFd_);
        serialFd_ = -1;

        return false;
    }

    if (tcflush(serialFd_, TCIOFLUSH) != 0)
    {
        std::cerr
            << "RS485 tcflush failed: "
            << std::strerror(errno)
            << std::endl;
    }

    std::cerr
        << "RS485 initialized on "
        << device_
        << " at "
        << baudRate_
        << " baud"
        << std::endl;

    return true;
}

// bool RS485Com::init()
// {
//     serialFd_ = open(
//         device_.c_str(),
//         O_RDWR | O_NOCTTY);

//     if (serialFd_ < 0)
//     {
//         return false;
//     }

//     termios tty{};

//     if (tcgetattr(serialFd_, &tty) != 0)
//     {
//         close(serialFd_);
//         serialFd_ = -1;
//         return false;
//     }

//     speed_t baud;

//     switch (baudRate_)
//     {
//         case 115200:
//             baud = B115200;
//             break;

//         default:
//             close(serialFd_);
//             serialFd_ = -1;
//             return false;
//     }

//     cfsetispeed(&tty, baud);
//     cfsetospeed(&tty, baud);

//     // 8 data bits
//     tty.c_cflag &= ~CSIZE;
//     tty.c_cflag |= CS8;

//     // No parity
//     tty.c_cflag &= ~PARENB;

//     // One stop bit
//     tty.c_cflag &= ~CSTOPB;

//     // No hardware flow control
//     tty.c_cflag &= ~CRTSCTS;

//     // Enable receiver and ignore modem control lines
//     tty.c_cflag |= CREAD | CLOCAL;

//     // Raw input
//     tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

//     // No software flow control
//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);

//     // Raw output
//     tty.c_oflag &= ~OPOST;

//     /*
//      * Non-blocking-style reads:
//      * read() immediately returns whatever is currently available.
//      */
//     tty.c_cc[VMIN] = 0;
//     tty.c_cc[VTIME] = 0;

//     if (tcsetattr(serialFd_, TCSANOW, &tty) != 0)
//     {
//         close(serialFd_);
//         serialFd_ = -1;
//         return false;
//     }

//     // Remove any garbage left in the USB/UART buffers.
//     tcflush(serialFd_, TCIOFLUSH);

//     return true;
// }

bool RS485Com::registerMotor(DDSM115* motor)
{
    if (motor == nullptr)
    {
        return false;
    }

    for (const auto* registeredMotor : motors_)
    {
        if (registeredMotor->id() == motor->id())
        {
            return false;
        }
    }

    motors_.push_back(motor);

    return true;
}

DDSM115* RS485Com::findMotorById(uint8_t id)
{
    for (auto* motor : motors_)
    {
        if (motor != nullptr && motor->id() == id)
        {
            return motor;
        }
    }

    return nullptr;
}

bool RS485Com::sendFrame(
    DDSM115* motor,
    const uint8_t* frame)
{
    if (serialFd_ < 0 ||
        motor == nullptr ||
        frame == nullptr)
    {
        return false;
    }

    const ssize_t bytesWritten =
        write(serialFd_, frame, FRAME_SIZE);

    if (bytesWritten != static_cast<ssize_t>(FRAME_SIZE))
    {
        return false;
    }

    /*
     * Wait until the kernel has actually transmitted the bytes.
     * At 115200 baud a 10-byte frame takes less than 1 ms.
     */
    if (tcdrain(serialFd_) != 0)
    {
        return false;
    }

    lastSendAt_ = std::chrono::steady_clock::now();

    return true;
}

void RS485Com::sendNextMotor()
{
    if (motors_.empty())
    {
        return;
    }

    DDSM115* motor = motors_[nextMotorIndex_];

    if (motor == nullptr)
    {
        nextMotorIndex_ =
            (nextMotorIndex_ + 1) % motors_.size();

        return;
    }

    /*
     * Mode changes ALWAYS take priority.
     * 0xA0 mode-switch commands do NOT return feedback.
     */
    if (motor->hasPendingModeChange())
    {
        if (sendFrame(motor, motor->modeFrame()))
        {
            motor->markModeChangeSent();

            /*
             * No nextMotorIndex_, since only mode was set.
             * The next frame sent will therefore be this same
             * motor's latest drive command.
             */
        }

        return;
    }

    /*
     * Normal 0x64 drive command.
     * This DOES return a 10-byte feedback frame.
     */
    if (sendFrame(motor, motor->driveFrame()))
    {
        waitingForResponse_ = true;
        awaitingMotor_ = motor;
        requestSentAt_ = std::chrono::steady_clock::now();
        rxIndex_ = 0;
    }
}

void RS485Com::receive()
{
    if (!waitingForResponse_ || serialFd_ < 0)
    {
        return;
    }

    const ssize_t bytesRead = read(
        serialFd_,
        rxBuffer_ + rxIndex_,
        FRAME_SIZE - rxIndex_);

    if (bytesRead < 0)
    {
        return;
    }

    if (bytesRead == 0)
    {
        return;
    }

    rxIndex_ += static_cast<std::size_t>(bytesRead);

    if (rxIndex_ == FRAME_SIZE)
    {
        dispatchResponse();
    }
}

void RS485Com::dispatchResponse()
{
    if (!waitingForResponse_ ||
        awaitingMotor_ == nullptr)
    {
        rxIndex_ = 0;
        return;
    }

    const bool valid =
        awaitingMotor_->processFeedback(
            rxBuffer_,
            FRAME_SIZE);

    if (!valid)
    {
        /*
         * Don't claim the transaction succeeded.
         * Let the normal response timeout recover the bus.
         */
        rxIndex_ = 0;
        return;
    }

    waitingForResponse_ = false;
    awaitingMotor_ = nullptr;
    rxIndex_ = 0;

    nextMotorIndex_ = (nextMotorIndex_ + 1) % motors_.size();
}

void RS485Com::update()
{
    const auto now =
        std::chrono::steady_clock::now();

    /*
     * Always service incoming data first.
     */
    receive();

    /*
     * If waiting for a drive response,
     * don't transmit anything else.
     */
    if (waitingForResponse_)
    {
        if (now - requestSentAt_ >= RESPONSE_TIMEOUT)
        {
            waitingForResponse_ = false;
            awaitingMotor_ = nullptr;
            rxIndex_ = 0;

            // Drop any incomplete/stale received data.
            tcflush(serialFd_, TCIFLUSH);

            if (!motors_.empty())
            {
                nextMotorIndex_ =
                    (nextMotorIndex_ + 1) % motors_.size();
            }
        }

        return;
    }

    /*
     * Account for DDSM115 maximum communication rate (500Hz).
     */
    if (now - lastSendAt_ < MIN_SEND_INTERVAL)
    {
        return;
    }

    sendNextMotor();
}