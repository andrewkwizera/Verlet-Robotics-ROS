#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{

constexpr uint32_t REGISTER_CAN_ID = 0x7FF;
constexpr uint8_t CMD_READ = 0x33;

constexpr int TIMEOUT_MS = 150;
constexpr int MAX_RETRIES = 5;

class CANReader
{
public:
    explicit CANReader(const std::string &channel)
    {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (fd_ < 0)
            throw std::runtime_error("Cannot open CAN socket");

        try
        {
            ifreq ifr{};
            std::strncpy(
                ifr.ifr_name,
                channel.c_str(),
                IFNAMSIZ - 1
            );

            if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
                throw std::runtime_error(
                    "CAN interface not found");

            sockaddr_can addr{};
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(
                    fd_,
                    reinterpret_cast<sockaddr *>(&addr),
                    sizeof(addr)) < 0)
            {
                throw std::runtime_error(
                    "Cannot bind CAN socket");
            }

            // Receive all CAN frames for diagnosis.
            // No restrictive response-ID filter.
        }
        catch (...)
        {
            close(fd_);
            fd_ = -1;
            throw;
        }
    }

    ~CANReader()
    {
        if (fd_ >= 0)
            close(fd_);
    }

    CANReader(const CANReader &) = delete;
    CANReader &operator=(const CANReader &) = delete;

    float read_register(
        uint8_t motor_id,
        uint8_t register_address)
    {
        for (int attempt = 1;
             attempt <= MAX_RETRIES;
             ++attempt)
        {
            // Drain previously queued frames.
            while (true)
            {
                can_frame stale{};
                ssize_t n = recv(
                    fd_,
                    &stale,
                    sizeof(stale),
                    MSG_DONTWAIT);

                if (n < 0)
                {
                    if (errno == EINTR)
                        continue;

                    if (errno == EAGAIN ||
                        errno == EWOULDBLOCK)
                        break;

                    throw std::runtime_error(
                        "CAN receive/drain failed");
                }

                if (n == 0)
                    break;
            }

            can_frame request{};
            request.can_id = REGISTER_CAN_ID;
            request.can_dlc = 8;

            request.data[0] = motor_id;
            request.data[1] = 0x00;
            request.data[2] = CMD_READ;
            request.data[3] = register_address;

            std::cout
                << "TX motor=" << int(motor_id)
                << " reg=" << int(register_address)
                << " attempt=" << attempt
                << std::endl;

            if (write(
                    fd_,
                    &request,
                    sizeof(request)) != sizeof(request))
            {
                throw std::runtime_error(
                    "CAN transmission failed");
            }

            auto deadline =
                std::chrono::steady_clock::now() +
                std::chrono::milliseconds(TIMEOUT_MS);

            while (std::chrono::steady_clock::now() <
                   deadline)
            {
                auto remaining =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds>(
                        deadline -
                        std::chrono::steady_clock::now()
                    ).count();

                pollfd pfd{};
                pfd.fd = fd_;
                pfd.events = POLLIN;

                int result = poll(
                    &pfd,
                    1,
                    static_cast<int>(
                        remaining > 0 ? remaining : 1));

                if (result == 0)
                    break;

                if (result < 0)
                {
                    if (errno == EINTR)
                        continue;

                    throw std::runtime_error(
                        "CAN poll failed");
                }

                if (pfd.revents &
                    (POLLERR | POLLHUP | POLLNVAL))
                {
                    throw std::runtime_error(
                        "CAN socket error");
                }

                if (!(pfd.revents & POLLIN))
                    continue;

                can_frame frame{};

                ssize_t n = recv(
                    fd_, &frame, sizeof(frame), 0);

                if (n != sizeof(frame))
                    continue;

                std::cout
                    << "  RX id=0x"
                    << std::hex
                    << (frame.can_id & CAN_SFF_MASK)
                    << std::dec
                    << " dlc=" << int(frame.can_dlc)
                    << " data=";

                for (int i = 0; i < frame.can_dlc; ++i)
                {
                    std::cout
                        << std::hex
                        << std::setw(2)
                        << std::setfill('0')
                        << int(frame.data[i]) << " ";
                }

                std::cout
                    << std::dec
                    << std::setfill(' ')
                    << std::endl;

                if ((frame.can_id &
                     (CAN_EFF_FLAG | CAN_RTR_FLAG |
                      CAN_ERR_FLAG)) != 0)
                    continue;

                if (frame.can_dlc != 8)
                    continue;

                // Keep I2RT's motor/register echo checks.
                if (frame.data[0] != motor_id ||
                    frame.data[3] != register_address)
                    continue;

                // Decode little-endian IEEE754 float.
                uint32_t bits =
                    uint32_t(frame.data[4]) |
                    (uint32_t(frame.data[5]) << 8) |
                    (uint32_t(frame.data[6]) << 16) |
                    (uint32_t(frame.data[7]) << 24);

                float value;
                std::memcpy(
                    &value, &bits, sizeof(value));

                if (!std::isfinite(value))
                    throw std::runtime_error(
                        "Non-finite register value");

                return value;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(10));
        }

        throw std::runtime_error(
            "No matching response after retries");
    }

private:
    int fd_ = -1;
};

} // namespace

int main(int argc, char **argv)
{
    std::string channel =
        argc > 1 ? argv[1] : "can1";

    if (channel != "can0" && channel != "can1")
    {
        std::cerr << "Use can0 or can1\n";
        return 1;
    }

    try
    {
        CANReader can(channel);

        std::cout
            << "\nYAM C++ REGISTER DIAGNOSTIC\n"
            << "Interface: " << channel << "\n"
            << "Read-only register requests\n"
            << "No motor enable or movement commands\n\n";

        // Test one already verified motor first.
        // Gr=40, p_m and xout should be readable.
        const std::array<std::pair<int, int>, 18> tests = {{
            {1, 20}, {1, 80}, {1, 81},
            {2, 20}, {2, 80}, {2, 81},
            {3, 20}, {3, 80}, {3, 81},
            {4, 20}, {4, 80}, {4, 81},
            {5, 20}, {5, 80}, {5, 81},
            {6, 20}, {6, 80}, {6, 81}
        }};

        bool all_passed = true;

        for (const auto &[motor, reg] : tests)
        {
            try
            {
                float value = can.read_register(
                    static_cast<uint8_t>(motor),
                    static_cast<uint8_t>(reg));

                std::cout
                    << "SUCCESS motor=" << motor
                    << " register=" << reg
                    << " value=" << value
                    << "\n\n";
            }
            catch (const std::exception &e)
            {
                all_passed = false;

                std::cerr
                    << "FAILED motor=" << motor
                    << " register=" << reg
                    << " reason=" << e.what()
                    << "\n\n";
            }
        }

        return all_passed ? 0 : 1;
    }
    catch (const std::exception &e)
    {
        std::cerr
            << "Fatal: " << e.what() << "\n";

        return 1;
    }
}
