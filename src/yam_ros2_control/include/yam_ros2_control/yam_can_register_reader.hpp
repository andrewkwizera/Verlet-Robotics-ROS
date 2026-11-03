
#ifndef YAM_ROS2_CONTROL__YAM_CAN_REGISTER_READER_HPP_
#define YAM_ROS2_CONTROL__YAM_CAN_REGISTER_READER_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cerrno>
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

namespace yam_ros2_control
{

    class YAMCanRegisterReader
    {
    public:
        explicit YAMCanRegisterReader(
            const std::string &channel)
        {
            fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

            if (fd_ < 0)
            {
                throw std::runtime_error(
                    "Cannot open SocketCAN socket");
            }

            try
            {
                ifreq ifr{};

                std::strncpy(
                    ifr.ifr_name,
                    channel.c_str(),
                    IFNAMSIZ - 1);

                if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
                {
                    throw std::runtime_error(
                        "CAN interface not found: " + channel);
                }

                // Receive response IDs 0x10-0x1F.
                // Real motor responses are 0x11-0x16.
                can_filter filter{};

                filter.can_id = 0x10;
                filter.can_mask =
                    0x7F0 | CAN_EFF_FLAG | CAN_RTR_FLAG;

                if (setsockopt(
                        fd_,
                        SOL_CAN_RAW,
                        CAN_RAW_FILTER,
                        &filter,
                        sizeof(filter)) < 0)
                {
                    throw std::runtime_error(
                        "Cannot set CAN response filter");
                }

                sockaddr_can address{};
                address.can_family = AF_CAN;
                address.can_ifindex = ifr.ifr_ifindex;

                if (bind(
                        fd_,
                        reinterpret_cast<sockaddr *>(&address),
                        sizeof(address)) < 0)
                {
                    throw std::runtime_error(
                        "Cannot bind SocketCAN interface");
                }
            }
            catch (...)
            {
                close(fd_);
                fd_ = -1;
                throw;
            }
        }

        ~YAMCanRegisterReader()
        {
            if (fd_ >= 0)
            {
                close(fd_);
            }
        }

        YAMCanRegisterReader(
            const YAMCanRegisterReader &) = delete;

        YAMCanRegisterReader &operator=(
            const YAMCanRegisterReader &) = delete;

        float read_float_register(
            uint8_t motor_id,
            uint8_t register_address)
        {
            if (motor_id < 1 || motor_id > 6)
            {
                throw std::runtime_error(
                    "Invalid YAM motor ID");
            }

            for (int attempt = 0; attempt < 3; ++attempt)
            {
                drain_frames();

                can_frame request{};

                request.can_id = 0x7FF;
                request.can_dlc = 8;

                request.data[0] = motor_id;
                request.data[1] = 0x00;
                request.data[2] = 0x33;
                request.data[3] = register_address;

                const ssize_t sent = ::write(
                    fd_,
                    &request,
                    sizeof(request));

                if (sent != sizeof(request))
                {
                    throw std::runtime_error(
                        "CAN register request transmission failed");
                }

                const auto deadline =
                    std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(30);

                while (std::chrono::steady_clock::now() <
                       deadline)
                {
                    const auto remaining =
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            deadline -
                            std::chrono::steady_clock::now())
                            .count();

                    pollfd pfd{};

                    pfd.fd = fd_;
                    pfd.events = POLLIN;

                    const int result = poll(
                        &pfd,
                        1,
                        static_cast<int>(
                            remaining > 0 ? remaining : 1));

                    if (result == 0)
                    {
                        break;
                    }

                    if (result < 0)
                    {
                        if (errno == EINTR)
                        {
                            continue;
                        }

                        throw std::runtime_error(
                            "CAN register response polling failed");
                    }

                    if (pfd.revents &
                        (POLLERR | POLLHUP | POLLNVAL))
                    {
                        throw std::runtime_error(
                            "CAN socket reported an error");
                    }

                    if (!(pfd.revents & POLLIN))
                    {
                        continue;
                    }

                    can_frame response{};

                    const ssize_t received = recv(
                        fd_,
                        &response,
                        sizeof(response),
                        0);

                    if (received != sizeof(response))
                    {
                        continue;
                    }

                    if (response.can_id !=
                        static_cast<canid_t>(0x10 + motor_id))
                    {
                        continue;
                    }

                    if (response.can_dlc != 8)
                    {
                        continue;
                    }

                    if (response.data[0] != motor_id ||
                        response.data[2] != 0x33 ||
                        response.data[3] != register_address)
                    {
                        continue;
                    }

                    const uint32_t bits =
                        uint32_t(response.data[4]) |
                        (uint32_t(response.data[5]) << 8) |
                        (uint32_t(response.data[6]) << 16) |
                        (uint32_t(response.data[7]) << 24);

                    float value = 0.0f;

                    std::memcpy(
                        &value,
                        &bits,
                        sizeof(value));

                    if (!std::isfinite(value))
                    {
                        throw std::runtime_error(
                            "Non-finite motor register value");
                    }

                    return value;
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(2));
            }

            throw std::runtime_error(
                "Motor " + std::to_string(motor_id) +
                " register " + std::to_string(register_address) +
                " did not respond");
        }

    private:
        void drain_frames()
        {
            while (true)
            {
                can_frame frame{};

                const ssize_t result = recv(
                    fd_,
                    &frame,
                    sizeof(frame),
                    MSG_DONTWAIT);

                if (result == sizeof(frame))
                {
                    continue;
                }

                if (result < 0 && errno == EINTR)
                {
                    continue;
                }

                if (result < 0 &&
                    (errno == EAGAIN || errno == EWOULDBLOCK))
                {
                    return;
                }

                if (result == 0)
                {
                    return;
                }

                throw std::runtime_error(
                    "Failed to drain CAN receive queue");
            }
        }

        int fd_ = -1;
    };

} // namespace yam_ros2_control

#endif