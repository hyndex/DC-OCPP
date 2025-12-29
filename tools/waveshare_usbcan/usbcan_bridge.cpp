// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "controlcan.h"

namespace {
std::atomic<bool> running{true};

void handle_signal(int) {
    running = false;
}

struct Options {
    std::string iface{"can0"};
    int device_index{0};
    int channel_index{0};
    int bitrate{125000};
    int timing0{-1};
    int timing1{-1};
    int poll_ms{1};
    bool verbose{false};
};

void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  --iface <name>      SocketCAN iface to bridge (default: can0)\n"
        << "  --device <index>    USB-CAN-B device index (default: 0)\n"
        << "  --channel <index>   CAN channel index 0/1 (default: 0)\n"
        << "  --bitrate <bps>     CAN bitrate (default: 125000)\n"
        << "  --timing0 <value>   Override Timing0 (hex like 0x03 or decimal)\n"
        << "  --timing1 <value>   Override Timing1 (hex like 0x1C or decimal)\n"
        << "  --poll-ms <ms>      Sleep between polls (default: 1)\n"
        << "  --verbose           Log bridge activity\n"
        << "  --help              Show this help\n";
}

bool parse_int(const std::string& value, int& out) {
    if (value.empty()) {
        return false;
    }
    char* end = nullptr;
    long parsed = std::strtol(value.c_str(), &end, 0);
    if (!end || *end != '\0') {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

bool parse_args(int argc, char* argv[], Options& opts) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        }
        if (arg == "--verbose") {
            opts.verbose = true;
            continue;
        }
        auto require_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                print_usage(argv[0]);
                return "";
            }
            return argv[++i];
        };
        if (arg == "--iface") {
            opts.iface = require_value("--iface");
            if (opts.iface.empty()) return false;
        } else if (arg == "--device") {
            const auto value = require_value("--device");
            if (value.empty() || !parse_int(value, opts.device_index)) return false;
        } else if (arg == "--channel") {
            const auto value = require_value("--channel");
            if (value.empty() || !parse_int(value, opts.channel_index)) return false;
        } else if (arg == "--bitrate") {
            const auto value = require_value("--bitrate");
            if (value.empty() || !parse_int(value, opts.bitrate)) return false;
        } else if (arg == "--timing0") {
            const auto value = require_value("--timing0");
            if (value.empty() || !parse_int(value, opts.timing0)) return false;
        } else if (arg == "--timing1") {
            const auto value = require_value("--timing1");
            if (value.empty() || !parse_int(value, opts.timing1)) return false;
        } else if (arg == "--poll-ms") {
            const auto value = require_value("--poll-ms");
            if (value.empty() || !parse_int(value, opts.poll_ms)) return false;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

bool resolve_timing(const Options& opts, int& timing0, int& timing1) {
    if (opts.timing0 >= 0 && opts.timing1 >= 0) {
        timing0 = opts.timing0;
        timing1 = opts.timing1;
        return true;
    }
    if (opts.bitrate == 125000) {
        timing0 = 0x03;
        timing1 = 0x1C;
        return true;
    }
    std::cerr << "Unsupported bitrate " << opts.bitrate
              << " (use --timing0/--timing1 for custom values)\n";
    return false;
}

int open_can_socket(const std::string& iface) {
    int sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        std::perror("socket(PF_CAN) failed");
        return -1;
    }
    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface.c_str());
    if (::ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        std::perror("SIOCGIFINDEX failed");
        ::close(sock);
        return -1;
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind(AF_CAN) failed");
        ::close(sock);
        return -1;
    }
    return sock;
}

bool init_device(const Options& opts, int timing0, int timing1) {
    if (VCI_OpenDevice(VCI_USBCAN2, static_cast<DWORD>(opts.device_index), 0) != STATUS_OK) {
        std::cerr << "VCI_OpenDevice failed for device " << opts.device_index << "\n";
        return false;
    }
    VCI_INIT_CONFIG cfg{};
    cfg.AccCode = 0;
    cfg.AccMask = 0xFFFFFFFF;
    cfg.Filter = 1;
    cfg.Timing0 = static_cast<UCHAR>(timing0);
    cfg.Timing1 = static_cast<UCHAR>(timing1);
    cfg.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN2, static_cast<DWORD>(opts.device_index),
                    static_cast<DWORD>(opts.channel_index), &cfg) != STATUS_OK) {
        std::cerr << "VCI_InitCAN failed\n";
        VCI_CloseDevice(VCI_USBCAN2, static_cast<DWORD>(opts.device_index));
        return false;
    }
    if (VCI_StartCAN(VCI_USBCAN2, static_cast<DWORD>(opts.device_index),
                     static_cast<DWORD>(opts.channel_index)) != STATUS_OK) {
        std::cerr << "VCI_StartCAN failed\n";
        VCI_CloseDevice(VCI_USBCAN2, static_cast<DWORD>(opts.device_index));
        return false;
    }
    VCI_ClearBuffer(VCI_USBCAN2, static_cast<DWORD>(opts.device_index),
                    static_cast<DWORD>(opts.channel_index));
    return true;
}

void shutdown_device(const Options& opts) {
    VCI_ResetCAN(VCI_USBCAN2, static_cast<DWORD>(opts.device_index),
                 static_cast<DWORD>(opts.channel_index));
    VCI_CloseDevice(VCI_USBCAN2, static_cast<DWORD>(opts.device_index));
}
} // namespace

int main(int argc, char* argv[]) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        return 1;
    }
    if (opts.channel_index < 0 || opts.channel_index > 1) {
        std::cerr << "channel must be 0 or 1\n";
        return 1;
    }
    if (opts.poll_ms < 0) {
        opts.poll_ms = 0;
    }

    int timing0 = 0;
    int timing1 = 0;
    if (!resolve_timing(opts, timing0, timing1)) {
        return 1;
    }

    const int sock = open_can_socket(opts.iface);
    if (sock < 0) {
        return 1;
    }

    if (!init_device(opts, timing0, timing1)) {
        ::close(sock);
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (opts.verbose) {
        std::cout << "USB-CAN-B bridge running: iface=" << opts.iface
                  << " dev=" << opts.device_index
                  << " ch=" << opts.channel_index
                  << " timing0=0x" << std::hex << timing0
                  << " timing1=0x" << timing1 << std::dec << "\n";
    }

    std::vector<VCI_CAN_OBJ> rx_buf(64);
    while (running) {
        for (;;) {
            struct can_frame frame {};
            const ssize_t nbytes = ::recv(sock, &frame, sizeof(frame), MSG_DONTWAIT);
            if (nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                if (errno == EINTR) {
                    continue;
                }
                std::perror("recv(can) failed");
                running = false;
                break;
            }
            if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
                continue;
            }
            if (frame.can_id & CAN_ERR_FLAG) {
                continue;
            }
            VCI_CAN_OBJ tx{};
            tx.ID = frame.can_id & CAN_EFF_MASK;
            tx.ExternFlag = (frame.can_id & CAN_EFF_FLAG) ? 1 : 0;
            tx.RemoteFlag = (frame.can_id & CAN_RTR_FLAG) ? 1 : 0;
            tx.SendType = 0;
            tx.DataLen = frame.can_dlc;
            if (tx.DataLen > 8) {
                tx.DataLen = 8;
            }
            std::memcpy(tx.Data, frame.data, tx.DataLen);
            const auto sent = VCI_Transmit(VCI_USBCAN2, static_cast<DWORD>(opts.device_index),
                                           static_cast<DWORD>(opts.channel_index), &tx, 1);
            if (sent != 1 && opts.verbose) {
                std::cerr << "VCI_Transmit failed (sent=" << sent << ")\n";
            }
        }

        const auto received = VCI_Receive(VCI_USBCAN2, static_cast<DWORD>(opts.device_index),
                                          static_cast<DWORD>(opts.channel_index),
                                          rx_buf.data(), static_cast<UINT>(rx_buf.size()), 0);
        for (ULONG i = 0; i < received; ++i) {
            const auto& rx = rx_buf[i];
            struct can_frame frame {};
            frame.can_id = rx.ID & CAN_EFF_MASK;
            if (rx.ExternFlag) {
                frame.can_id |= CAN_EFF_FLAG;
            }
            if (rx.RemoteFlag) {
                frame.can_id |= CAN_RTR_FLAG;
            }
            frame.can_dlc = rx.DataLen;
            if (frame.can_dlc > 8) {
                frame.can_dlc = 8;
            }
            std::memcpy(frame.data, rx.Data, frame.can_dlc);
            const auto written = ::write(sock, &frame, sizeof(frame));
            if (written != static_cast<ssize_t>(sizeof(frame)) && opts.verbose) {
                std::perror("write(can) failed");
            }
        }

        if (opts.poll_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(opts.poll_ms));
        }
    }

    shutdown_device(opts);
    ::close(sock);
    return 0;
}
