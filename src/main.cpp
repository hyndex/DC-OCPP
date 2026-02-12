// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"
#include "ocpp_adapter.hpp"
#include "plc_can.hpp"

#include <atomic>
#include <cctype>
#include <csignal>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <set>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
std::atomic<bool> keep_running{true};

struct InterfaceLock {
    int fd{-1};
    std::string path;

    InterfaceLock() = default;
    InterfaceLock(int lock_fd, std::string lock_path) : fd(lock_fd), path(std::move(lock_path)) {}
    InterfaceLock(const InterfaceLock&) = delete;
    InterfaceLock& operator=(const InterfaceLock&) = delete;
    InterfaceLock(InterfaceLock&& other) noexcept : fd(other.fd), path(std::move(other.path)) { other.fd = -1; }
    InterfaceLock& operator=(InterfaceLock&& other) noexcept {
        if (this != &other) {
            release();
            fd = other.fd;
            path = std::move(other.path);
            other.fd = -1;
        }
        return *this;
    }
    ~InterfaceLock() { release(); }

    void release() {
        if (fd >= 0) {
            (void)::flock(fd, LOCK_UN);
            (void)::close(fd);
            fd = -1;
        }
    }
};

void handle_signal(int) {
    keep_running = false;
}

std::string parse_config_path(int argc, char* argv[]) {
    std::string path = "configs/charger.json";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            path = argv[i + 1];
        }
    }
    return path;
}

std::string sanitize_iface(const std::string& iface) {
    std::string out;
    out.reserve(iface.size());
    for (const unsigned char ch : iface) {
        if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.') {
            out.push_back(static_cast<char>(ch));
        } else {
            out.push_back('_');
        }
    }
    if (out.empty()) {
        return "unknown";
    }
    return out;
}

std::string trim_copy(std::string in) {
    while (!in.empty() && std::isspace(static_cast<unsigned char>(in.front()))) {
        in.erase(in.begin());
    }
    while (!in.empty() && std::isspace(static_cast<unsigned char>(in.back()))) {
        in.pop_back();
    }
    return in;
}

bool acquire_can_lock(const std::string& iface, InterfaceLock& lock_out, std::string& error) {
    const std::string lock_path = "/tmp/dc_ocpp_can_" + sanitize_iface(iface) + ".lock";
    const int fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR, 0644);
    if (fd < 0) {
        error = "Failed to open lock file for CAN iface '" + iface + "': " + std::strerror(errno);
        return false;
    }

    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        std::string holder;
        char buf[128] = {};
        (void)::lseek(fd, 0, SEEK_SET);
        const ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            holder = trim_copy(std::string(buf, static_cast<std::size_t>(n)));
        }
        if (holder.empty()) {
            error = "Another dc_ocpp instance is already using CAN iface '" + iface + "' (lock " + lock_path + ")";
        } else {
            error = "Another dc_ocpp instance is already using CAN iface '" + iface + "' (lock " + lock_path +
                    ", holder_pid=" + holder + ")";
        }
        (void)::close(fd);
        return false;
    }

    const std::string pid_line = std::to_string(::getpid()) + "\n";
    (void)::ftruncate(fd, 0);
    (void)::lseek(fd, 0, SEEK_SET);
    (void)::write(fd, pid_line.data(), pid_line.size());
    lock_out = InterfaceLock(fd, lock_path);
    return true;
}
} // namespace

int main(int argc, char* argv[]) {
    const auto config_path = parse_config_path(argc, argv);

    charger::ChargerConfig cfg;
    try {
        cfg = charger::load_charger_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return 1;
    }

    if (!cfg.use_plc) {
        std::cerr << "Simulated hardware backend removed; set chargePoint.usePlc=true in config" << std::endl;
        return 1;
    }

    std::set<std::string> can_ifaces;
    if (!cfg.can_interface.empty()) {
        can_ifaces.insert(cfg.can_interface);
    }
    for (const auto& c : cfg.connectors) {
        const std::string iface = c.can_interface.empty() ? cfg.can_interface : c.can_interface;
        if (!iface.empty()) {
            can_ifaces.insert(iface);
        }
    }
    if (can_ifaces.empty()) {
        std::cerr << "No CAN interface configured; refusing to start" << std::endl;
        return 1;
    }

    std::vector<InterfaceLock> can_locks;
    can_locks.reserve(can_ifaces.size());
    for (const auto& iface : can_ifaces) {
        InterfaceLock lock;
        std::string error;
        if (!acquire_can_lock(iface, lock, error)) {
            std::cerr << error << std::endl;
            return 1;
        }
        can_locks.emplace_back(std::move(lock));
    }

    auto plc_hw = std::make_shared<charger::PlcCanHardware>(cfg);
    if (!plc_hw->ok()) {
        std::cerr << "PLC CAN backend failed to initialize on interface " << cfg.can_interface << std::endl;
        return 1;
    }
    cfg.plc_backend_available = true;
    std::shared_ptr<charger::HardwareInterface> hardware = plc_hw;
    charger::OcppAdapter adapter(cfg, hardware);

    if (!adapter.start()) {
        std::cerr << "Failed to start OCPP adapter" << std::endl;
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    adapter.stop();
    return 0;
}
