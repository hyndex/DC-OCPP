// SPDX-License-Identifier: Apache-2.0
#include "can_plc.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <cstdlib>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <cmath>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <curl/curl.h>
#include <everest/logging.hpp>
#include "can_contract.hpp"

namespace charger {

namespace {
constexpr uint32_t RELAY_STATUS_BASE = 0x160;
constexpr uint32_t SAFETY_STATUS_BASE = 0x190;
constexpr uint32_t ENERGY_METER_BASE = 0x170;
constexpr uint32_t CONFIG_ACK_BASE = 0x1A0;
constexpr uint32_t CONFIG_CMD_BASE = 0x380;
constexpr uint32_t CP_LEVELS_BASE = 0x430;
constexpr uint32_t CHARGING_SESSION_BASE = 0x410;
constexpr uint32_t EVAC_CTRL_BASE = 0x250;
constexpr uint32_t EVDC_MAX_LIMITS_BASE = 0x200;
constexpr uint32_t EVDC_TARGETS_BASE = 0x210;
constexpr uint32_t EVDC_ENERGY_LIMITS_BASE = 0x230;
constexpr uint32_t EV_STATUS_DISPLAY_BASE = 0x220;
constexpr uint32_t CHARGE_INFO_BASE = 0x100;
constexpr uint32_t RFID_EVENT_BASE = 0x180;
constexpr uint32_t EVCCID_BASE = 0x280;
constexpr uint32_t EMAID0_BASE = 0x260;
constexpr uint32_t EMAID1_BASE = 0x270;
constexpr uint32_t EVMAC_BASE = 0x240;
constexpr uint32_t EVSE_DC_MAX_LIMITS_CMD_BASE = 0x300;
constexpr uint32_t EVSE_DC_PRESENT_CMD_BASE = 0x310;
constexpr uint32_t GCMC_STATUS_BASE = 0x150;
constexpr uint32_t HW_STATUS_BASE = 0x130;
constexpr uint32_t DEBUG_INFO_BASE = 0x1B0;
constexpr uint32_t BOOT_CONFIG_BASE = 0x90000;
constexpr uint32_t GCMC_CMD_BASE = 0x390;
constexpr uint32_t RELAY_CMD_BASE = 0x340;
constexpr uint32_t RTEVLOG_BASE = 0x420;
constexpr uint32_t RTTLOG_BASE = 0x400;
constexpr uint32_t SOFTWARE_INFO_BASE = 0x110;
constexpr uint32_t ERROR_CODES_BASE = 0x120;
constexpr uint32_t HW_CONFIG_BASE = 0x306;
constexpr std::chrono::milliseconds RELAY_TIMEOUT_MS(300);
constexpr std::chrono::milliseconds METER_TIMEOUT_MS(2000);
constexpr std::chrono::milliseconds SAFETY_DEBOUNCE_MS(50);
constexpr std::chrono::milliseconds CP_TIMEOUT_MS(1000);
constexpr std::chrono::milliseconds BUS_IDLE_TIMEOUT_MS(1000);
constexpr std::chrono::milliseconds PROTO_VERSION_RETRY_MS(1000);
constexpr uint8_t HLC_POWER_DELIVERY_STAGE = 4;
constexpr std::chrono::seconds SEGMENT_TTL(5);
constexpr uint8_t CONFIG_PARAM_EVSE_LIMIT_ACK = can_contract::kConfigParamEvseLimitAck;
constexpr uint8_t CONFIG_PARAM_PROTO_VERSION = can_contract::kConfigParamProtoVersion;
constexpr uint32_t CAN_PROTOCOL_VERSION = can_contract::kProtoVersion;
constexpr uint8_t CONFIG_PARAM_AUTH_STATE = can_contract::kConfigParamAuthState;
constexpr uint8_t CONFIG_PARAM_AUTH_PENDING = can_contract::kConfigParamAuthPending;
constexpr uint8_t CONFIG_PARAM_LOCK_CMD = can_contract::kConfigParamLockCmd;
constexpr std::chrono::milliseconds AUTH_REFRESH_INTERVAL_MS(1000);

uint16_t le_u16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

int16_t le_i16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

uint32_t le_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

std::string bytes_to_hex(const std::vector<uint8_t>& bytes, size_t len) {
    std::ostringstream oss;
    oss << std::uppercase << std::hex;
    for (size_t i = 0; i < len && i < bytes.size(); ++i) {
        oss << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]);
    }
    return oss.str();
}

bool cp_state_plugged(char state_char) {
    switch (state_char) {
    case 'B':
    case 'C':
    case 'D':
        return true;
    default:
        return false;
    }
}

std::string bundle_logs(const std::string& prefix) {
    const auto ts = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    const std::string dir = "logs";
    std::filesystem::create_directories(dir);
    const std::string fname = dir + "/" + prefix + "_" + std::to_string(ts) + ".log";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    out << "Log bundle generated at " << ts << "\n";
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().string() == fname) continue;
        out << "\n==== " << entry.path().filename().string() << " ====\n";
        std::ifstream in(entry.path());
        out << in.rdbuf();
    }
    return fname;
}

bool is_http_url(const std::string& target) {
    return target.rfind("http://", 0) == 0 || target.rfind("https://", 0) == 0;
}

bool upload_via_curl(const std::string& source, const std::string& url, bool require_https,
                     int connect_timeout_s, int transfer_timeout_s) {
    if (url.empty()) return false;
    if (require_https && url.rfind("https://", 0) != 0) {
        EVLOG_warning << "Rejecting non-HTTPS upload target " << url;
        return false;
    }
    static std::once_flag curl_once;
    std::call_once(curl_once, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    std::FILE* file = std::fopen(source.c_str(), "rb");
    if (!file) {
        curl_easy_cleanup(curl);
        return false;
    }

    const auto size = std::filesystem::file_size(source);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_READDATA, file);
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, static_cast<curl_off_t>(size));
    curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s > 0 ? connect_timeout_s : 10);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, transfer_timeout_s > 0 ? transfer_timeout_s : 60);

    const CURLcode res = curl_easy_perform(curl);
    std::fclose(file);
    curl_easy_cleanup(curl);
    return res == CURLE_OK;
}

bool upload_file_to_target(const std::string& source,
                           const std::string& target,
                           bool require_https,
                           std::size_t max_bytes,
                           int connect_timeout_s,
                           int transfer_timeout_s,
                           bool allow_file_targets) {
    if (target.empty()) {
        return false;
    }
    std::error_code ec;
    const auto source_size = std::filesystem::file_size(source, ec);
    if (ec || source_size > max_bytes) {
        EVLOG_warning << "Rejecting upload: size exceeds limit or unreadable (" << source << ")";
        return false;
    }
    if (is_http_url(target)) {
        return upload_via_curl(source, target, require_https, connect_timeout_s, transfer_timeout_s);
    }
    if (!allow_file_targets) {
        EVLOG_warning << "File uploads disabled; rejecting target " << target;
        return false;
    }
    std::string path = target;
    const std::string prefix = "file://";
    if (path.rfind(prefix, 0) == 0) {
        path = path.substr(prefix.size());
    }
    const auto canonical_parent = std::filesystem::weakly_canonical(std::filesystem::path(path).parent_path(), ec);
    if (ec) {
        EVLOG_warning << "Rejecting upload: invalid path " << path;
        return false;
    }
    std::filesystem::create_directories(canonical_parent, ec);
    const auto dest_path = std::filesystem::weakly_canonical(path, ec);
    if (ec) {
        EVLOG_warning << "Rejecting upload: invalid destination " << path;
        return false;
    }
    std::filesystem::copy_file(source, dest_path, std::filesystem::copy_options::overwrite_existing, ec);
    return !ec;
}

bool download_via_curl(const std::string& url, const std::string& destination, bool require_https,
                       int connect_timeout_s, int transfer_timeout_s) {
    if (url.empty()) return false;
    if (require_https && url.rfind("https://", 0) != 0) {
        EVLOG_warning << "Rejecting non-HTTPS firmware URL " << url;
        return false;
    }
    static std::once_flag curl_once;
    std::call_once(curl_once, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });
    CURL* curl = curl_easy_init();
    if (!curl) return false;
    std::FILE* file = std::fopen(destination.c_str(), "wb");
    if (!file) {
        curl_easy_cleanup(curl);
        return false;
    }
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, nullptr);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, file);
    curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s > 0 ? connect_timeout_s : 10);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, transfer_timeout_s > 0 ? transfer_timeout_s : 60);
    const CURLcode res = curl_easy_perform(curl);
    std::fclose(file);
    curl_easy_cleanup(curl);
    return res == CURLE_OK;
}

bool fetch_firmware(const std::string& location,
                    std::size_t max_bytes,
                    bool require_https,
                    int connect_timeout_s,
                    int transfer_timeout_s,
                    bool allow_file_targets) {
    if (location.empty()) return false;
    std::error_code ec;
    const std::string dest = "data/firmware/download.bin";
    std::filesystem::create_directories(std::filesystem::path(dest).parent_path(), ec);
    if (is_http_url(location)) {
        if (!download_via_curl(location, dest, require_https, connect_timeout_s, transfer_timeout_s)) {
            return false;
        }
    } else {
        if (!allow_file_targets) return false;
        std::string src = location;
        const std::string prefix = "file://";
        if (src.rfind(prefix, 0) == 0) {
            src = src.substr(prefix.size());
        }
        std::filesystem::copy_file(src, dest, std::filesystem::copy_options::overwrite_existing, ec);
        if (ec) return false;
    }
    const auto size = std::filesystem::file_size(dest, ec);
    if (ec || size > max_bytes) {
        EVLOG_warning << "Firmware download failed size check";
        std::filesystem::remove(dest, ec);
        return false;
    }
    return true;
}

uint16_t clamp_to_deciv(double value) {
    if (value <= 0.0) return 0;
    const double scaled = std::round(value * 10.0);
    if (scaled >= 65535.0) return 65535;
    if (scaled <= 0.0) return 0;
    return static_cast<uint16_t>(scaled);
}

bool crc_expected(uint32_t can_id) {
    const uint32_t masked = can_id & PLC_TX_MASK;
    return masked == (RELAY_STATUS_BASE & PLC_TX_MASK) ||
           masked == (SAFETY_STATUS_BASE & PLC_TX_MASK) ||
           masked == (CONFIG_ACK_BASE & PLC_TX_MASK) ||
           masked == (GCMC_STATUS_BASE & PLC_TX_MASK) ||
           masked == (RELAY_CMD_BASE & PLC_TX_MASK) ||
           masked == (CONFIG_CMD_BASE & PLC_TX_MASK) ||
           masked == (GCMC_CMD_BASE & PLC_TX_MASK);
}
} // namespace

bool PlcHardware::open_socket() {
#ifdef __linux__
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        EVLOG_error << "Failed to open CAN socket on " << iface_ << ": " << std::strerror(errno);
        return false;
    }
    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface_.c_str());
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        EVLOG_error << "CAN interface not found: " << iface_;
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        EVLOG_error << "Failed to bind CAN socket on " << iface_ << ": " << std::strerror(errno);
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    const can_err_mask_t err_mask = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED | CAN_ERR_CRTL;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        EVLOG_error << "Failed to set CAN error filter: " << std::strerror(errno);
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    std::vector<can_filter> filters;
    for (const auto& kv : plc_to_connector_) {
        const int plc_id = kv.first & 0x0F;
        const PlcFilterSpec relay = make_plc_rx_filter(RELAY_STATUS_BASE, plc_id);
        can_filter f{};
        f.can_id = relay.id;
        f.can_mask = relay.mask;
        filters.push_back(f);
        const PlcFilterSpec safety = make_plc_rx_filter(SAFETY_STATUS_BASE, plc_id);
        f.can_id = safety.id; f.can_mask = safety.mask; filters.push_back(f);
        const PlcFilterSpec energy = make_plc_rx_filter(ENERGY_METER_BASE, plc_id);
        f.can_id = energy.id; f.can_mask = energy.mask; filters.push_back(f);
        const PlcFilterSpec cfgack = make_plc_rx_filter(CONFIG_ACK_BASE, plc_id);
        f.can_id = cfgack.id; f.can_mask = cfgack.mask; filters.push_back(f);
        const PlcFilterSpec cp = make_plc_rx_filter(CP_LEVELS_BASE, plc_id);
        f.can_id = cp.id; f.can_mask = cp.mask; filters.push_back(f);
        const PlcFilterSpec session = make_plc_rx_filter(CHARGING_SESSION_BASE, plc_id);
        f.can_id = session.id; f.can_mask = session.mask; filters.push_back(f);
        const PlcFilterSpec evac = make_plc_rx_filter(EVAC_CTRL_BASE, plc_id);
        f.can_id = evac.id; f.can_mask = evac.mask; filters.push_back(f);
        const PlcFilterSpec max_limits = make_plc_rx_filter(EVDC_MAX_LIMITS_BASE, plc_id);
        f.can_id = max_limits.id; f.can_mask = max_limits.mask; filters.push_back(f);
        const PlcFilterSpec targets = make_plc_rx_filter(EVDC_TARGETS_BASE, plc_id);
        f.can_id = targets.id; f.can_mask = targets.mask; filters.push_back(f);
        const PlcFilterSpec energy_limits = make_plc_rx_filter(EVDC_ENERGY_LIMITS_BASE, plc_id);
        f.can_id = energy_limits.id; f.can_mask = energy_limits.mask; filters.push_back(f);
        const PlcFilterSpec ev_status = make_plc_rx_filter(EV_STATUS_DISPLAY_BASE, plc_id);
        f.can_id = ev_status.id; f.can_mask = ev_status.mask; filters.push_back(f);
        const PlcFilterSpec charge_info = make_plc_rx_filter(CHARGE_INFO_BASE, plc_id);
        f.can_id = charge_info.id; f.can_mask = charge_info.mask; filters.push_back(f);
        const PlcFilterSpec rfid = make_plc_rx_filter(RFID_EVENT_BASE, plc_id);
        f.can_id = rfid.id; f.can_mask = rfid.mask; filters.push_back(f);
        const PlcFilterSpec evccid = make_plc_rx_filter(EVCCID_BASE, plc_id);
        f.can_id = evccid.id; f.can_mask = evccid.mask; filters.push_back(f);
        const PlcFilterSpec emaid0 = make_plc_rx_filter(EMAID0_BASE, plc_id);
        f.can_id = emaid0.id; f.can_mask = emaid0.mask; filters.push_back(f);
        const PlcFilterSpec emaid1 = make_plc_rx_filter(EMAID1_BASE, plc_id);
        f.can_id = emaid1.id; f.can_mask = emaid1.mask; filters.push_back(f);
        const PlcFilterSpec evmac = make_plc_rx_filter(EVMAC_BASE, plc_id);
        f.can_id = evmac.id; f.can_mask = evmac.mask; filters.push_back(f);
        const PlcFilterSpec gcmc_status = make_plc_rx_filter(GCMC_STATUS_BASE, plc_id);
        f.can_id = gcmc_status.id; f.can_mask = gcmc_status.mask; filters.push_back(f);
        const PlcFilterSpec hw_status = make_plc_rx_filter(HW_STATUS_BASE, plc_id);
        f.can_id = hw_status.id; f.can_mask = hw_status.mask; filters.push_back(f);
        const PlcFilterSpec debug_info = make_plc_rx_filter(DEBUG_INFO_BASE, plc_id);
        f.can_id = debug_info.id; f.can_mask = debug_info.mask; filters.push_back(f);
        const PlcFilterSpec boot_cfg = make_plc_rx_filter(BOOT_CONFIG_BASE, plc_id);
        f.can_id = boot_cfg.id; f.can_mask = boot_cfg.mask; filters.push_back(f);
        const PlcFilterSpec rtev = make_plc_rx_filter(RTEVLOG_BASE, plc_id);
        f.can_id = rtev.id; f.can_mask = rtev.mask; filters.push_back(f);
        const PlcFilterSpec rtt = make_plc_rx_filter(RTTLOG_BASE, plc_id);
        f.can_id = rtt.id; f.can_mask = rtt.mask; filters.push_back(f);
        const PlcFilterSpec sw_info = make_plc_rx_filter(SOFTWARE_INFO_BASE, plc_id);
        f.can_id = sw_info.id; f.can_mask = sw_info.mask; filters.push_back(f);
        const PlcFilterSpec err = make_plc_rx_filter(ERROR_CODES_BASE, plc_id);
        f.can_id = err.id; f.can_mask = err.mask; filters.push_back(f);
        const PlcFilterSpec hw_cfg = make_plc_rx_filter(HW_CONFIG_BASE, plc_id);
        f.can_id = hw_cfg.id; f.can_mask = hw_cfg.mask; filters.push_back(f);
    }
    if (!filters.empty()) {
        if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                       sizeof(can_filter) * filters.size()) < 0) {
            EVLOG_error << "Failed to set CAN filters: " << std::strerror(errno);
            ::close(sock_);
            sock_ = -1;
            return false;
        }
    }
    return true;
#else
    (void)iface_;
    return false;
#endif
}

void PlcHardware::restart_can() {
#ifdef __linux__
    std::lock_guard<std::mutex> lock(mtx_);
    if (!running_) return;
    if (open_socket()) {
        restart_requested_ = false;
        EVLOG_warning << "CAN socket reopened on " << iface_;
    } else {
        EVLOG_error << "Failed to reopen CAN socket on " << iface_;
    }
#endif
}

void PlcHardware::ingest_can_frame(uint32_t can_id, const uint8_t* data, size_t len) {
    handle_frame(can_id, data, len);
}

PlcHardware::PlcHardware(const ChargerConfig& cfg, bool open_can) : use_crc8_(cfg.plc_use_crc8),
                                                     module_relays_enabled_(cfg.plc_module_relays_enabled),
                                                     gun_relay_owned_by_plc_(cfg.plc_owns_gun_relay),
                                                     present_warn_ms_(cfg.plc_present_warn_ms),
                                                     limits_warn_ms_(cfg.plc_limits_warn_ms),
                                                     require_https_uploads_(cfg.require_https_uploads),
                                                     upload_max_bytes_(cfg.upload_max_bytes),
                                                     upload_connect_timeout_s_(cfg.upload_connect_timeout_s),
                                                     upload_transfer_timeout_s_(cfg.upload_transfer_timeout_s),
                                                     upload_allow_file_targets_(cfg.upload_allow_file_targets) {
    iface_ = cfg.can_interface.empty() ? "can0" : cfg.can_interface;
    for (const auto& c : cfg.connectors) {
        if (plc_to_connector_.count(c.plc_id)) {
            throw std::runtime_error("Duplicate plc_id " + std::to_string(c.plc_id) + " in PLC driver init");
        }
        Node node{};
        node.cfg = c;
        node.lock_engaged = !c.require_lock;
        node.lock_feedback_engaged = !c.require_lock;
        node.module_mask = 0x00;
        nodes_.emplace(c.id, node);
        plc_to_connector_[c.plc_id] = c.id;
    }
    if (!cfg.plc_use_crc8) {
        EVLOG_warning << "Ignoring plc_use_crc8=false; enforcing CRC8 on safety-critical PLC frames";
        use_crc8_ = true;
    }
    EVLOG_info << "PLC gun relay ownership: " << (gun_relay_owned_by_plc_ ? "PLC" : "controller");

#ifdef __linux__
    if (open_can) {
        if (!open_socket()) {
            throw std::runtime_error("Failed to open CAN socket");
        }
        running_ = true;
        rx_thread_ = std::thread([this]() { rx_loop(); });
    } else {
        running_ = false;
    }
#else
    if (open_can) {
        throw std::runtime_error("CAN PLC driver requires Linux socketcan support");
    }
#endif
}

PlcHardware::~PlcHardware() {
    running_ = false;
#ifdef __linux__
    if (sock_ >= 0) {
        shutdown(sock_, SHUT_RDWR);
    }
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (sock_ >= 0) {
        close(sock_);
    }
#endif
}

PlcHardware::Node* PlcHardware::find_node(std::int32_t connector) {
    auto it = nodes_.find(connector);
    if (it == nodes_.end()) {
        return nullptr;
    }
    return &it->second;
}

PlcHardware::Node* PlcHardware::find_node_by_plc(int plc_id) {
    auto it = plc_to_connector_.find(plc_id);
    if (it == plc_to_connector_.end()) return nullptr;
    return find_node(it->second);
}

bool PlcHardware::enable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(connector);
    if (!node) return false;
    if (node->cfg.require_lock && !node->lock_engaged) {
        return false;
    }
    return send_relay_command(*node, true, false);
}

bool PlcHardware::disable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(connector);
    if (!node) return false;
    node->lock_engaged = true; // auto-lock when disabling
    return send_relay_command(*node, false, true);
}

bool PlcHardware::pause_charging(std::int32_t connector) {
    return disable(connector);
}

bool PlcHardware::resume_charging(std::int32_t connector) {
    return enable(connector);
}

bool PlcHardware::stop_transaction(std::int32_t connector, ocpp::v16::Reason /*reason*/) {
    return disable(connector);
}

void PlcHardware::apply_power_command(const PowerCommand& cmd) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(cmd.connector);
    if (!node) return;
    if (!node->authorization_granted) {
        node->mc_closed_cmd = false;
        node->gc_closed_cmd = false;
        node->module_mask = 0x00;
        send_relay_command(*node, false, true);
        return;
    }
    node->mc_closed_cmd = cmd.mc_closed;
    node->gc_closed_cmd = cmd.gc_closed;
    const bool gc_cmd_allowed = !gun_relay_owned_by_plc_;
    if (!cmd.mc_closed) {
        node->module_mask = 0x00;
        send_relay_command(*node, false, true);
        return;
    }
    uint8_t mask = 0x00;
    if (module_relays_enabled_) {
        // module_mask bit0 -> module0, map to RLY2 (bit1)
        if (cmd.module_mask & 0x01) {
            mask |= 0x02;
        }
        if (cmd.module_mask & 0x02) {
            mask |= 0x04;
        }
        if (cmd.module_mask & 0xFC) {
            EVLOG_warning << "PLC command module mask uses unsupported bits: 0x" << std::hex
                          << static_cast<int>(cmd.module_mask) << std::dec << " (only two modules supported by PLC relays)";
        }
    } else if (!module_relays_disabled_logged_) {
        EVLOG_info << "PLC module relays disabled by config; ignoring module_mask updates";
        module_relays_disabled_logged_ = true;
    }
    if (gc_cmd_allowed && cmd.module_count > 0 && cmd.gc_closed) {
        mask |= 0x01;
    }
    node->module_mask = mask;
    const bool any_relay_cmd = (mask & 0x07) != 0;
    const bool expect_plc_gc = gun_relay_owned_by_plc_ && cmd.module_count > 0 && cmd.gc_closed;
    const bool force_all_off = !any_relay_cmd && !expect_plc_gc;
    send_relay_command(*node, any_relay_cmd, force_all_off);
}

void PlcHardware::apply_power_allocation(std::int32_t connector, int modules) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(connector);
    if (!node) return;
    if (!node->authorization_granted) {
        node->mc_closed_cmd = false;
        node->gc_closed_cmd = false;
        node->module_mask = 0x00;
        send_relay_command(*node, false, true);
        return;
    }
    const bool gc_cmd_allowed = !gun_relay_owned_by_plc_;
    uint8_t mask = 0x00;
    if (module_relays_enabled_) {
        if (modules >= 1) {
            mask |= 0x02; // module 1
        }
        if (modules >= 2) {
            mask |= 0x04; // module 2
        }
    } else if (!module_relays_disabled_logged_) {
        EVLOG_info << "PLC module relays disabled by config; ignoring module allocation";
        module_relays_disabled_logged_ = true;
    }
    // keep gun relay in sync with module availability
    if (gc_cmd_allowed && modules > 0) {
        mask |= 0x01;
    }
    node->module_mask = mask;
    const bool close = (mask & 0x07) != 0;
    const bool expect_plc_gc = gun_relay_owned_by_plc_ && modules > 0;
    const bool force_all_off = !close && !expect_plc_gc;
    send_relay_command(*node, close, force_all_off);
}

ocpp::v16::UnlockStatus PlcHardware::unlock(std::int32_t /*connector*/) {
    std::lock_guard<std::mutex> lock(mtx_);
    const bool can_available = (sock_ >= 0);
    bool failed = false;
    for (auto& kv : nodes_) {
        bool sent = true;
        if (can_available) {
            sent = send_config_command(kv.second, CONFIG_PARAM_LOCK_CMD, 0U);
        }
        if (sent || !can_available) {
            kv.second.lock_engaged = false;
            kv.second.lock_feedback_engaged = false;
        }
        if (can_available && !sent) {
            failed = true;
        }
    }
    if (!can_available) {
        return ocpp::v16::UnlockStatus::Unlocked;
    }
    return failed ? ocpp::v16::UnlockStatus::UnlockFailed : ocpp::v16::UnlockStatus::Unlocked;
}

ocpp::v16::ReservationStatus PlcHardware::reserve(std::int32_t /*reservation_id*/, std::int32_t /*connector*/,
                                                  ocpp::DateTime /*expiry*/, const std::string& /*id_tag*/,
                                                  const std::optional<std::string>& /*parent_id*/) {
    return ocpp::v16::ReservationStatus::Accepted;
}

bool PlcHardware::cancel_reservation(std::int32_t /*reservation_id*/) {
    return true;
}

ocpp::v16::GetLogResponse PlcHardware::upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) {
    ocpp::v16::GetLogResponse resp;
    resp.status = ocpp::v16::LogStatusEnumType::Accepted;
    const std::string fname = bundle_logs("diagnostics");
    bool upload_ok = true;
    if (!request.location.empty()) {
        upload_ok = upload_file_to_target(fname, request.location, require_https_uploads_,
                                          upload_max_bytes_, upload_connect_timeout_s_,
                                          upload_transfer_timeout_s_, upload_allow_file_targets_);
        if (!upload_ok) {
            EVLOG_warning << "Diagnostics upload to " << request.location << " failed";
        }
    }
    if (!upload_ok) {
        resp.status = ocpp::v16::LogStatusEnumType::Rejected;
    }
    resp.filename.emplace(fname);
    return resp;
}

ocpp::v16::GetLogResponse PlcHardware::upload_logs(const ocpp::v16::GetLogRequest& request) {
    ocpp::v16::GetLogResponse resp;
    resp.status = ocpp::v16::LogStatusEnumType::Accepted;
    const std::string fname = bundle_logs("logs");
    bool upload_ok = true;
    if (!request.log.remoteLocation.get().empty()) {
        upload_ok = upload_file_to_target(fname, request.log.remoteLocation, require_https_uploads_,
                                          upload_max_bytes_, upload_connect_timeout_s_,
                                          upload_transfer_timeout_s_, upload_allow_file_targets_);
        if (!upload_ok) {
            EVLOG_warning << "Log upload to " << request.log.remoteLocation << " failed";
        }
    }
    if (!upload_ok) {
        resp.status = ocpp::v16::LogStatusEnumType::Rejected;
    }
    resp.filename.emplace(fname);
    return resp;
}

bool PlcHardware::update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) {
    const bool ok = fetch_firmware(request.location, upload_max_bytes_, require_https_uploads_,
                                   upload_connect_timeout_s_, upload_transfer_timeout_s_, upload_allow_file_targets_);
    if (!ok) {
        EVLOG_warning << "PLC firmware download failed for " << request.location;
    } else {
        EVLOG_info << "PLC firmware downloaded from " << request.location << " scheduled at "
                   << request.retrieveDate.to_rfc3339();
    }
    return ok;
}

ocpp::v16::UpdateFirmwareStatusEnumType
PlcHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) {
    const bool ok = fetch_firmware(request.firmware.location, upload_max_bytes_, require_https_uploads_,
                                   upload_connect_timeout_s_, upload_transfer_timeout_s_, upload_allow_file_targets_);
    return ok ? ocpp::v16::UpdateFirmwareStatusEnumType::Accepted
              : ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
}

void PlcHardware::set_connection_timeout(std::int32_t seconds) {
    std::lock_guard<std::mutex> lock(mtx_);
    connection_timeout_s_ = std::max<std::int32_t>(0, seconds);
    EVLOG_info << "Updated PLC connection timeout to " << connection_timeout_s_ << "s";
}

bool PlcHardware::is_reset_allowed(const ocpp::v16::ResetType& reset_type) {
    // Only soft resets are supported; hard resets require out-of-band control of the host.
    return reset_type == ocpp::v16::ResetType::Soft;
}

void PlcHardware::reset(const ocpp::v16::ResetType& reset_type) {
    if (reset_type != ocpp::v16::ResetType::Soft) {
        EVLOG_warning << "Hard reset requested but not supported by PLC backend";
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    restart_requested_ = true;
    for (auto& kv : nodes_) {
        kv.second.awaiting_ack = false;
        kv.second.mc_closed_cmd = false;
        kv.second.gc_closed_cmd = false;
    }
    EVLOG_info << "Soft reset requested; reopening CAN interface and clearing pending relay commands";
}

void PlcHardware::on_remote_start_token(const std::string& /*id_token*/,
                                        const std::vector<std::int32_t>& /*referenced_connectors*/,
                                        bool /*prevalidated*/) {
}

ocpp::Measurement PlcHardware::sample_meter(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    ocpp::Measurement m{};
    auto* node = find_node(connector);
    if (!node) {
        return m;
    }

    const auto now = std::chrono::steady_clock::now();
    auto& meter = node->status.meter;
    const bool has_rx = node->status.last_meter_rx.time_since_epoch().count() != 0;
    meter.stale = has_rx &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(now - node->status.last_meter_rx) > METER_TIMEOUT_MS);

    const bool prefer_shunt = (node->cfg.meter_source == "shunt");
    // Prefer PLC-provided voltage if present; otherwise reuse last known DC voltage.
    if (meter.voltage_v <= 0.0 && node->status.present_voltage_v > 0.0) {
        meter.voltage_v = node->status.present_voltage_v;
    }
    if (prefer_shunt) {
        // For shunt-mode, rely on present V/I from EVDC telemetry rather than PLC meter counters.
        const double v = node->status.present_voltage_v > 0.0 ? node->status.present_voltage_v : meter.voltage_v;
        const double i = node->status.present_current_a;
        meter.voltage_v = v;
        meter.current_a = i;
        meter.power_w = v * i;
        meter.stale = false;
    }

    const bool meter_ok = !prefer_shunt && meter.ok && !meter.stale;
    const double fallback_voltage = node->status.present_voltage_v > 0.0 ? node->status.present_voltage_v : meter.voltage_v;
    const double fallback_current = node->status.present_current_a > 0.0 ? node->status.present_current_a : meter.current_a;
    double fallback_power_w = node->status.present_power_w;
    if (fallback_power_w <= 0.0 && fallback_voltage > 0.0 && fallback_current > 0.0) {
        fallback_power_w = fallback_voltage * fallback_current;
    }

    double energy_wh = node->energy_fallback_Wh;
    const double power_for_energy_raw = meter_ok ? meter.power_w
        : (prefer_shunt ? meter.power_w : fallback_power_w);
    if (meter_ok) {
        node->meter_fallback_active = false;
        energy_wh = meter.energy_Wh;
        node->energy_fallback_Wh = energy_wh;
        node->last_energy_update = now;
    } else {
        node->meter_fallback_active = (power_for_energy_raw > 0.0);
        if (node->last_energy_update.time_since_epoch().count() == 0) {
            node->last_energy_update = now;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - node->last_energy_update).count() /
            1000.0;
        const double power_w = power_for_energy_raw * node->cfg.meter_scale;
        if (elapsed > 0) {
            const double delta = (power_w * elapsed) / 3600.0;
            node->energy_fallback_Wh = std::max(0.0, node->energy_fallback_Wh + delta);
        }
        node->last_energy_update = now;
        energy_wh = node->energy_fallback_Wh;
    }

    // Apply calibration scaling
    double power_raw = power_for_energy_raw;
    if (!meter_ok && !prefer_shunt && fallback_power_w > 0.0) {
        power_raw = fallback_power_w;
    }
    const double power_scaled = power_raw * node->cfg.meter_scale;
    const double current_scaled = meter.current_a * node->cfg.meter_scale;
    const double energy_scaled = energy_wh * node->cfg.meter_scale + node->cfg.meter_offset_wh;
    node->status.present_power_w = power_scaled;
    node->status.present_current_a = current_scaled;

    m.power_meter.timestamp = ocpp::DateTime().to_rfc3339();
    m.power_meter.energy_Wh_import.total = energy_scaled;
    m.power_meter.voltage_V.emplace();
    m.power_meter.voltage_V->DC = static_cast<float>(meter.voltage_v);
    m.power_meter.current_A.emplace();
    m.power_meter.current_A->DC = static_cast<float>(current_scaled);
    m.power_meter.power_W.emplace();
    m.power_meter.power_W->total = static_cast<float>(power_scaled);
    return m;
}

GunStatus PlcHardware::get_status(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    GunStatus st{};
    auto* node = find_node(connector);
    if (!node) return st;
    const auto now = std::chrono::steady_clock::now();
    refresh_authorization_locked(*node, now, false);
    ensure_protocol_version(*node, now);
    // Timeout detections
    if (node->status.last_relay_status.time_since_epoch().count() &&
        (now - node->status.last_relay_status) > RELAY_TIMEOUT_MS) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
    }
    if (node->status.last_safety_status.time_since_epoch().count() &&
        (now - node->status.last_safety_status) > RELAY_TIMEOUT_MS) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
    }
    if (node->awaiting_ack && (now - node->last_cmd_sent) > RELAY_TIMEOUT_MS) {
        if (node->retry_count < 3) {
            // retry last command
            send_relay_command(*node, node->last_cmd_close, node->last_force_all_off);
            node->retry_count++;
        } else {
            node->status.safety.comm_fault = true;
            node->status.safety.safety_ok = false;
        }
    }

    if (node->status.last_any_rx.time_since_epoch().count() &&
        (now - node->status.last_any_rx) > BUS_IDLE_TIMEOUT_MS) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
    }

    if (node->status.pending_safety_since.time_since_epoch().count()) {
        if ((now - node->status.pending_safety_since) >= SAFETY_DEBOUNCE_MS) {
            node->status.safety = node->status.pending_safety;
            node->status.pending_safety_since = {};
        }
    }

    const bool cp_seen = node->status.cp.last_rx.time_since_epoch().count() != 0;
    const bool cp_stale = cp_seen &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(now - node->status.cp.last_rx) > CP_TIMEOUT_MS);
    const bool plugged = (cp_seen && !cp_stale) ? cp_state_plugged(node->status.cp.state_char) : false;

    st.safety_ok = node->status.safety.safety_ok;
    st.estop = node->status.safety.estop;
    st.earth_fault = node->status.safety.earth_fault;
    st.comm_fault = node->status.safety.comm_fault;
    st.relay_closed = node->status.relay_closed;
    const bool fallback_stale = node->meter_fallback_active &&
        node->last_energy_update.time_since_epoch().count() != 0 &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(now - node->last_energy_update) > METER_TIMEOUT_MS);
    st.meter_stale = (!node->meter_fallback_active && node->status.meter.stale) || fallback_stale;
    st.plugged_in = plugged;
    st.cp_fault = cp_stale;
    st.cp_state = node->status.cp.state_char;
    st.hlc_stage = node->status.hlc_stage;
    st.hlc_cable_check_ok = node->status.hlc_cable_check_ok;
    st.hlc_precharge_active = node->status.hlc_precharge_active;
    st.hlc_charge_complete = node->status.hlc_charge_complete;
    st.hlc_power_ready = node->authorization_granted && node->status.hlc_power_ready;
    st.authorization_granted = node->authorization_granted;
    st.pilot_duty_pct = node->status.cp.duty_pct;
    st.present_voltage_v = node->status.present_voltage_v > 0.0 ? std::optional<double>(node->status.present_voltage_v) : std::nullopt;
    st.present_current_a = std::optional<double>(node->status.present_current_a);
    st.present_power_w = node->status.present_power_w > 0.0 ? std::optional<double>(node->status.present_power_w) : std::nullopt;
    st.target_voltage_v = node->status.target_voltage_v > 0.0 ? std::optional<double>(node->status.target_voltage_v) : std::nullopt;
    st.target_current_a = node->status.target_current_a > 0.0 ? std::optional<double>(node->status.target_current_a) : std::nullopt;
    st.evse_max_voltage_v = node->status.max_voltage_v > 0.0 ? std::optional<double>(node->status.max_voltage_v) : std::nullopt;
    st.evse_max_current_a = node->status.max_current_a > 0.0 ? std::optional<double>(node->status.max_current_a) : std::nullopt;
    st.evse_max_power_kw = node->status.max_power_kw > 0.0 ? std::optional<double>(node->status.max_power_kw) : std::nullopt;
    st.lock_engaged = !node->cfg.require_lock || (node->lock_engaged && node->lock_feedback_engaged);
    if (st.comm_fault && node->cfg.require_lock) {
        st.lock_engaged = false;
    }
    if (node->crc_mode_mismatch) {
        st.safety_ok = false;
        st.comm_fault = true;
    }
    auto choose_limit = [](std::optional<double> telem, double commanded, double cfg) -> std::optional<double> {
        if (telem && *telem > 0.0) return telem;
        if (commanded > 0.0) return commanded;
        if (cfg > 0.0) return cfg;
        return std::nullopt;
    };

    st.evse_max_voltage_v = choose_limit(st.evse_max_voltage_v, node->last_limit_voltage_v, node->cfg.max_voltage_v);
    st.evse_max_current_a = choose_limit(st.evse_max_current_a, node->last_limit_current_a, node->cfg.max_current_a);
    const double cfg_power_kw = node->cfg.max_power_w > 0.0 ? node->cfg.max_power_w / 1000.0 : 0.0;
    st.evse_max_power_kw = choose_limit(st.evse_max_power_kw, node->last_limit_power_kw, cfg_power_kw);
    st.evse_limit_ack_count = node->status.limit_ack_count;
    st.last_evse_limit_ack = node->status.last_limit_ack;
    st.present_stale_events = node->present_stale_events;
    st.limit_stale_events = node->limit_stale_events;
    st.auth_push_count = node->auth_push_count;
    st.last_telemetry = node->status.last_any_rx;
    uint8_t healthy_mask = node->crc_mode_mismatch ? 0x00 : 0x03; // two modules default healthy
    uint8_t fault_mask = node->crc_mode_mismatch ? 0xFF : 0x00;
    if (!node->crc_mode_mismatch) {
        const uint8_t commanded_mask = node->module_mask & 0x06;
        const uint8_t actual_mask = node->status.relay_state_mask & 0x06;
        const bool gun_cmd = gun_relay_owned_by_plc_
                                 ? node->gc_closed_cmd
                                 : ((node->module_mask & 0x01) != 0);
        const bool gun_actual = (node->status.relay_state_mask & 0x01) != 0;
        st.gc_welded = (!gun_relay_owned_by_plc_ && !gun_cmd && gun_actual);
        st.mc_welded = false;
        bool relay_conflict = false;
        if (st.comm_fault || !st.safety_ok || node->status.safety.remote_force_off) {
            healthy_mask = 0x00;
        } else {
            for (int idx = 0; idx < 2; ++idx) {
                const uint8_t bit = static_cast<uint8_t>(1U << (idx + 1));
                const bool commanded_on = (commanded_mask & bit) != 0;
                const bool actual_on = (actual_mask & bit) != 0;
                if (commanded_on && !actual_on) {
                    healthy_mask &= static_cast<uint8_t>(~(1U << idx));
                    fault_mask |= static_cast<uint8_t>(1U << idx);
                    relay_conflict = true;
                } else if (!commanded_on && actual_on) {
                    st.mc_welded = true;
                    healthy_mask &= static_cast<uint8_t>(~(1U << idx));
                    fault_mask |= static_cast<uint8_t>(1U << idx);
                    relay_conflict = true;
                }
            }
        }
        if (relay_conflict) {
            node->relay_conflict_count++;
        }
    }
    st.relay_closed = st.relay_closed && !node->crc_mode_mismatch;
    st.module_healthy_mask = healthy_mask;
    st.module_fault_mask = fault_mask;
    st.present_stale_events = node->present_stale_events;
    st.limit_stale_events = node->limit_stale_events;
    st.auth_push_count = node->auth_push_count;
    st.relay_conflict_count = node->relay_conflict_count;
    return st;
}

void PlcHardware::set_authorization_state(std::int32_t connector, bool authorized) {
    set_authorization_state(connector, authorized ? AuthorizationState::Granted : AuthorizationState::Denied);
}

void PlcHardware::set_authorization_state(std::int32_t connector, AuthorizationState state) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (auto* node = find_node(connector)) {
        node->authorization_state = state;
        node->authorization_granted = (state == AuthorizationState::Granted);
        node->status.hlc_power_ready = derive_hlc_power_ready(node->status, node->authorization_granted);
        const auto now = std::chrono::steady_clock::now();
        refresh_authorization_locked(*node, now, true);
    }
}

std::vector<AuthToken> PlcHardware::poll_auth_tokens() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<AuthToken> tokens;
    tokens.swap(auth_events_);
    return tokens;
}

void PlcHardware::set_evse_limits(std::int32_t connector, const EvseLimits& limits) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (auto* node = find_node(connector)) {
        send_evse_limits(*node, limits);
    }
}

void PlcHardware::publish_evse_present(std::int32_t connector, double voltage_v, double current_a,
                                       double power_kw, bool output_enabled, bool regulating) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (auto* node = find_node(connector)) {
        send_evse_present(*node, voltage_v, current_a, power_kw, output_enabled, regulating);
    }
}

void PlcHardware::publish_fault_state(std::int32_t connector, uint8_t fault_bits) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (auto* node = find_node(connector)) {
        const auto now = std::chrono::steady_clock::now();
        update_fault_bits(*node, fault_bits, now);
    }
}

void PlcHardware::update_fault_bits(Node& node, uint8_t fault_bits,
                                    const std::chrono::steady_clock::time_point& now) {
    node.fault_bits = static_cast<uint8_t>(fault_bits & 0x3F); // 6 bits packed into EVSE_PRESENT flags
    node.last_fault_update = now;
}

void PlcHardware::refresh_authorization_locked(Node& node,
                                               const std::chrono::steady_clock::time_point& now,
                                               bool force) {
    if (node.authorization_state == AuthorizationState::Unknown) {
        return;
    }
    if (!force && node.last_auth_push.time_since_epoch().count() &&
        (now - node.last_auth_push) < AUTH_REFRESH_INTERVAL_MS) {
        return;
    }
    const bool granted = node.authorization_state == AuthorizationState::Granted;
    const bool pending = node.authorization_state == AuthorizationState::Pending;
    send_config_command(node, CONFIG_PARAM_AUTH_STATE, granted ? 1U : 0U);
    send_config_command(node, CONFIG_PARAM_AUTH_PENDING, pending ? 1U : 0U);
    node.last_auth_push = now;
    node.auth_push_count++;
}

bool PlcHardware::send_config_command(Node& node, uint8_t param_id, uint32_t value) {
#ifdef __linux__
    uint8_t data[8] = {};
    data[0] = param_id;
    data[1] = 0x00; // set operation
    data[2] = static_cast<uint8_t>(value & 0xFF);
    data[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[5] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[6] = 0;
    if (use_crc8_) {
        data[7] = plc_crc8(data, 7);
    }
    uint32_t can_id = CONFIG_CMD_BASE | (node.cfg.plc_id & 0x0F);
    return send_frame(can_id | CAN_EFF_FLAG, data, sizeof(data));
#else
    (void)node;
    (void)param_id;
    (void)value;
    return false;
#endif
}

void PlcHardware::ensure_protocol_version(Node& node, const std::chrono::steady_clock::time_point& now) {
#ifdef __linux__
    if (sock_ < 0) {
        return;
    }
    if (node.protocol_version_mismatch) {
        node.status.safety.comm_fault = true;
        node.status.safety.safety_ok = false;
        node.status.pending_safety = node.status.safety;
        node.status.pending_safety_since = now;
        return;
    }
    if (node.protocol_version_ok) {
        return;
    }
    if (node.last_proto_version_sent.time_since_epoch().count() &&
        (now - node.last_proto_version_sent) < PROTO_VERSION_RETRY_MS) {
        return;
    }
    const bool sent = send_config_command(node, CONFIG_PARAM_PROTO_VERSION, CAN_PROTOCOL_VERSION);
    node.last_proto_version_sent = now;
    if (!sent && !node.protocol_version_fault_logged) {
        EVLOG_warning << "Failed to send protocol version handshake to PLC " << node.cfg.plc_id;
        node.protocol_version_fault_logged = true;
    }
#else
    (void)node;
    (void)now;
#endif
}

void PlcHardware::send_evse_limits(Node& node, const EvseLimits& limits) {
#ifdef __linux__
    const auto now = std::chrono::steady_clock::now();
    const double cfg_v = node.cfg.max_voltage_v > 0.0 ? node.cfg.max_voltage_v : 0.0;
    const double cfg_i = node.cfg.max_current_a > 0.0 ? node.cfg.max_current_a : 0.0;
    const double cfg_p = node.cfg.max_power_w > 0.0 ? node.cfg.max_power_w / 1000.0 : 0.0;

    const double v = limits.max_voltage_v.value_or(node.last_limit_voltage_v > 0.0 ? node.last_limit_voltage_v : cfg_v);
    const double i = limits.max_current_a.value_or(node.last_limit_current_a > 0.0 ? node.last_limit_current_a : cfg_i);
    const double p = limits.max_power_kw.value_or(node.last_limit_power_kw > 0.0 ? node.last_limit_power_kw : cfg_p);

    uint8_t data[8] = {};
    const uint16_t v_deciv = clamp_to_deciv(v);
    const uint16_t i_deciv = clamp_to_deciv(i);
    const uint16_t p_decik = clamp_to_deciv(p);
    data[0] = static_cast<uint8_t>(v_deciv & 0xFF);
    data[1] = static_cast<uint8_t>((v_deciv >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(i_deciv & 0xFF);
    data[3] = static_cast<uint8_t>((i_deciv >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(p_decik & 0xFF);
    data[5] = static_cast<uint8_t>((p_decik >> 8) & 0xFF);
    data[6] = 0;
    if (use_crc8_) {
        data[7] = plc_crc8(data, 7);
    }
    const uint32_t can_id = EVSE_DC_MAX_LIMITS_CMD_BASE | static_cast<uint32_t>(node.cfg.plc_id & 0x0F);
    const bool ok = send_frame(can_id | CAN_EFF_FLAG, data, sizeof(data));
    if (ok) {
        node.limit_tx_count++;
        node.last_limit_voltage_v = v_deciv / 10.0;
        node.last_limit_current_a = i_deciv / 10.0;
        node.last_limit_power_kw = p_decik / 10.0;
        node.status.max_voltage_v = node.last_limit_voltage_v;
        node.status.max_current_a = node.last_limit_current_a;
        node.status.max_power_kw = node.last_limit_power_kw;
        if (node.last_evse_limits_tx.time_since_epoch().count()) {
            const auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - node.last_evse_limits_tx).count();
            if (gap_ms > limits_warn_ms_) {
                node.limit_stale_events++;
                if (!cadence_warn_logged_) {
                    EVLOG_warning << "EVSE limit publish gap " << gap_ms << "ms to PLC " << node.cfg.plc_id;
                    cadence_warn_logged_ = true;
                }
            }
        }
        node.last_evse_limits_tx = now;
    } else {
        node.limit_tx_fail++;
        EVLOG_warning << "Failed to send EVSE limits to PLC " << node.cfg.plc_id
                      << " (tx_fail=" << node.limit_tx_fail << ", sent=" << node.limit_tx_count << ")";
    }
#else
    (void)node;
    (void)limits;
#endif
}

void PlcHardware::send_evse_present(Node& node, double voltage_v, double current_a, double power_kw,
                                    bool output_enabled, bool regulating) {
#ifdef __linux__
    const auto now = std::chrono::steady_clock::now();
    uint8_t data[8] = {};
    const uint16_t v_deciv = clamp_to_deciv(voltage_v);
    const uint16_t i_deciv = clamp_to_deciv(current_a);
    const uint16_t p_decik = clamp_to_deciv(power_kw);
    data[0] = static_cast<uint8_t>(v_deciv & 0xFF);
    data[1] = static_cast<uint8_t>((v_deciv >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(i_deciv & 0xFF);
    data[3] = static_cast<uint8_t>((i_deciv >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(p_decik & 0xFF);
    data[5] = static_cast<uint8_t>((p_decik >> 8) & 0xFF);
    uint8_t flags = 0;
    if (output_enabled) flags |= 0x01;
    if (regulating) flags |= 0x02;
    const uint8_t faults = static_cast<uint8_t>(node.fault_bits & 0x3F);
    flags |= static_cast<uint8_t>(faults << 2);
    data[6] = flags;
    if (use_crc8_) {
        data[7] = plc_crc8(data, 7);
    }
    const uint32_t can_id = EVSE_DC_PRESENT_CMD_BASE | static_cast<uint32_t>(node.cfg.plc_id & 0x0F);
    (void)send_frame(can_id | CAN_EFF_FLAG, data, sizeof(data));
    node.status.present_voltage_v = voltage_v;
    node.status.present_current_a = current_a;
    node.status.present_power_w = power_kw * 1000.0;
    if (node.last_evse_present_tx.time_since_epoch().count()) {
        const auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - node.last_evse_present_tx).count();
        if (gap_ms > present_warn_ms_) {
            node.present_stale_events++;
            if (!cadence_warn_logged_) {
                EVLOG_warning << "EVSE present publish gap " << gap_ms << "ms to PLC " << node.cfg.plc_id;
                cadence_warn_logged_ = true;
            }
        }
    }
    node.last_evse_present_tx = now;
#else
    (void)node;
    (void)voltage_v;
    (void)current_a;
    (void)power_kw;
    (void)output_enabled;
    (void)regulating;
#endif
}

bool PlcHardware::send_relay_command(Node& node, bool close, bool force_all_off) {
#ifdef __linux__
    uint8_t data[8] = {};
    data[0] = 0;
    if (close) {
        data[0] |= (node.module_mask & 0x07); // RLY1/2/3_CMD bits
    }
    data[0] |= (1 << 3);                 // SYS_ENABLE
    if (force_all_off) data[0] |= (1 << 4);
    const bool clear_faults = node.status.last_fault_reason != 0;
    if (clear_faults) data[0] |= (1 << 5); // CLEAR_FAULTS
    const uint8_t seq = node.cmd_seq++;
    data[1] = seq;
    // Enable mask for relays present
    data[2] = node.module_mask & 0x07; // RLY1/2/3_ENABLE
    // CMD_MODE (steady)
    data[3] = 0x00;
    // PULSE_MS (steady)
    data[4] = 0x00;
    data[5] = 0x00;
    if (use_crc8_) {
        data[7] = plc_crc8(data, 7);
    }
    uint32_t can_id = 0x0300 | ((0x4 << 4) | (node.cfg.plc_id & 0x0F));
    const auto ok = send_frame(can_id | CAN_EFF_FLAG, data, sizeof(data));
    if (ok) {
        node.expected_cmd_seq = seq;
        node.awaiting_ack = true;
        node.last_cmd_sent = std::chrono::steady_clock::now();
        node.retry_count = 0;
        node.last_cmd_close = close;
        node.last_force_all_off = force_all_off;
        // Mirror command on GCMC path for firmware that consumes it.
        if (!send_gcmc_command(node, seq, close, force_all_off)) {
            EVLOG_warning << "Failed to send GCMC command to PLC " << node.cfg.plc_id;
        }
    }
    return ok;
#else
    (void)node;
    (void)close;
    (void)force_all_off;
    return false;
#endif
}

bool PlcHardware::send_frame(uint32_t can_id, const uint8_t* data, size_t len) {
#ifdef __linux__
    if (sock_ < 0) return false;
    struct can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = static_cast<uint8_t>(len);
    std::memcpy(frame.data, data, std::min<size_t>(len, sizeof(frame.data)));
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto written = write(sock_, &frame, sizeof(frame));
        if (written == sizeof(frame)) {
            return true;
        }
        if (errno == ENOBUFS || errno == EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5 * (attempt + 1)));
            continue;
        }
        EVLOG_warning << "CAN write failed (id=0x" << std::hex << can_id << std::dec << "): "
                      << std::strerror(errno);
        break;
    }
    return false;
#else
    (void)can_id;
    (void)data;
    (void)len;
    return false;
#endif
}

void PlcHardware::rx_loop() {
#ifdef __linux__
    while (running_) {
        if (restart_requested_) {
            restart_can();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (sock_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        struct can_frame frame {};
        const auto nbytes = read(sock_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            continue;
        }
        if (frame.can_id & CAN_ERR_FLAG) {
            handle_error_frame(frame);
            continue;
        }
        if ((frame.can_id & CAN_EFF_FLAG) == 0) {
            continue;
        }
        handle_frame(frame.can_id & CAN_EFF_MASK, frame.data, frame.can_dlc);
    }
#endif
}

void PlcHardware::handle_frame(uint32_t can_id, const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (nodes_.empty()) return;

    const int plc_id = static_cast<int>(can_id & 0x0F);
    auto* node = find_node_by_plc(plc_id);
    if (!node) return;
    const auto rx_time = std::chrono::steady_clock::now();
    auto mark_crc_mismatch = [&](const char* reason) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
        node->crc_mode_mismatch = true;
        node->status.pending_safety = node->status.safety;
        node->status.pending_safety_since = rx_time;
        if (!node->crc_mode_mismatch_logged) {
            EVLOG_error << "CRC mode mismatch on PLC " << node->cfg.plc_id << ": " << reason;
            node->crc_mode_mismatch_logged = true;
        }
    };
    auto mark_crc_fault = [&](const char* reason) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
        node->status.pending_safety = node->status.safety;
        node->status.pending_safety_since = rx_time;
        if (!node->crc_fault_logged) {
            EVLOG_error << "CRC fault on PLC " << node->cfg.plc_id << ": " << reason;
            node->crc_fault_logged = true;
        }
    };

    const bool crc_frame = crc_expected(can_id);
    if (crc_frame && len != 8) {
        mark_crc_mismatch("expected CRC8-protected frame with dlc=8");
        return;
    }
    // CRC check based on DBC (only specific frames carry CRC).
    if (use_crc8_ && crc_frame) {
        const uint8_t expected = plc_crc8(data, 7);
        if (expected != data[7]) {
            mark_crc_fault("CRC8 verification failed");
            return;
        }
    }
    node->crc_fault_logged = false;
    node->status.last_any_rx = rx_time;
    prune_segment_buffers(*node, rx_time);

    if ((can_id & PLC_TX_MASK) == (RELAY_STATUS_BASE & PLC_TX_MASK)) {
        handle_relay_status(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (SAFETY_STATUS_BASE & PLC_TX_MASK)) {
        handle_safety_status(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (ENERGY_METER_BASE & PLC_TX_MASK)) {
        handle_meter(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CP_LEVELS_BASE & PLC_TX_MASK)) {
        handle_cp_levels(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CHARGING_SESSION_BASE & PLC_TX_MASK)) {
        handle_charging_session(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVAC_CTRL_BASE & PLC_TX_MASK)) {
        handle_evac_control(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVDC_TARGETS_BASE & PLC_TX_MASK)) {
        handle_evdc_targets(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVDC_MAX_LIMITS_BASE & PLC_TX_MASK) ||
               (can_id & PLC_TX_MASK) == (EVDC_ENERGY_LIMITS_BASE & PLC_TX_MASK)) {
        handle_evdc_limits(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EV_STATUS_DISPLAY_BASE & PLC_TX_MASK)) {
        handle_ev_status_display(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CHARGE_INFO_BASE & PLC_TX_MASK)) {
        handle_charge_info(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (GCMC_STATUS_BASE & PLC_TX_MASK)) {
        handle_gcmc_status(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (HW_STATUS_BASE & PLC_TX_MASK)) {
        handle_hw_status(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (DEBUG_INFO_BASE & PLC_TX_MASK)) {
        handle_debug_info(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (BOOT_CONFIG_BASE & PLC_TX_MASK)) {
        handle_boot_config(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (RTEVLOG_BASE & PLC_TX_MASK)) {
        handle_rtev_log(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (RTTLOG_BASE & PLC_TX_MASK)) {
        handle_rtt_log(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (SOFTWARE_INFO_BASE & PLC_TX_MASK)) {
        handle_software_info(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (ERROR_CODES_BASE & PLC_TX_MASK)) {
        handle_error_codes(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (HW_CONFIG_BASE & PLC_TX_MASK)) {
        handle_hw_config(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (RFID_EVENT_BASE & PLC_TX_MASK)) {
        handle_rfid_event(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVCCID_BASE & PLC_TX_MASK)) {
        handle_identity_segment(*node, node->evccid, AuthTokenSource::Autocharge, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EMAID0_BASE & PLC_TX_MASK)) {
        handle_identity_segment(*node, node->emaid0, AuthTokenSource::Autocharge, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EMAID1_BASE & PLC_TX_MASK)) {
        handle_identity_segment(*node, node->emaid1, AuthTokenSource::Autocharge, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVMAC_BASE & PLC_TX_MASK)) {
        handle_identity_segment(*node, node->evmac, AuthTokenSource::Autocharge, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CONFIG_ACK_BASE & PLC_TX_MASK)) {
        if (len >= 6) {
            const uint8_t param_id = data[0];
            const uint8_t status = data[1];
            const uint32_t value = le_u32(&data[2]);
            EVLOG_info << "PLC " << node->cfg.plc_id << " ConfigAck param=" << static_cast<int>(param_id)
                       << " status=" << static_cast<int>(status) << " value=0x" << std::hex << value << std::dec;
            if (status == 0 && param_id == CONFIG_PARAM_EVSE_LIMIT_ACK) {
                node->status.limit_ack_count = value;
                node->status.last_limit_ack = rx_time;
            } else if (param_id == CONFIG_PARAM_PROTO_VERSION) {
                if (status == 0 && value == CAN_PROTOCOL_VERSION) {
                    node->protocol_version_ok = true;
                    node->protocol_version_mismatch = false;
                    node->protocol_version_fault_logged = false;
                    node->last_proto_ack = rx_time;
                } else {
                    node->protocol_version_ok = false;
                    node->protocol_version_mismatch = true;
                    node->status.safety.comm_fault = true;
                    node->status.safety.safety_ok = false;
                    node->status.pending_safety = node->status.safety;
                    node->status.pending_safety_since = rx_time;
                    if (!node->protocol_version_fault_logged) {
                        EVLOG_error << "PLC " << node->cfg.plc_id << " protocol version mismatch: expected "
                                    << CAN_PROTOCOL_VERSION << " got " << value << " status="
                                    << static_cast<int>(status);
                        node->protocol_version_fault_logged = true;
                    }
                }
            }
        }
    } else {
        if (unknown_can_ids_.insert(can_id).second) {
            EVLOG_debug << "Unhandled PLC CAN frame id=0x" << std::hex << can_id << std::dec
                        << " len=" << static_cast<int>(len);
        }
    }
}

void PlcHardware::handle_relay_status(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    node.status.relay_state_mask = data[0] & 0x07;
    node.status.relay_closed = (data[0] & 0x01) != 0;
    PlcSafetyStatus s = node.status.safety;
    s.safety_ok = (data[2] & 0x10) != 0;    // bit20
    s.earth_fault = (data[2] & 0x20) != 0;  // bit21
    s.estop = (data[2] & 0x80) != 0;        // bit23
    node.status.last_cmd_seq_applied = data[1];
    node.status.last_cmd_seq_received = data[1];
    s.comm_fault = (data[0] & 0x80) != 0;   // bit7
    s.remote_force_off = false;
    const uint8_t fault_reason = data[3];
    node.status.last_fault_reason = fault_reason;
    if (fault_reason != 0) {
        s.safety_ok = false;
        if (fault_reason == 6 || fault_reason == 5 || fault_reason == 8) {
            s.comm_fault = true; // CAN_BUS_OFF, RELAYCTRL_TIMEOUT, CRC_FAIL
        }
        if (fault_reason == 7) {
            s.remote_force_off = true;
        }
        if (fault_reason == 10 || fault_reason == 12) {
            s.earth_fault = true;
        }
    }
    const bool lock_input = derive_lock_input(node, data[2], true);
    node.lock_feedback_engaged = lock_input;
    node.lock_engaged = node.cfg.require_lock ? lock_input : true;
    // Update pending safety to debounce
    node.status.pending_safety = s;
    node.status.pending_safety_since = std::chrono::steady_clock::now();
    if (node.awaiting_ack && node.expected_cmd_seq == node.status.last_cmd_seq_applied) {
        node.awaiting_ack = false;
        node.retry_count = 0;
    }
    node.status.last_relay_status = std::chrono::steady_clock::now();
}

void PlcHardware::handle_safety_status(Node& node, const uint8_t* data, size_t len) {
    if (len < 2) return;
    PlcSafetyStatus s = node.status.safety;
    s.safety_ok = (data[0] & 0x10) != 0;
    s.earth_fault = (data[0] & 0x20) != 0;
    s.estop = (data[0] & 0x08) != 0;
    s.comm_fault = false; // explicit status frame indicates comm alive
    const bool lock_input = derive_lock_input(node, data[0], false);
    node.lock_feedback_engaged = lock_input;
    node.lock_engaged = node.cfg.require_lock ? lock_input : true;
    node.status.pending_safety = s;
    node.status.pending_safety_since = std::chrono::steady_clock::now();
    node.status.last_safety_status = std::chrono::steady_clock::now();
}

void PlcHardware::handle_meter(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    const uint8_t mux = data[0];
    auto& meter = node.status.meter;
    const bool meter_ok = (data[1] & 0x01) != 0;
    const bool comm_error = (data[1] & 0x02) != 0;
    const bool data_stale = (data[1] & 0x04) != 0;
    const bool overrange = (data[1] & 0x08) != 0;
    const bool fallback_active = (data[1] & 0x10) != 0;
    if (mux == 0) {
        const uint16_t v = le_u16(&data[2]);
        const int16_t i = le_i16(&data[4]);
        const int16_t p = le_i16(&data[6]);
        meter.voltage_v = v * 0.1;
        meter.current_a = i * 0.01;
        meter.power_w = p * 10.0;
        meter.ok = meter_ok && !overrange && !fallback_active;
        node.status.present_voltage_v = meter.voltage_v;
        node.status.present_current_a = meter.current_a;
        node.status.present_power_w = meter.power_w;
    } else if (mux == 1) {
        const uint32_t e = le_u32(&data[2]);
        const uint16_t f = le_u16(&data[6]);
        meter.energy_Wh = static_cast<double>(e) * 100.0; // 0.1 kWh -> Wh
        meter.freq_hz = f * 0.01;
        meter.ok = meter_ok && !overrange && !fallback_active;
    }
    meter.stale = data_stale || fallback_active;
    node.meter_fallback_active = fallback_active;
    node.status.safety.comm_fault = comm_error;
    node.status.last_meter_rx = std::chrono::steady_clock::now();
    if (comm_error) {
        node.status.safety.safety_ok = false;
    }
}

void PlcHardware::handle_cp_levels(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    const uint16_t mv_robust = le_u16(&data[0]);
    const char cp_state_char = static_cast<char>(data[2]);
    const uint8_t duty = data[3];
    const uint16_t mv_peak = le_u16(&data[4]);
    const uint16_t mv_min = le_u16(&data[6]);
    update_cp_status(node, cp_state_char, duty, mv_peak, mv_min, true, mv_robust, true);
}

void PlcHardware::handle_charging_session(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    node.status.session_epoch = le_u32(&data[0]);
    const char cp_state_char = static_cast<char>(data[4]);
    const uint8_t duty = data[5];
    update_cp_status(node, cp_state_char, duty, 0, 0, false, 0, false);
    uint8_t stage = node.status.hlc_stage;
    if (len >= 7) {
        stage = data[6];
    }
    bool auth_pending = false;
    if (len >= 8) {
        auth_pending = (data[7] & 0x01) != 0;
    }
    node.status.auth_pending_flag = auth_pending;
    uint8_t flags = 0;
    if (node.status.hlc_charge_complete) flags |= 0x01;
    if (node.status.hlc_precharge_active) flags |= 0x02;
    if (node.status.hlc_cable_check_ok) flags |= 0x04;
    if (node.authorization_granted) flags |= 0x08;
    if (auth_pending) flags |= 0x10;
    update_hlc_state(node, stage, flags);
}

void PlcHardware::handle_evac_control(Node& node, const uint8_t* data, size_t len) {
    if (len < 2) return;
    const uint8_t duty = data[0];
    const char cp_state_char = static_cast<char>(data[1]);
    uint16_t tgt_v_0p1 = 0;
    uint16_t tgt_i_0p1 = 0;
    uint16_t pres_i_0p1 = 0;
    if (len >= 6) {
        tgt_v_0p1 = le_u16(&data[2]);
        tgt_i_0p1 = le_u16(&data[4]);
    }
    if (len >= 8) {
        pres_i_0p1 = le_u16(&data[6]);
    }
    node.status.target_voltage_v = tgt_v_0p1 * 0.1;
    node.status.target_current_a = tgt_i_0p1 * 0.1;
    node.status.present_current_a = pres_i_0p1 * 0.1;
    update_cp_status(node, cp_state_char, duty, 0, 0, false);
}

void PlcHardware::handle_evdc_targets(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    const uint16_t tgt_v = le_u16(&data[0]);
    const uint16_t tgt_i = le_u16(&data[2]);
    const uint16_t pres_v = le_u16(&data[4]);
    const uint16_t pres_i = le_u16(&data[6]);
    node.status.target_voltage_v = tgt_v * 0.1;
    node.status.target_current_a = tgt_i * 0.1;
    node.status.present_voltage_v = pres_v * 0.1;
    node.status.present_current_a = pres_i * 0.1;
    node.status.present_power_w = node.status.present_voltage_v * node.status.present_current_a;
}

void PlcHardware::handle_evdc_limits(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    const uint16_t max_v = le_u16(&data[0]);
    const uint16_t max_i = le_u16(&data[2]);
    const uint16_t max_p = le_u16(&data[4]);
    node.status.max_voltage_v = max_v * 0.1;
    node.status.max_current_a = max_i * 0.1;
    node.status.max_power_kw = max_p * 0.1;
}

void PlcHardware::handle_ev_status_display(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    const uint16_t pres_v = le_u16(&data[0]);
    const uint16_t pres_i = le_u16(&data[2]);
    const uint8_t stage = data[4];
    const uint8_t duty = data[5];
    const char cp_state_char = (len >= 7) ? static_cast<char>(data[6]) : node.status.cp.state_char;
    node.status.present_voltage_v = pres_v * 0.1;
    node.status.present_current_a = pres_i * 0.1;
    node.status.present_power_w = node.status.present_voltage_v * node.status.present_current_a;
    update_cp_status(node, cp_state_char, duty, 0, 0, false);
    uint8_t flags = 0;
    if (node.status.hlc_charge_complete) flags |= 0x01;
    if (node.status.hlc_precharge_active) flags |= 0x02;
    if (node.status.hlc_cable_check_ok) flags |= 0x04;
    update_hlc_state(node, stage, flags);
}

void PlcHardware::handle_charge_info(Node& node, const uint8_t* data, size_t len) {
    if (len < 2) return;
    const uint8_t stage = data[0];
    const uint8_t flags = data[1];
    const bool auth_granted = (flags & 0x08) != 0;
    const bool auth_pending = (flags & 0x10) != 0;
    const bool lock_engaged = (flags & 0x20) != 0;
    node.authorization_granted = auth_granted;
    node.status.auth_pending_flag = auth_pending;
    node.lock_feedback_engaged = lock_engaged;
    node.lock_engaged = node.cfg.require_lock ? lock_engaged : true;
    update_hlc_state(node, stage, flags);
}

void PlcHardware::handle_gcmc_status(Node& node, const uint8_t* data, size_t len) {
    if (len < 7) return;
    const uint8_t cmd_bits = data[0];
    const uint8_t fb_bits = data[1];
    const uint8_t fault_reason = data[2];
    const uint8_t comm_fault = data[3];
    const uint8_t safety_active = data[4];
    const uint8_t last_applied = data[5];
    const uint8_t cmd_seq_rx = data[6];

    uint8_t mask = 0;
    if (fb_bits & 0x01) mask |= 0x01; // GC
    if (fb_bits & 0x02) mask |= 0x02; // MC
    if (fb_bits & 0x04) mask |= 0x04; // MN0/module
    node.status.relay_state_mask = mask;
    node.status.relay_closed = (mask & 0x01) != 0;
    PlcSafetyStatus s = node.status.safety;
    s.comm_fault = comm_fault != 0;
    s.remote_force_off = false;
    s.safety_ok = !safety_active;
    if (fault_reason != 0) {
        s.safety_ok = false;
        if (fault_reason == 6 || fault_reason == 5 || fault_reason == 8) {
            s.comm_fault = true;
        }
        if (fault_reason == 7) {
            s.remote_force_off = true;
        }
        if (fault_reason == 10 || fault_reason == 12) {
            s.earth_fault = true;
        }
    }
    node.status.last_fault_reason = fault_reason;
    node.status.pending_safety = s;
    const auto now = std::chrono::steady_clock::now();
    node.status.pending_safety_since = now;
    node.status.last_cmd_seq_applied = last_applied;
    node.status.last_cmd_seq_received = cmd_seq_rx;
    node.status.rx_count = std::max<uint32_t>(node.status.rx_count, static_cast<uint32_t>(cmd_seq_rx));
    if (node.awaiting_ack && node.expected_cmd_seq == node.status.last_cmd_seq_applied) {
        node.awaiting_ack = false;
        node.retry_count = 0;
    }
    node.status.last_any_rx = now;
    (void)cmd_bits;
}

void PlcHardware::handle_hw_status(Node& node, const uint8_t* data, size_t len) {
    if (len < 7) return;
    const uint8_t tec = data[0];
    const uint8_t rec = data[1];
    const bool bus_off = data[2] != 0;
    node.status.uptime_s = static_cast<uint32_t>(le_u16(&data[5]));
    node.status.rx_count = static_cast<uint32_t>(data[3]);
    node.status.tx_count = static_cast<uint32_t>(data[4]);
    if (bus_off) {
        node.status.safety.comm_fault = true;
        node.status.safety.safety_ok = false;
        node.status.relay_closed = false;
        if (!node.bus_off_logged) {
            EVLOG_error << "CAN controller bus-off (TEC=" << static_cast<int>(tec)
                        << ", REC=" << static_cast<int>(rec) << ") on PLC " << node.cfg.plc_id;
            node.bus_off_logged = true;
        }
    } else {
        node.bus_off_logged = false;
    }
    node.status.last_any_rx = std::chrono::steady_clock::now();
    if (bus_off) {
        restart_requested_ = true;
    }
}

void PlcHardware::handle_debug_info(Node& node, const uint8_t* data, size_t len) {
    if (len < 4) return;
    const uint8_t cmd_seq_tx = data[0];
    const uint8_t last_applied = data[1];
    const uint8_t relay_fault = data[2];
    const bool bus_off = data[3] != 0;
    if (bus_off) {
        node.status.safety.comm_fault = true;
        node.status.safety.safety_ok = false;
    }
    node.status.last_cmd_seq_applied = last_applied;
    node.status.last_cmd_seq_received = cmd_seq_tx;
    node.status.rx_count = std::max<uint32_t>(node.status.rx_count, static_cast<uint32_t>(cmd_seq_tx));
    if (node.awaiting_ack && node.expected_cmd_seq == node.status.last_cmd_seq_applied) {
        node.awaiting_ack = false;
        node.retry_count = 0;
    }
    if (relay_fault != 0) {
        node.status.last_fault_reason = relay_fault;
        node.status.safety.safety_ok = false;
    }
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::handle_boot_config(Node& node, const uint8_t* data, size_t len) {
    if (len < 4) return;
    const uint8_t fw_major = data[0];
    const uint8_t fw_minor = data[1];
    const uint8_t fw_patch = data[2];
    const uint8_t features = data[3];
    const bool relays = (features & 0x01) != 0;
    const bool safety = (features & 0x02) != 0;
    if (!relays || !safety) {
        EVLOG_warning << "PLC " << node.cfg.plc_id << " reports missing features (relays=" << relays
                      << ", safety=" << safety << ") fw=" << static_cast<int>(fw_major) << "."
                      << static_cast<int>(fw_minor) << "." << static_cast<int>(fw_patch);
    }
    node.status.fw_major = fw_major;
    node.status.fw_minor = fw_minor;
    node.status.fw_patch = fw_patch;
    node.status.feature_flags = features;
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::handle_rtev_log(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    node.status.rx_count = le_u32(&data[0]);
    node.status.tx_count = le_u32(&data[4]);
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::handle_rtt_log(Node& node, const uint8_t* data, size_t len) {
    if (len < 4) return;
    node.status.uptime_s = le_u32(&data[0]);
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::handle_software_info(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    node.status.fw_major = data[5];
    node.status.fw_minor = data[6];
    node.status.fw_patch = data[7];
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::handle_error_codes(Node& node, const uint8_t* data, size_t len) {
    if (len < 1) return;
    node.status.error_code = data[0];
    if (node.status.error_code != 0) {
        EVLOG_error << "PLC " << node.cfg.plc_id << " error code " << static_cast<int>(node.status.error_code);
        node.status.safety.safety_ok = false;
    }
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::handle_hw_config(Node& node, const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    node.status.last_any_rx = std::chrono::steady_clock::now();
}

void PlcHardware::prune_segment_buffers(Node& node, const std::chrono::steady_clock::time_point& now) {
    auto stale = [&](const SegmentBuffer& buf) {
        return buf.last_rx.time_since_epoch().count() != 0 && (now - buf.last_rx) > SEGMENT_TTL;
    };
    for (auto it = node.rfid_events.begin(); it != node.rfid_events.end();) {
        if (stale(it->second)) {
            it = node.rfid_events.erase(it);
        } else {
            ++it;
        }
    }
    auto prune_single = [&](SegmentBuffer& buf) {
        if (stale(buf)) {
            buf = SegmentBuffer{};
        }
    };
    prune_single(node.evccid);
    prune_single(node.emaid0);
    prune_single(node.emaid1);
    prune_single(node.evmac);
}

void PlcHardware::handle_rfid_event(Node& node, const uint8_t* data, size_t len) {
    if (len < 3) return;
    const uint8_t len_nibble = static_cast<uint8_t>(data[0] & 0x0F);
    const uint8_t event_type = static_cast<uint8_t>((data[0] >> 4) & 0x0F);
    // Only handle UID events (type=0)
    if (event_type != 0 || len_nibble == 0) {
        return;
    }
    const uint8_t event_id = data[1];
    const uint8_t seg_idx = static_cast<uint8_t>(data[2] & 0x0F);
    const uint8_t seg_cnt_raw = static_cast<uint8_t>((data[2] >> 4) & 0x0F);
    const uint8_t expected_segments = std::max<uint8_t>(static_cast<uint8_t>(1), seg_cnt_raw);
    const auto now = std::chrono::steady_clock::now();
    auto& buf = node.rfid_events[event_id];
    const bool need_reset = buf.total_len != len_nibble || buf.expected_segments != expected_segments ||
        seg_idx == 0 || buf.data.size() < len_nibble;
    if (need_reset) {
        buf.reset(len_nibble, expected_segments, now);
    } else {
        buf.last_rx = now;
    }
    if (seg_idx >= buf.expected_segments) {
        buf.received.resize(seg_idx + 1, false);
    }
    const size_t payload_len = len > 3 ? std::min<size_t>(5, len - 3) : 0;
    const size_t offset = static_cast<size_t>(seg_idx) * 5;
    if (buf.data.size() < buf.total_len) {
        buf.data.resize(buf.total_len, 0);
    }
    for (size_t i = 0; i < payload_len && (offset + i) < buf.data.size(); ++i) {
        buf.data[offset + i] = data[3 + i];
    }
    if (seg_idx < buf.received.size()) {
        buf.received[seg_idx] = true;
    }
    if (buf.complete()) {
        AuthToken token;
        token.id_token = bytes_to_hex(buf.data, buf.total_len);
        token.source = AuthTokenSource::RFID;
        token.connector_hint = node.cfg.id;
        token.received_at = now;
        auth_events_.push_back(std::move(token));
        node.rfid_events.erase(event_id);
    }
}

void PlcHardware::handle_identity_segment(Node& node, SegmentBuffer& buffer, AuthTokenSource source,
                                          const uint8_t* data, size_t len) {
    if (len < 3) return;
    const uint8_t total_len = data[0];
    const uint8_t seg_cnt = data[1];
    const uint8_t seg_idx = data[2];
    if (total_len == 0) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const uint8_t expected_segments = std::max<uint8_t>(static_cast<uint8_t>(1), seg_cnt);
    const bool need_reset = buffer.total_len != total_len || buffer.expected_segments != expected_segments ||
        seg_idx == 0 || buffer.data.size() < total_len;
    if (need_reset) {
        buffer.reset(total_len, expected_segments, now);
    } else {
        buffer.last_rx = now;
    }
    if (seg_idx >= buffer.expected_segments) {
        buffer.received.resize(seg_idx + 1, false);
    }
    const size_t payload_len = len > 3 ? std::min<size_t>(5, len - 3) : 0;
    const size_t offset = static_cast<size_t>(seg_idx) * 5;
    if (buffer.data.size() < buffer.total_len) {
        buffer.data.resize(buffer.total_len, 0);
    }
    for (size_t i = 0; i < payload_len && (offset + i) < buffer.data.size(); ++i) {
        buffer.data[offset + i] = data[3 + i];
    }
    if (seg_idx < buffer.received.size()) {
        buffer.received[seg_idx] = true;
    }
    if (buffer.complete()) {
        AuthToken token;
        token.id_token = bytes_to_hex(buffer.data, buffer.total_len);
        token.source = source;
        token.connector_hint = node.cfg.id;
        token.received_at = now;
        auth_events_.push_back(std::move(token));
        buffer = SegmentBuffer{};
    }
}

bool PlcHardware::derive_lock_input(const Node& node, uint8_t sw_mask_byte, bool relay_status_frame) const {
    const int idx = node.cfg.lock_input_switch;
    if (idx < 1 || idx > 4) return true;
    (void)relay_status_frame;
    const int bit = (idx == 4) ? 6 : (idx - 1);
    return (sw_mask_byte & (1 << bit)) != 0;
}

bool PlcHardware::derive_hlc_power_ready(const PlcStatus& status, bool authorized) const {
    if (!authorized) {
        return false;
    }
    if (status.hlc_charge_complete) {
        return false;
    }
    if (status.hlc_stage >= HLC_POWER_DELIVERY_STAGE && (status.hlc_cable_check_ok || !status.hlc_precharge_active)) {
        return true;
    }
    return false;
}

void PlcHardware::update_hlc_state(Node& node, uint8_t stage, uint8_t flags) {
    node.status.hlc_stage = stage;
    node.status.hlc_charge_complete = (flags & 0x01) != 0;
    node.status.hlc_precharge_active = (flags & 0x02) != 0;
    node.status.hlc_cable_check_ok = (flags & 0x04) != 0;
    const bool auth_granted = (flags & 0x08) != 0;
    const bool auth_pending = (flags & 0x10) != 0;
    const bool lock_feedback = (flags & 0x20) != 0;
    node.authorization_granted = auth_granted;
    node.status.auth_pending_flag = auth_pending;
    node.lock_feedback_engaged = lock_feedback;
    node.lock_engaged = node.cfg.require_lock ? lock_feedback : true;
    node.status.hlc_power_ready = derive_hlc_power_ready(node.status, node.authorization_granted);
}

bool PlcHardware::supports_cross_slot_islands() const {
    return false;
}

void PlcHardware::update_cp_status(Node& node, char cp_state_char, uint8_t duty_pct, uint16_t mv_peak,
                                   uint16_t mv_min, bool has_mv, uint16_t mv_robust, bool has_robust) {
    auto& cp = node.status.cp;
    if (cp_state_char == 0) {
        cp_state_char = 'U';
    }
    cp.state_char = cp_state_char;
    cp.duty_pct = static_cast<double>(duty_pct);
    if (has_mv) {
        cp.mv_peak = mv_peak;
        cp.mv_min = mv_min;
    }
    if (has_robust) {
        cp.mv_robust = mv_robust;
    }
    cp.valid = true;
    cp.last_rx = std::chrono::steady_clock::now();
}

#ifdef __linux__
void PlcHardware::handle_error_frame(const struct can_frame& frame) {
    const uint32_t err = frame.can_id & CAN_ERR_MASK;
    const bool bus_off = (err & CAN_ERR_BUSOFF) != 0;
    const bool restarted = (err & CAN_ERR_RESTARTED) != 0;
    const bool ctrl_err = (err & CAN_ERR_CRTL) != 0;

    if (bus_off || ctrl_err) {
        EVLOG_error << "CAN error frame received (bus_off=" << bus_off << ", ctrl=" << ctrl_err << ")";
        for (auto& kv : nodes_) {
            kv.second.status.safety.comm_fault = true;
            kv.second.status.safety.safety_ok = false;
            kv.second.status.relay_closed = false;
        }
    }
    if (restarted) {
        EVLOG_info << "CAN controller restarted, clearing comm faults";
        for (auto& kv : nodes_) {
            kv.second.status.safety.comm_fault = false;
        }
    }
}
#endif

bool PlcHardware::send_gcmc_command(Node& node, uint8_t seq, bool close, bool force_all_off) {
#ifdef __linux__
    uint8_t data[8] = {};
    uint8_t cmd_bits = 0;
    if (close && (node.module_mask & 0x01)) cmd_bits |= 0x01; // GC_CMD
    if (node.mc_closed_cmd) cmd_bits |= 0x02;                 // MC_CMD
    if (close && (node.module_mask & 0x02)) cmd_bits |= 0x04; // MN0_CMD (module 1)
    if (close && (node.module_mask & 0x04)) cmd_bits |= 0x08; // MN1_CMD (module 2)
    data[0] = cmd_bits;
    data[1] = seq;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    if (force_all_off) data[0] |= (1 << 7);
    const bool clear_faults = node.status.last_fault_reason != 0;
    if (clear_faults) data[0] |= (1 << 6);
    if (use_crc8_) {
        data[7] = plc_crc8(data, 7);
    }
    uint32_t can_id = GCMC_CMD_BASE | static_cast<uint32_t>(node.cfg.plc_id & 0x0F);
    return send_frame(can_id | CAN_EFF_FLAG, data, sizeof(data));
#else
    (void)node;
    (void)seq;
    (void)close;
    (void)force_all_off;
    return false;
#endif
}

} // namespace charger
