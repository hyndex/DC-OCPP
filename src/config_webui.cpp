// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"

#include "httplib.h"
#include "webview/webview.h"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <iostream>
#include <vector>
#include <system_error>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <cctype>
#ifdef HAVE_DNSSD
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif
#endif

#include <nlohmann/json.hpp>

namespace {
namespace fs = std::filesystem;
using json = nlohmann::json;

#ifndef CONFIG_WEBUI_ASSET_DIR
#define CONFIG_WEBUI_ASSET_DIR "."
#endif

#ifndef CONFIG_WEBUI_DEFAULT_CONFIG
#define CONFIG_WEBUI_DEFAULT_CONFIG "configs/charger.json"
#endif

struct AppOptions {
    fs::path config_path{CONFIG_WEBUI_DEFAULT_CONFIG};
    fs::path asset_dir{CONFIG_WEBUI_ASSET_DIR};
    int port{8844};
    bool headless{false};
    std::string bind_host{"0.0.0.0"};
    bool enable_mdns{true};
    std::string mdns_host_label;
    std::string mdns_service_name;
};

std::string read_text_file(const fs::path& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Unable to open file: " + path.string());
    }
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

void write_pretty_json(const fs::path& path, const json& j) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("Unable to write file: " + path.string());
    }
    out << std::setw(2) << j;
}

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d-%H%M%S");
    return oss.str();
}

std::optional<std::string> validate_config_json(const json& j, const fs::path& config_path) {
    const auto tmp_path = config_path.parent_path() / (config_path.filename().string() + ".tmp");
    try {
        write_pretty_json(tmp_path, j);
        charger::load_charger_config(tmp_path);
    } catch (const std::exception& e) {
        std::error_code ec;
        fs::remove(tmp_path, ec);
        return e.what();
    }
    std::error_code ec;
    fs::remove(tmp_path, ec);
    return std::nullopt;
}

std::int64_t to_millis_since_epoch(const fs::file_time_type& ft) {
    using namespace std::chrono;
    const auto sctp = time_point_cast<milliseconds>(ft - fs::file_time_type::clock::now() + system_clock::now());
    return sctp.time_since_epoch().count();
}

bool path_is_within(const fs::path& parent, const fs::path& child) {
    auto p_it = parent.begin();
    auto c_it = child.begin();
    for (; p_it != parent.end(); ++p_it, ++c_it) {
        if (c_it == child.end() || *p_it != *c_it) {
            return false;
        }
    }
    return true;
}

std::string detect_mime(const fs::path& p) {
    const auto ext = p.extension().string();
    if (ext == ".html") return "text/html";
    if (ext == ".js") return "text/javascript";
    if (ext == ".css") return "text/css";
    if (ext == ".json") return "application/json";
    if (ext == ".svg") return "image/svg+xml";
    if (ext == ".png") return "image/png";
    if (ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
    if (ext == ".ico") return "image/x-icon";
    if (ext == ".wasm") return "application/wasm";
    return "text/plain";
}

struct ConfigServer {
    ConfigServer(fs::path config_path, fs::path asset_dir, int port_hint, std::string bind_host)
        : config_path_(std::move(config_path)), asset_dir_(std::move(asset_dir)), port_(port_hint), bind_host_(std::move(bind_host)) {}

    ~ConfigServer() {
        stop();
    }

    bool start() {
        setup_routes();
        if (!server_.set_mount_point("/", asset_dir_.string())) {
            return false;
        }
        server_.set_default_headers({
            {"Access-Control-Allow-Origin", "*"},
            {"Cache-Control", "no-cache"}
        });
        server_.set_exception_handler([this](const httplib::Request&, httplib::Response& res, std::exception_ptr eptr) {
            std::string msg = "Unhandled server error";
            if (eptr) {
                try {
                    std::rethrow_exception(eptr);
                } catch (const std::exception& e) {
                    msg = e.what();
                }
            }
            json payload{{"ok", false}, {"error", msg}};
            set_json_response(res, payload, 500);
        });
        server_.set_error_handler([this](const httplib::Request&, httplib::Response& res) {
            json payload{{"ok", false}, {"error", "Not found"}};
            set_json_response(res, payload, 404);
        });
        server_.set_payload_max_length(5 * 1024 * 1024); // 5 MB payload ceiling

        const auto host = bind_host_.empty() ? "0.0.0.0" : bind_host_.c_str();
        if (port_ <= 0 || !server_.bind_to_port(host, port_)) {
            port_ = server_.bind_to_any_port(host);
            if (port_ <= 0) {
                return false;
            }
        }

        server_thread_ = std::thread([this]() { server_.listen_after_bind(); });
        return true;
    }

    void stop() {
        if (stopped_.exchange(true)) {
            return;
        }
        server_.stop();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    int port() const { return port_; }
    const std::string& bind_host() const { return bind_host_; }

private:
    void set_json_response(httplib::Response& res, const json& payload, int status = 200) {
        res.status = status;
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.set_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
        res.set_content(payload.dump(), "application/json");
    }

    void setup_routes() {
        server_.Options(R"(/.*)", [this](const httplib::Request&, httplib::Response& res) {
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Headers", "Content-Type");
            res.set_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
            res.status = 200;
        });

        server_.Get("/health", [this](const httplib::Request&, httplib::Response& res) {
            json payload{{"ok", true}, {"configPath", config_path_.string()}};
            set_json_response(res, payload);
        });

        server_.Get("/api/config", [this](const httplib::Request&, httplib::Response& res) {
            std::lock_guard<std::mutex> lock(mutex_);
            json payload;
            try {
                const auto text = read_text_file(config_path_);
                payload["config"] = json::parse(text);
                payload["path"] = config_path_.string();
                if (fs::exists(config_path_)) {
                    payload["modified"] = to_millis_since_epoch(fs::last_write_time(config_path_));
                    payload["size"] = fs::file_size(config_path_);
                }
                payload["ok"] = true;
                set_json_response(res, payload);
            } catch (const std::exception& e) {
                payload = {{"ok", false}, {"error", e.what()}};
                set_json_response(res, payload, 500);
            }
        });

        server_.Post("/api/config/validate", [this](const httplib::Request& req, httplib::Response& res) {
            json payload;
            try {
                auto j = json::parse(req.body);
                if (j.contains("config")) {
                    j = j["config"];
                }
                const auto validation_error = validate_config_json(j, config_path_);
                if (validation_error) {
                    payload = {{"ok", false}, {"error", *validation_error}};
                    set_json_response(res, payload, 422);
                } else {
                    payload = {{"ok", true}, {"message", "Config validated successfully"}};
                    set_json_response(res, payload);
                }
            } catch (const std::exception& e) {
                payload = {{"ok", false}, {"error", e.what()}};
                set_json_response(res, payload, 400);
            }
        });

        server_.Post("/api/config", [this](const httplib::Request& req, httplib::Response& res) {
            json payload;
            try {
                auto j = json::parse(req.body);
                if (j.contains("config")) {
                    j = j["config"];
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (fs::exists(config_path_)) {
                        const auto backup_dir = config_path_.parent_path() / "backups";
                        fs::create_directories(backup_dir);
                        const auto backup_path = backup_dir / ("charger-" + timestamp() + ".json");
                        fs::copy_file(config_path_, backup_path, fs::copy_options::overwrite_existing);
                        payload["backup"] = backup_path.string();
                    }

                    const auto validation_error = validate_config_json(j, config_path_);
                    if (validation_error) {
                        payload = {{"ok", false}, {"error", *validation_error}};
                        set_json_response(res, payload, 422);
                        return;
                    }
                    write_pretty_json(config_path_, j);
                }
                payload["ok"] = true;
                payload["message"] = "Config saved";
                set_json_response(res, payload);
            } catch (const std::exception& e) {
                payload = {{"ok", false}, {"error", e.what()}};
                set_json_response(res, payload, 400);
            }
        });

        server_.Post("/api/config/restore", [this](const httplib::Request& req, httplib::Response& res) {
            json payload;
            try {
                auto j = json::parse(req.body);
                const auto backup_dir = fs::weakly_canonical(config_path_.parent_path() / "backups");
                fs::path candidate;
                if (j.contains("path")) {
                    candidate = fs::path(j.value("path", ""));
                } else if (j.contains("name")) {
                    candidate = backup_dir / j.value("name", "");
                }
                if (candidate.empty()) {
                    throw std::runtime_error("Missing backup path");
                }
                candidate = fs::weakly_canonical(candidate);
                if (!path_is_within(backup_dir, candidate)) {
                    throw std::runtime_error("Backup path must reside under " + backup_dir.string());
                }
                if (!fs::exists(candidate) || !fs::is_regular_file(candidate)) {
                    throw std::runtime_error("Backup file not found: " + candidate.string());
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    const auto restored_json = json::parse(read_text_file(candidate));
                    if (const auto err = validate_config_json(restored_json, config_path_)) {
                        throw std::runtime_error("Restore failed validation: " + *err);
                    }
                    write_pretty_json(config_path_, restored_json);
                }
                payload = {{"ok", true}, {"restored", candidate.string()}, {"configPath", config_path_.string()}};
                set_json_response(res, payload);
            } catch (const std::exception& e) {
                payload = {{"ok", false}, {"error", e.what()}};
                set_json_response(res, payload, 400);
            }
        });

        server_.Post("/api/config/backup", [this](const httplib::Request&, httplib::Response& res) {
            json payload;
            try {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!fs::exists(config_path_)) {
                    throw std::runtime_error("Config file does not exist: " + config_path_.string());
                }
                const auto backup_dir = config_path_.parent_path() / "backups";
                fs::create_directories(backup_dir);
                const auto backup_path = backup_dir / ("manual-" + timestamp() + ".json");
                fs::copy_file(config_path_, backup_path, fs::copy_options::overwrite_existing);
                payload = {{"ok", true}, {"backup", backup_path.string()}};
                set_json_response(res, payload);
            } catch (const std::exception& e) {
                payload = {{"ok", false}, {"error", e.what()}};
                set_json_response(res, payload, 400);
            }
        });

        server_.Get("/api/config/history", [this](const httplib::Request&, httplib::Response& res) {
            json payload;
            try {
                std::vector<json> entries;
                const auto backup_dir = config_path_.parent_path() / "backups";
                if (fs::exists(backup_dir) && fs::is_directory(backup_dir)) {
                    for (const auto& entry : fs::directory_iterator(backup_dir)) {
                        if (!entry.is_regular_file()) {
                            continue;
                        }
                        json item;
                        item["path"] = entry.path().string();
                        item["name"] = entry.path().filename().string();
                        item["modified"] = to_millis_since_epoch(fs::last_write_time(entry));
                        item["size"] = entry.file_size();
                        entries.push_back(item);
                    }
                }
                std::sort(entries.begin(), entries.end(), [](const json& a, const json& b) {
                    return a.value("modified", 0LL) > b.value("modified", 0LL);
                });
                payload = {{"ok", true}, {"history", entries}};
                set_json_response(res, payload);
            } catch (const std::exception& e) {
                payload = {{"ok", false}, {"error", e.what()}};
                set_json_response(res, payload, 400);
            }
        });

        server_.Get(R"(/.*)", [this](const httplib::Request& req, httplib::Response& res) {
            try {
                if (req.path.find("..") != std::string::npos) {
                    res.status = 403;
                    res.set_content("Forbidden", "text/plain");
                    return;
                }
                fs::path path = fs::weakly_canonical(asset_dir_ / req.path.substr(1));
                if (!path_is_within(asset_dir_, path)) {
                    res.status = 403;
                    res.set_content("Forbidden", "text/plain");
                    return;
                }
                if (fs::is_directory(path)) {
                    path /= "index.html";
                }
                if (!fs::exists(path)) {
                    path = asset_dir_ / "index.html";
                }
                const auto body = read_text_file(path);
                res.set_content(body, detect_mime(path));
            } catch (const std::exception& e) {
                res.status = 404;
                res.set_content(std::string("Not found: ") + e.what(), "text/plain");
            }
        });
    }

    httplib::Server server_;
    std::thread server_thread_;
    fs::path config_path_;
    fs::path asset_dir_;
    std::mutex mutex_;
    std::atomic<bool> stopped_{false};
    int port_{8844};
    std::string bind_host_{"0.0.0.0"};
};

AppOptions parse_args(int argc, char* argv[]) {
    AppOptions opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            opts.config_path = argv[++i];
        } else if ((arg == "--assets" || arg == "-a") && i + 1 < argc) {
            opts.asset_dir = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            opts.port = std::stoi(argv[++i]);
        } else if (arg == "--headless") {
            opts.headless = true;
        } else if ((arg == "--host" || arg == "--bind") && i + 1 < argc) {
            opts.bind_host = argv[++i];
        } else if (arg == "--no-mdns") {
            opts.enable_mdns = false;
        } else if (arg == "--mdns-host" && i + 1 < argc) {
            opts.mdns_host_label = argv[++i];
        } else if (arg == "--mdns-name" && i + 1 < argc) {
            opts.mdns_service_name = argv[++i];
        }
    }
    opts.config_path = fs::exists(opts.config_path) ? fs::weakly_canonical(opts.config_path) : fs::absolute(opts.config_path);
    opts.asset_dir = fs::exists(opts.asset_dir) ? fs::weakly_canonical(opts.asset_dir) : fs::absolute(opts.asset_dir);
    return opts;
}

std::atomic<bool> keep_running{true};

void handle_signal(int) {
    keep_running = false;
}

std::string sanitize_label(const std::string& raw) {
    std::string out;
    out.reserve(raw.size());
    for (char c : raw) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '-') {
            out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        } else if (c == ' ' || c == '_' || c == '.') {
            out.push_back('-');
        }
    }
    if (out.empty()) {
        out = "charger";
    }
    if (out.size() > 63) {
        out = out.substr(0, 63);
    }
    return out;
}

#ifdef HAVE_DNSSD
#include <dns_sd.h>

class MdnsService {
public:
    ~MdnsService() { stop(); }

    bool start(const std::string& instance, const std::string& host_label, uint16_t port, const std::string& bind_ip) {
        stop();
        std::string reg_name = instance.empty() ? "charger-config" : instance;
        std::string reg_type = "_http._tcp";
        std::string domain = "local.";
        std::string host = host_label.empty() ? "" : host_label + ".local";

        auto err = DNSServiceRegister(&ref_,
                                      0,
                                      0,
                                      reg_name.c_str(),
                                      reg_type.c_str(),
                                      domain.c_str(),
                                      host.empty() ? nullptr : host.c_str(),
                                      htons(port),
                                      0,
                                      nullptr,
                                      nullptr,
                                      nullptr);
        if (err != kDNSServiceErr_NoError) {
            ref_ = nullptr;
            return false;
        }
        register_a_record(host_label, bind_ip);
        return true;
    }

    void stop() {
        if (ref_) {
            DNSServiceRefDeallocate(ref_);
            ref_ = nullptr;
        }
        if (record_ref_) {
            DNSServiceRefDeallocate(record_ref_);
            record_ref_ = nullptr;
        }
    }

private:
    void register_a_record(const std::string& host_label, const std::string& bind_ip) {
        if (host_label.empty()) {
            return;
        }
        auto ip = bind_ip;
#ifndef _WIN32
        if (ip.empty() || ip == "0.0.0.0" || ip == "::") {
            ifaddrs* ifaddr = nullptr;
            if (getifaddrs(&ifaddr) == 0) {
                for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
                    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
                    auto* sa = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
                    if (sa->sin_addr.s_addr == htonl(INADDR_LOOPBACK)) continue;
                    char buf[INET_ADDRSTRLEN];
                    if (inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf))) {
                        ip = buf;
                        break;
                    }
                }
                freeifaddrs(ifaddr);
            }
        }
#endif
        if (ip.empty() || ip == "0.0.0.0" || ip == "::") {
            ip = "127.0.0.1";
        }
        in_addr addr{};
        if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) {
            return;
        }
        std::string fqdn = host_label + ".local.";
        uint32_t ttl = 120;
        DNSServiceErrorType err = DNSServiceRegisterRecord(&record_ref_,
                                                           ref_,
                                                           kDNSServiceFlagsShared,
                                                           0,
                                                           fqdn.c_str(),
                                                           kDNSServiceType_A,
                                                           kDNSServiceClass_IN,
                                                           ttl,
                                                           &addr,
                                                           sizeof(addr),
                                                           nullptr,
                                                           nullptr);
        if (err != kDNSServiceErr_NoError) {
            if (record_ref_) {
                DNSServiceRefDeallocate(record_ref_);
                record_ref_ = nullptr;
            }
        }
    }

    DNSServiceRef ref_{nullptr};
    DNSServiceRef record_ref_{nullptr};
};
#endif

std::string extract_chargepoint_id(const fs::path& path) {
    try {
        const auto text = read_text_file(path);
        const auto j = json::parse(text);
        if (j.contains("chargePoint") && j["chargePoint"].is_object()) {
            return j["chargePoint"].value("id", "");
        }
    } catch (...) {
    }
    return {};
}

} // namespace

int main(int argc, char* argv[]) {
    auto opts = parse_args(argc, argv);
    if (!fs::exists(opts.config_path)) {
        try {
            fs::create_directories(opts.config_path.parent_path());
            std::ofstream out(opts.config_path);
            out << "{}";
            out.close();
            std::cout << "Created new config at " << opts.config_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Config file missing and could not be created: " << e.what() << std::endl;
            return 1;
        }
    }
    if (!fs::exists(opts.asset_dir)) {
        std::cerr << "Asset directory missing: " << opts.asset_dir << std::endl;
        return 1;
    }

    auto cp_id = extract_chargepoint_id(opts.config_path);
    const auto default_host_label = sanitize_label(cp_id.empty() ? "charger" : cp_id);
    if (!opts.mdns_host_label.empty()) {
        opts.mdns_host_label = sanitize_label(opts.mdns_host_label);
    }
    if (opts.mdns_host_label.empty()) {
        opts.mdns_host_label = default_host_label;
    }
    if (opts.mdns_service_name.empty()) {
        opts.mdns_service_name = cp_id.empty() ? "Charger Config" : cp_id;
    }

    ConfigServer server(opts.config_path, opts.asset_dir, opts.port, opts.bind_host);
    if (!server.start()) {
        std::cerr << "Failed to start config web server" << std::endl;
        return 1;
    }

    const std::string nav_host = (opts.bind_host == "0.0.0.0" || opts.bind_host == "::") ? "127.0.0.1" : opts.bind_host;
    std::cout << "Serving config UI from " << opts.asset_dir << " using config "
              << opts.config_path << " on http://" << nav_host << ":" << server.port() << std::endl;

#ifdef HAVE_DNSSD
    MdnsService mdns;
    if (opts.enable_mdns) {
        if (mdns.start(opts.mdns_service_name, opts.mdns_host_label, static_cast<uint16_t>(server.port()))) {
            std::cout << "mDNS announced as " << opts.mdns_service_name << " on host "
                      << opts.mdns_host_label << ".local" << std::endl;
        } else {
            std::cerr << "mDNS announcement failed; continuing without mDNS" << std::endl;
        }
    }
#endif

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (!opts.headless) {
        webview::webview view(true, nullptr);
        view.set_title("Config Guardian - Charger UI");
        view.set_size(1280, 800, WEBVIEW_HINT_NONE);
        view.navigate("http://" + nav_host + ":" + std::to_string(server.port()) + "/");
        view.run();
        keep_running = false;
    } else {
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    server.stop();
    return 0;
}
