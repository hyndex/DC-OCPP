// SPDX-License-Identifier: Apache-2.0
#include "maintenance_manager.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <random>
#include <sstream>
#include <system_error>
#include <thread>
#include <vector>

#include <everest/logging.hpp>
#include <nlohmann/json.hpp>

#include <curl/curl.h>

#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/x509.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace charger {
namespace {

namespace fs = std::filesystem;

constexpr int kDefaultRetryIntervalSeconds = 5;

bool starts_with(const std::string& s, const char* prefix) {
    return s.rfind(prefix, 0) == 0;
}

std::string trim_copy(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::string random_suffix() {
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<unsigned long long> dist;
    std::ostringstream ss;
    ss << std::hex << dist(gen);
    return ss.str();
}

std::string timestamp_compact() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto t = system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02dT%02d%02d%02dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
}

bool is_https_url(const std::string& url) {
    return starts_with(url, "https://");
}

bool is_http_url(const std::string& url) {
    return starts_with(url, "http://");
}

bool is_ftp_url(const std::string& url) {
    return starts_with(url, "ftp://") || starts_with(url, "ftps://");
}

bool is_file_url(const std::string& url) {
    return starts_with(url, "file://");
}

std::optional<fs::path> file_url_to_path(const std::string& url) {
    if (!is_file_url(url)) {
        return std::nullopt;
    }
    std::string p = url.substr(std::strlen("file://"));
    if (p.empty()) {
        return std::nullopt;
    }
    return fs::path(p);
}

bool is_service_name_safe(const std::string& s) {
    if (s.empty()) return false;
    for (const char c : s) {
        const bool ok = std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_' || c == '@';
        if (!ok) return false;
    }
    return true;
}

template <typename TimePoint>
std::chrono::steady_clock::time_point to_steady_from_utc(const TimePoint& t_utc) {
    const auto now_utc = ocpp::DateTime().to_time_point();
    const auto now_steady = std::chrono::steady_clock::now();
    return now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(t_utc - now_utc);
}

bool ensure_dir(const fs::path& dir) {
    std::error_code ec;
    if (dir.empty()) {
        return false;
    }
    fs::create_directories(dir, ec);
    return !ec;
}

bool has_nonempty_file(const fs::path& path) {
    if (path.empty()) return false;
    std::error_code ec;
    if (!fs::exists(path, ec) || ec) return false;
    const auto size = fs::file_size(path, ec);
    if (ec) return false;
    return size > 0;
}

fs::path resolve_install_target(const fs::path& configured) {
    // If the operator configured a symlink path (common on Linux deployments),
    // prefer updating the symlink *target* so we don't replace the symlink itself.
    std::error_code ec;
    if (configured.empty()) {
        return {};
    }
    if (!fs::is_symlink(configured, ec) || ec) {
        return configured;
    }
    fs::path link = fs::read_symlink(configured, ec);
    if (ec || link.empty()) {
        return configured; // fallback: treat as a normal path
    }
    if (link.is_relative()) {
        link = configured.parent_path() / link;
    }
    fs::path resolved = fs::weakly_canonical(link, ec);
    if (ec || resolved.empty()) {
        resolved = link;
    }
    return resolved;
}

std::optional<std::string> read_file_tail(const fs::path& path, std::size_t max_bytes) {
    if (max_bytes == 0) {
        return std::string{};
    }
    std::error_code ec;
    const auto size = fs::file_size(path, ec);
    if (ec) {
        return std::nullopt;
    }
    const std::size_t take = static_cast<std::size_t>(std::min<std::uintmax_t>(size, max_bytes));
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    if (take < static_cast<std::size_t>(size)) {
        in.seekg(static_cast<std::streamoff>(size - take), std::ios::beg);
    }
    std::string out;
    out.resize(take);
    in.read(out.data(), static_cast<std::streamsize>(take));
    if (!in) {
        return std::nullopt;
    }
    return out;
}

bool write_file_atomic(const fs::path& path, const std::string& content) {
    if (path.empty()) return false;
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    if (ec) return false;

    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) return false;
        out.write(content.data(), static_cast<std::streamsize>(content.size()));
        out.flush();
        if (!out) return false;
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

void write_fw_state(const fs::path& path, const nlohmann::json& state) {
    try {
        (void)write_file_atomic(path, state.dump(2));
    } catch (...) {
        // Best-effort only.
    }
}

void clear_fw_state(const fs::path& path) {
    std::error_code ec;
    fs::remove(path, ec);
}

std::vector<fs::path> enumerate_log_files(const ChargerConfig& cfg) {
    std::vector<fs::path> out;
    std::error_code ec;
    const fs::path root = cfg.message_log_path;
    if (root.empty() || !fs::exists(root, ec) || ec) {
        return out;
    }
    const fs::path uploads_dir = (cfg.database_dir.empty() ? cfg.message_log_path : cfg.database_dir) / "uploads";

    auto excluded_root = [&](const fs::path& p) {
        std::error_code ec2;
        const auto canon = fs::weakly_canonical(p, ec2);
        const auto cert_roots = std::array<fs::path, 4>{
            cfg.security.client_cert_dir,
            cfg.security.client_key_dir,
            cfg.security.secc_cert_dir,
            cfg.security.secc_key_dir,
        };
        for (const auto& r : cert_roots) {
            if (r.empty()) continue;
            const auto rc = fs::weakly_canonical(r, ec2);
            if (!ec2 && !rc.empty() && canon.string().rfind(rc.string(), 0) == 0) {
                return true;
            }
        }
        if (!uploads_dir.empty()) {
            const auto uc = fs::weakly_canonical(uploads_dir, ec2);
            if (!ec2 && !uc.empty() && canon.string().rfind(uc.string(), 0) == 0) {
                return true;
            }
        }
        return false;
    };

    for (const auto& entry : fs::recursive_directory_iterator(root, ec)) {
        if (ec) break;
        if (!entry.is_regular_file(ec) || ec) continue;
        const auto p = entry.path();
        const auto ext = p.extension().string();
        if (ext == ".pem" || ext == ".key" || ext == ".p12" || ext == ".pfx") {
            continue;
        }
        if (excluded_root(p)) {
            continue;
        }
        out.push_back(p);
    }
    std::sort(out.begin(), out.end(), [](const fs::path& a, const fs::path& b) {
        std::error_code ea, eb;
        const auto ta = fs::last_write_time(a, ea);
        const auto tb = fs::last_write_time(b, eb);
        if (ea || eb) {
            return a.string() < b.string();
        }
        return ta < tb;
    });
    return out;
}

std::string build_log_bundle_text(const ChargerConfig& cfg, std::size_t max_bytes) {
    std::ostringstream out;
    out << "dc_ocpp log bundle\n";
    out << "generated_at=" << timestamp_compact() << "\n";
    out << "charge_point_id=" << cfg.charge_point_id << "\n\n";

    const auto files = enumerate_log_files(cfg);
    // Keep newest data within budget: iterate from newest to oldest and include tails.
    std::size_t remaining = max_bytes;
    const std::string header = out.str();
    if (header.size() >= remaining) {
        return header.substr(0, remaining);
    }
    remaining -= header.size();

    for (auto it = files.rbegin(); it != files.rend(); ++it) {
        if (remaining == 0) break;
        const auto& p = *it;
        std::error_code ec;
        const auto size = fs::file_size(p, ec);
        const std::string label = "\n==== " + p.string() + " (" + (ec ? "?" : std::to_string(size)) + " bytes) ====\n";
        if (label.size() > remaining) {
            break;
        }
        out << label;
        remaining -= label.size();

        const std::size_t take = std::min<std::size_t>(remaining, 64 * 1024); // cap per file tail
        auto chunk = read_file_tail(p, take);
        if (!chunk) {
            out << "[read failed]\n";
            continue;
        }
        if (chunk->size() > remaining) {
            chunk->resize(remaining);
        }
        out << *chunk;
        remaining -= chunk->size();
        if (remaining == 0) break;
    }

    auto s = out.str();
    if (s.size() > max_bytes) {
        s.resize(max_bytes);
    }
    return s;
}

struct CurlWriteCtx {
    FILE* fp{nullptr};
    std::size_t written{0};
    std::size_t max_bytes{0};
    bool aborted{false};
};

size_t curl_write_cb(char* ptr, size_t size, size_t nmemb, void* userdata) {
    auto* ctx = static_cast<CurlWriteCtx*>(userdata);
    const std::size_t total = size * nmemb;
    if (!ctx || !ctx->fp) return 0;
    if (ctx->max_bytes > 0 && ctx->written + total > ctx->max_bytes) {
        ctx->aborted = true;
        return 0;
    }
    const auto n = std::fwrite(ptr, 1, total, ctx->fp);
    ctx->written += n;
    return n;
}

bool curl_upload_file_put(const std::string& url, const fs::path& file_path,
                          int connect_timeout_s, int transfer_timeout_s,
                          std::string* error_out) {
    if (!has_nonempty_file(file_path)) {
        if (error_out) *error_out = "missing upload file";
        return false;
    }
    std::error_code ec;
    const auto size = fs::file_size(file_path, ec);
    if (ec) {
        if (error_out) *error_out = "stat failed";
        return false;
    }

    FILE* fp = std::fopen(file_path.string().c_str(), "rb");
    if (!fp) {
        if (error_out) *error_out = std::string("fopen failed: ") + std::strerror(errno);
        return false;
    }

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::fclose(fp);
        if (error_out) *error_out = "curl_easy_init failed";
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_READDATA, fp);
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, static_cast<curl_off_t>(size));
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, transfer_timeout_s);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "dc_ocpp/1.0");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    // HTTPS verification is enabled by default; keep defaults.
    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_easy_cleanup(curl);
    std::fclose(fp);

    if (res != CURLE_OK) {
        if (error_out) *error_out = curl_easy_strerror(res);
        return false;
    }
    if (is_http_url(url) || is_https_url(url)) {
        if (http_code < 200 || http_code >= 300) {
            if (error_out) *error_out = "HTTP status " + std::to_string(http_code);
            return false;
        }
    }
    return true;
}

bool curl_download_to_file(const std::string& url, const fs::path& out_path,
                           int connect_timeout_s, int transfer_timeout_s,
                           std::size_t max_bytes,
                           std::string* error_out) {
    std::error_code ec;
    fs::create_directories(out_path.parent_path(), ec);
    if (ec) {
        if (error_out) *error_out = "mkdir failed";
        return false;
    }

    FILE* fp = std::fopen(out_path.string().c_str(), "wb");
    if (!fp) {
        if (error_out) *error_out = std::string("fopen failed: ") + std::strerror(errno);
        return false;
    }

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::fclose(fp);
        if (error_out) *error_out = "curl_easy_init failed";
        return false;
    }

    CurlWriteCtx wctx;
    wctx.fp = fp;
    wctx.max_bytes = max_bytes;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &wctx);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, transfer_timeout_s);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "dc_ocpp/1.0");

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_easy_cleanup(curl);
    std::fclose(fp);

    if (wctx.aborted) {
        fs::remove(out_path, ec);
        if (error_out) *error_out = "download exceeded size cap";
        return false;
    }
    if (res != CURLE_OK) {
        fs::remove(out_path, ec);
        if (error_out) *error_out = curl_easy_strerror(res);
        return false;
    }
    if (is_http_url(url) || is_https_url(url)) {
        if (http_code < 200 || http_code >= 300) {
            fs::remove(out_path, ec);
            if (error_out) *error_out = "HTTP status " + std::to_string(http_code);
            return false;
        }
    }
    return true;
}

bool copy_file_fsync(const fs::path& src, const fs::path& dst, mode_t mode, std::string* error_out) {
    int in_fd = ::open(src.string().c_str(), O_RDONLY);
    if (in_fd < 0) {
        if (error_out) *error_out = std::string("open src failed: ") + std::strerror(errno);
        return false;
    }
    int out_fd = ::open(dst.string().c_str(), O_CREAT | O_TRUNC | O_WRONLY, mode);
    if (out_fd < 0) {
        ::close(in_fd);
        if (error_out) *error_out = std::string("open dst failed: ") + std::strerror(errno);
        return false;
    }
    std::array<char, 64 * 1024> buf{};
    while (true) {
        const ssize_t n = ::read(in_fd, buf.data(), buf.size());
        if (n == 0) break;
        if (n < 0) {
            if (errno == EINTR) continue;
            ::close(in_fd);
            ::close(out_fd);
            if (error_out) *error_out = std::string("read failed: ") + std::strerror(errno);
            return false;
        }
        ssize_t off = 0;
        while (off < n) {
            const ssize_t w = ::write(out_fd, buf.data() + off, static_cast<std::size_t>(n - off));
            if (w < 0) {
                if (errno == EINTR) continue;
                ::close(in_fd);
                ::close(out_fd);
                if (error_out) *error_out = std::string("write failed: ") + std::strerror(errno);
                return false;
            }
            off += w;
        }
    }
    if (::fsync(out_fd) != 0) {
        ::close(in_fd);
        ::close(out_fd);
        if (error_out) *error_out = std::string("fsync failed: ") + std::strerror(errno);
        return false;
    }
    ::close(in_fd);
    ::close(out_fd);
    return true;
}

void fsync_dir_best_effort(const fs::path& dir) {
    if (dir.empty()) return;
    const int fd = ::open(dir.string().c_str(), O_RDONLY);
    if (fd < 0) return;
    (void)::fsync(fd);
    ::close(fd);
}

bool rename_overwrite_posix(const fs::path& src, const fs::path& dst, std::string* error_out) {
    if (src.empty() || dst.empty()) {
        if (error_out) *error_out = "rename: empty path";
        return false;
    }
    if (::rename(src.string().c_str(), dst.string().c_str()) != 0) {
        if (error_out) *error_out = std::string("rename failed: ") + std::strerror(errno);
        return false;
    }
    fsync_dir_best_effort(dst.parent_path());
    return true;
}

std::optional<std::vector<unsigned char>> base64_decode(const std::string& input) {
    std::string in = input;
    in.erase(std::remove_if(in.begin(), in.end(),
                            [](unsigned char c) { return std::isspace(c); }),
             in.end());
    if (in.empty()) {
        return std::vector<unsigned char>{};
    }
    BIO* bio = BIO_new_mem_buf(in.data(), static_cast<int>(in.size()));
    BIO* b64 = BIO_new(BIO_f_base64());
    if (!bio || !b64) {
        if (bio) BIO_free(bio);
        if (b64) BIO_free(b64);
        return std::nullopt;
    }
    BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
    bio = BIO_push(b64, bio);
    std::vector<unsigned char> out((in.size() * 3) / 4 + 4);
    const int n = BIO_read(bio, out.data(), static_cast<int>(out.size()));
    BIO_free_all(bio);
    if (n < 0) {
        return std::nullopt;
    }
    out.resize(static_cast<std::size_t>(n));
    return out;
}

std::unique_ptr<X509, decltype(&X509_free)> parse_pem_cert(const std::string& pem) {
    BIO* bio = BIO_new_mem_buf(pem.data(), static_cast<int>(pem.size()));
    if (!bio) {
        return {nullptr, X509_free};
    }
    X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    return {cert, X509_free};
}

bool verify_signature_over_file(const std::string& signing_cert_pem,
                                const std::string& signature_b64,
                                const fs::path& artifact_path,
                                std::string* error_out) {
    if (!has_nonempty_file(artifact_path)) {
        if (error_out) *error_out = "artifact missing";
        return false;
    }
    auto sig = base64_decode(signature_b64);
    if (!sig) {
        if (error_out) *error_out = "signature base64 decode failed";
        return false;
    }
    auto cert = parse_pem_cert(signing_cert_pem);
    if (!cert) {
        if (error_out) *error_out = "signingCertificate PEM parse failed";
        return false;
    }
    EVP_PKEY* pkey = X509_get_pubkey(cert.get());
    if (!pkey) {
        if (error_out) *error_out = "signingCertificate has no public key";
        return false;
    }

    std::array<const EVP_MD*, 3> mds = {EVP_sha256(), EVP_sha384(), EVP_sha512()};
    bool ok = false;
    std::string last_err;
    for (const EVP_MD* md : mds) {
        EVP_MD_CTX* ctx = EVP_MD_CTX_new();
        if (!ctx) {
            last_err = "EVP_MD_CTX_new failed";
            continue;
        }
        if (EVP_DigestVerifyInit(ctx, nullptr, md, nullptr, pkey) != 1) {
            EVP_MD_CTX_free(ctx);
            last_err = "EVP_DigestVerifyInit failed";
            continue;
        }

        std::ifstream in(artifact_path, std::ios::binary);
        if (!in) {
            EVP_MD_CTX_free(ctx);
            last_err = "open artifact failed";
            continue;
        }
        std::array<char, 64 * 1024> buf{};
        while (in) {
            in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
            const std::streamsize n = in.gcount();
            if (n <= 0) break;
            if (EVP_DigestVerifyUpdate(ctx, buf.data(), static_cast<std::size_t>(n)) != 1) {
                last_err = "EVP_DigestVerifyUpdate failed";
                break;
            }
        }
        if (last_err.empty()) {
            const int rc = EVP_DigestVerifyFinal(ctx, sig->data(), sig->size());
            if (rc == 1) {
                ok = true;
            } else {
                last_err = "EVP_DigestVerifyFinal failed";
            }
        }
        EVP_MD_CTX_free(ctx);
        if (ok) break;
    }
    EVP_PKEY_free(pkey);

    if (!ok && error_out) {
        *error_out = last_err;
    }
    return ok;
}

int default_restart_service(const std::string& service) {
    if (!is_service_name_safe(service)) {
        return 2;
    }
    const fs::path systemctl1("/bin/systemctl");
    const fs::path systemctl2("/usr/bin/systemctl");
    fs::path systemctl;
    if (fs::exists(systemctl1)) {
        systemctl = systemctl1;
    } else if (fs::exists(systemctl2)) {
        systemctl = systemctl2;
    } else {
        return 127;
    }

    const char* argv[] = {systemctl.c_str(), "restart", "--no-block", service.c_str(), nullptr};
    pid_t pid = ::fork();
    if (pid < 0) {
        return 1;
    }
    if (pid == 0) {
        ::execv(systemctl.c_str(), const_cast<char* const*>(argv));
        ::_exit(127);
    }
    int status = 0;
    if (::waitpid(pid, &status, 0) < 0) {
        return 1;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return 1;
}

void default_exit_process(int code) {
    std::exit(code);
}

} // namespace

MaintenanceManager::MaintenanceManager(ChargerConfig cfg,
                                       std::shared_ptr<ocpp::EvseSecurity> evse_security,
                                       Callbacks callbacks) :
    cfg_(std::move(cfg)),
    evse_security_(std::move(evse_security)),
    callbacks_(std::move(callbacks)) {
    static std::once_flag curl_once;
    std::call_once(curl_once, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });

    const fs::path base = cfg_.database_dir.empty() ? cfg_.message_log_path : cfg_.database_dir;
    ensure_dir(base);
    uploads_dir_ = base / "uploads";
    fw_state_path_ = base / "fw_update_state.json";
    ensure_dir(uploads_dir_);

    if (has_nonempty_file(fw_state_path_)) {
        EVLOG_warning << "Found existing firmware update state file at " << fw_state_path_
                      << " (previous update may have been interrupted)";
    }

    if (!callbacks_.restart_service) {
        callbacks_.restart_service = default_restart_service;
    }
    if (!callbacks_.exit_process) {
        callbacks_.exit_process = default_exit_process;
    }

    stopping_.store(false);
    worker_ = std::thread([this]() { worker_loop(); });
}

MaintenanceManager::~MaintenanceManager() {
    shutdown();
}

void MaintenanceManager::shutdown() {
    if (stopping_.exchange(true)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(job_mutex_);
        while (!jobs_.empty()) jobs_.pop();
        active_jobs_ = 0;
    }
    job_cv_.notify_all();
    idle_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool MaintenanceManager::wait_for_idle(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(job_mutex_);
    return idle_cv_.wait_for(lock, timeout, [this]() { return jobs_.empty() && active_jobs_ == 0; });
}

void MaintenanceManager::enqueue_job(std::chrono::steady_clock::time_point run_at, std::function<void()> fn) {
    if (stopping_.load()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(job_mutex_);
        jobs_.push(Job{run_at, std::move(fn)});
    }
    job_cv_.notify_one();
}

void MaintenanceManager::worker_loop() {
    while (!stopping_.load()) {
        std::function<void()> fn;
        {
            std::unique_lock<std::mutex> lock(job_mutex_);
            if (jobs_.empty()) {
                job_cv_.wait(lock, [this]() { return stopping_.load() || !jobs_.empty(); });
            }
            if (stopping_.load()) {
                break;
            }
            if (jobs_.empty()) {
                continue;
            }
            const auto now = std::chrono::steady_clock::now();
            const auto next_at = jobs_.top().run_at;
            if (next_at > now) {
                job_cv_.wait_until(lock, next_at, [this, next_at]() {
                    return stopping_.load() || jobs_.empty() || jobs_.top().run_at < next_at;
                });
                continue;
            }
            fn = std::move(jobs_.top().fn);
            jobs_.pop();
            active_jobs_++;
        }
        if (!fn) {
            std::lock_guard<std::mutex> lock(job_mutex_);
            active_jobs_ = std::max(0, active_jobs_ - 1);
            if (jobs_.empty() && active_jobs_ == 0) {
                idle_cv_.notify_all();
            }
            continue;
        }
        try {
            fn();
        } catch (const std::exception& e) {
            EVLOG_error << "Maintenance job threw exception: " << e.what();
        } catch (...) {
            EVLOG_error << "Maintenance job threw unknown exception";
        }

        {
            std::lock_guard<std::mutex> lock(job_mutex_);
            active_jobs_ = std::max(0, active_jobs_ - 1);
            if (jobs_.empty() && active_jobs_ == 0) {
                idle_cv_.notify_all();
            }
        }
    }
}

ocpp::v16::GetLogResponse MaintenanceManager::handle_get_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) {
    ocpp::v16::GetLogResponse resp{};

    const std::string url = request.location;
    const bool allow_file = cfg_.upload_allow_file_targets;
    if (cfg_.require_https_uploads) {
        if (!(is_https_url(url) || (allow_file && is_file_url(url)))) {
            resp.status = ocpp::v16::LogStatusEnumType::Rejected;
            return resp;
        }
    }

    ensure_dir(uploads_dir_);
    const std::string filename = "diagnostics_" + timestamp_compact() + "_" + random_suffix() + ".txt";
    const fs::path bundle_path = uploads_dir_ / filename;

    resp.status = ocpp::v16::LogStatusEnumType::Accepted;
    resp.filename.emplace(ocpp::CiString<255>(filename));

    if (callbacks_.on_log_status) {
        callbacks_.on_log_status(-1, "Uploading");
    }

    const int retries = request.retries.value_or(0);
    const int interval = request.retryInterval.value_or(kDefaultRetryIntervalSeconds);

    enqueue_job(std::chrono::steady_clock::now(), [this, url, bundle_path, retries, interval]() {
        bool ok = false;
        std::string err;
        try {
            const auto txt = build_log_bundle_text(cfg_, cfg_.upload_max_bytes);
            ok = write_file_atomic(bundle_path, txt);
            if (!ok) {
                err = "bundle write failed";
            } else if (is_file_url(url)) {
                if (!cfg_.upload_allow_file_targets) {
                    ok = false;
                    err = "file:// targets disabled";
                } else {
                    auto target = file_url_to_path(url);
                    if (!target) {
                        ok = false;
                        err = "invalid file:// URL";
                    } else {
                        std::error_code ec;
                        fs::create_directories(target->parent_path(), ec);
                        ok = !ec && fs::copy_file(bundle_path, *target, fs::copy_options::overwrite_existing, ec);
                        if (!ok) {
                            err = "file copy failed";
                        }
                    }
                }
            } else {
                for (int attempt = 0; attempt <= retries; ++attempt) {
                    if (curl_upload_file_put(url, bundle_path, cfg_.upload_connect_timeout_s,
                                             cfg_.upload_transfer_timeout_s, &err)) {
                        ok = true;
                        break;
                    }
                    if (attempt < retries) {
                        std::this_thread::sleep_for(std::chrono::seconds(std::max(0, interval)));
                    }
                }
            }
        } catch (const std::exception& e) {
            ok = false;
            err = e.what();
        }

        if (!ok) {
            EVLOG_error << "Diagnostics upload failed: " << err;
        }
        if (callbacks_.on_log_status) {
            callbacks_.on_log_status(-1, ok ? "Uploaded" : "UploadFailed");
        }
    });

    return resp;
}

ocpp::v16::GetLogResponse MaintenanceManager::handle_get_log(const ocpp::v16::GetLogRequest& request) {
    ocpp::v16::GetLogResponse resp{};

    const std::string url = request.log.remoteLocation.get();
    const bool allow_file = cfg_.upload_allow_file_targets;
    if (cfg_.require_https_uploads) {
        if (!(is_https_url(url) || (allow_file && is_file_url(url)))) {
            resp.status = ocpp::v16::LogStatusEnumType::Rejected;
            return resp;
        }
    }

    ensure_dir(uploads_dir_);
    const std::string filename = "logs_" + std::to_string(request.requestId) + "_" + timestamp_compact() + "_" + random_suffix() + ".txt";
    const fs::path bundle_path = uploads_dir_ / filename;

    resp.status = ocpp::v16::LogStatusEnumType::Accepted;
    resp.filename.emplace(ocpp::CiString<255>(filename));

    if (callbacks_.on_log_status) {
        callbacks_.on_log_status(request.requestId, "Uploading");
    }

    enqueue_job(std::chrono::steady_clock::now(), [this, url, bundle_path, request]() {
        bool ok = false;
        std::string err;
        try {
            const auto txt = build_log_bundle_text(cfg_, cfg_.upload_max_bytes);
            ok = write_file_atomic(bundle_path, txt);
            if (!ok) {
                err = "bundle write failed";
            } else if (is_file_url(url)) {
                if (!cfg_.upload_allow_file_targets) {
                    ok = false;
                    err = "file:// targets disabled";
                } else {
                    auto target = file_url_to_path(url);
                    if (!target) {
                        ok = false;
                        err = "invalid file:// URL";
                    } else {
                        std::error_code ec;
                        fs::create_directories(target->parent_path(), ec);
                        ok = !ec && fs::copy_file(bundle_path, *target, fs::copy_options::overwrite_existing, ec);
                        if (!ok) {
                            err = "file copy failed";
                        }
                    }
                }
            } else {
                const int retries = request.retries.value_or(0);
                const int interval = request.retryInterval.value_or(kDefaultRetryIntervalSeconds);
                for (int attempt = 0; attempt <= retries; ++attempt) {
                    if (curl_upload_file_put(url, bundle_path, cfg_.upload_connect_timeout_s,
                                             cfg_.upload_transfer_timeout_s, &err)) {
                        ok = true;
                        break;
                    }
                    if (attempt < retries) {
                        std::this_thread::sleep_for(std::chrono::seconds(std::max(0, interval)));
                    }
                }
            }
        } catch (const std::exception& e) {
            ok = false;
            err = e.what();
        }

        if (!ok) {
            EVLOG_error << "Log upload failed: " << err;
        }
        if (callbacks_.on_log_status) {
            callbacks_.on_log_status(request.requestId, ok ? "Uploaded" : "UploadFailed");
        }
    });

    return resp;
}

void MaintenanceManager::handle_update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) {
    if (!cfg_.firmware_update.enabled) {
        EVLOG_warning << "UpdateFirmware rejected (firmwareUpdate.enabled=false)";
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
        }
        return;
    }
    if (!cfg_.firmware_update.allow_unsigned) {
        EVLOG_warning << "UpdateFirmware rejected (firmwareUpdate.allowUnsigned=false)";
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(fw_mutex_);
        if (fw_job_active_) {
            EVLOG_warning << "Firmware update already in progress; rejecting new UpdateFirmware request";
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            return;
        }
        fw_job_active_ = true;
    }

    const auto run_at = to_steady_from_utc(request.retrieveDate.to_time_point());
    const std::string url = request.location;

    {
        nlohmann::json st;
        st["updatedAt"] = timestamp_compact();
        st["stage"] = "Scheduled";
        st["signed"] = false;
        st["location"] = url;
        st["retrieveDateTime"] = request.retrieveDate.to_rfc3339();
        st["stagingDir"] = cfg_.firmware_update.staging_dir.string();
        st["targetBinaryPath"] = cfg_.firmware_update.target_binary_path.string();
        st["systemdServiceName"] = cfg_.firmware_update.systemd_service_name;
        write_fw_state(fw_state_path_, st);
    }

    enqueue_job(run_at, [this, url, request]() {
        auto release_fw_job = [&]() {
            std::lock_guard<std::mutex> lock(fw_mutex_);
            fw_job_active_ = false;
        };

        write_fw_state(fw_state_path_, {
                                           {"updatedAt", timestamp_compact()},
                                           {"stage", "Downloading"},
                                           {"signed", false},
                                           {"location", url},
                                           {"retrieveDateTime", request.retrieveDate.to_rfc3339()},
                                       });
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Downloading);
        }

        const fs::path staging_dir = cfg_.firmware_update.staging_dir;
        ensure_dir(staging_dir);
        const fs::path download_path = staging_dir / ("fw_unsigned_" + timestamp_compact() + "_" + random_suffix() + ".bin");

        bool ok = false;
        std::string err;

        if (cfg_.require_https_uploads) {
            const bool allow_file = cfg_.upload_allow_file_targets;
            if (!(is_https_url(url) || (allow_file && is_file_url(url)))) {
                err = "URL not allowed (requireHttpsUploads=true)";
                ok = false;
            }
        }

        if (err.empty()) {
            if (is_file_url(url)) {
                auto src = file_url_to_path(url);
                if (!src || !has_nonempty_file(*src)) {
                    ok = false;
                    err = "file:// source missing";
                } else if (!cfg_.upload_allow_file_targets) {
                    ok = false;
                    err = "file:// targets disabled";
                } else {
                    ok = fs::copy_file(*src, download_path, fs::copy_options::overwrite_existing);
                }
            } else {
                ok = curl_download_to_file(url, download_path, cfg_.upload_connect_timeout_s,
                                           cfg_.upload_transfer_timeout_s, cfg_.upload_max_bytes, &err);
            }
        }

        if (!ok) {
            EVLOG_error << "Firmware download failed: " << err;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "DownloadFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", err}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::DownloadFailed);
            }
            release_fw_job();
            return;
        }

        write_fw_state(fw_state_path_, {
                                           {"updatedAt", timestamp_compact()},
                                           {"stage", "Downloaded"},
                                           {"signed", false},
                                           {"location", url},
                                           {"stagingPath", download_path.string()},
                                       });
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Downloaded);
        }

        // Wait for active transactions to finish.
        const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(cfg_.firmware_update.max_wait_seconds);
        while (callbacks_.any_active_transaction && callbacks_.any_active_transaction() &&
               std::chrono::steady_clock::now() < wait_deadline) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (callbacks_.any_active_transaction && callbacks_.any_active_transaction()) {
            EVLOG_error << "Firmware install blocked by active transaction(s) beyond maxWaitSeconds="
                        << cfg_.firmware_update.max_wait_seconds;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", "active transaction(s)"}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        write_fw_state(fw_state_path_, {
                                           {"updatedAt", timestamp_compact()},
                                           {"stage", "Installing"},
                                           {"signed", false},
                                           {"location", url},
                                           {"stagingPath", download_path.string()},
                                       });
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Installing);
        }

        fs::path target = cfg_.firmware_update.target_binary_path;
        if (target.empty()) {
            // Linux-only default path discovery.
            std::error_code ec;
            const fs::path self("/proc/self/exe");
            if (fs::exists(self, ec) && !ec) {
                target = fs::read_symlink(self, ec);
            }
        }
        target = resolve_install_target(target);
        if (target.empty()) {
            EVLOG_error << "Firmware install failed: targetBinaryPath not configured and /proc/self/exe unavailable";
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", "targetBinaryPath unavailable"}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        std::error_code ec;
        fs::create_directories(target.parent_path(), ec);
        if (ec) {
            EVLOG_error << "Firmware install failed: mkdir " << target.parent_path() << " error=" << ec.message();
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", ec.message()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        const fs::path new_path = target.string() + ".new";
        const fs::path bak_path = target.string() + ".bak";

        if (!copy_file_fsync(download_path, new_path, 0755, &err)) {
            EVLOG_error << "Firmware install failed: " << err;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", err}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        // Create/refresh a backup before replacing the on-disk binary. This avoids leaving
        // the system without a runnable binary if power is lost mid-update.
        if (fs::exists(target, ec) && !ec) {
            std::string bak_err;
            if (!copy_file_fsync(target, bak_path, 0755, &bak_err)) {
                EVLOG_error << "Firmware install failed: backup creation failed: " << bak_err;
                write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                                {"stage", "InstallationFailed"},
                                                {"signed", false},
                                                {"location", url},
                                                {"error", "backup failed: " + bak_err}});
                if (callbacks_.on_firmware_status) {
                    callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
                }
                release_fw_job();
                return;
            }
        }

        if (!rename_overwrite_posix(new_path, target, &err)) {
            EVLOG_error << "Firmware install failed: " << err;
            fs::remove(new_path, ec);
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", err}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        // Trigger restart.
        int restart_rc = 0;
        const std::string svc = cfg_.firmware_update.systemd_service_name;
        if (!svc.empty()) {
            restart_rc = callbacks_.restart_service ? callbacks_.restart_service(svc) : default_restart_service(svc);
            if (restart_rc != 0) {
                EVLOG_error << "Firmware restart failed: systemctl rc=" << restart_rc << " service=" << svc;
            }
        }
        if (restart_rc != 0) {
            // Roll back using the previously created backup, if present.
            if (fs::exists(bak_path, ec) && !ec) {
                const fs::path rollback_path = target.string() + ".rollback";
                std::string rb_err;
                if (copy_file_fsync(bak_path, rollback_path, 0755, &rb_err)) {
                    (void)rename_overwrite_posix(rollback_path, target, &rb_err);
                    fs::remove(rollback_path, ec);
                } else {
                    EVLOG_error << "Firmware rollback failed: " << rb_err;
                }
            }
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", false},
                                            {"location", url},
                                            {"error", "restart failed"}});
            release_fw_job();
            return;
        }

        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Installed);
        }
        clear_fw_state(fw_state_path_);
        release_fw_job();

        // If we swapped the binary on disk, this process is still running the old image.
        // Exit cleanly so the supervisor/systemd can restart on the new binary.
        if (callbacks_.exit_process) {
            callbacks_.exit_process(0);
        }
    });
}

ocpp::v16::UpdateFirmwareStatusEnumType
MaintenanceManager::handle_signed_update_firmware(const ocpp::v16::SignedUpdateFirmwareRequest& request) {
    if (!cfg_.firmware_update.enabled) {
        EVLOG_warning << "SignedUpdateFirmware rejected (firmwareUpdate.enabled=false)";
        return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
    }

    {
        std::lock_guard<std::mutex> lock(fw_mutex_);
        if (fw_job_active_) {
            EVLOG_warning << "Firmware update already in progress; rejecting new SignedUpdateFirmware request";
            return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
        }
        fw_job_active_ = true;
    }

    const auto& fw = request.firmware;
    const std::string url = fw.location.get();
    const std::string signing_cert = fw.signingCertificate.get();
    const std::string signature = fw.signature.get();
    const auto retrieve_at = to_steady_from_utc(fw.retrieveDateTime.to_time_point());
    const auto install_at = fw.installDateTime ? std::optional<std::chrono::steady_clock::time_point>(
                                                    to_steady_from_utc(fw.installDateTime->to_time_point()))
                                               : std::optional<std::chrono::steady_clock::time_point>{};

    // Certificate validation (strict).
    if (evse_security_) {
        const auto vr = evse_security_->verify_certificate(signing_cert, ocpp::LeafCertificateType::MF);
        if (vr != ocpp::CertificateValidationResult::Valid) {
            EVLOG_error << "SignedUpdateFirmware rejected: signingCertificate failed MF verification ("
                        << static_cast<int>(vr) << ")";
            std::lock_guard<std::mutex> lock(fw_mutex_);
            fw_job_active_ = false;
            return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
        }
    }

    {
        nlohmann::json st;
        st["updatedAt"] = timestamp_compact();
        st["stage"] = "Scheduled";
        st["signed"] = true;
        st["requestId"] = request.requestId;
        st["location"] = url;
        st["retrieveDateTime"] = fw.retrieveDateTime.to_rfc3339();
        if (fw.installDateTime) {
            st["installDateTime"] = fw.installDateTime->to_rfc3339();
        }
        st["stagingDir"] = cfg_.firmware_update.staging_dir.string();
        st["targetBinaryPath"] = cfg_.firmware_update.target_binary_path.string();
        st["systemdServiceName"] = cfg_.firmware_update.systemd_service_name;
        write_fw_state(fw_state_path_, st);
    }

    enqueue_job(retrieve_at, [this, url, signing_cert, signature, install_at, request]() {
        auto release_fw_job = [&]() {
            std::lock_guard<std::mutex> lock(fw_mutex_);
            fw_job_active_ = false;
        };

        write_fw_state(fw_state_path_, {
                                           {"updatedAt", timestamp_compact()},
                                           {"stage", "Downloading"},
                                           {"signed", true},
                                           {"requestId", request.requestId},
                                           {"location", url},
                                       });
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Downloading);
        }

        const fs::path staging_dir = cfg_.firmware_update.staging_dir;
        ensure_dir(staging_dir);
        const fs::path download_path = staging_dir / ("fw_signed_" + std::to_string(request.requestId) + "_" +
                                                     timestamp_compact() + "_" + random_suffix() + ".bin");

        bool ok = false;
        std::string err;

        if (cfg_.require_https_uploads) {
            const bool allow_file = cfg_.upload_allow_file_targets;
            if (!(is_https_url(url) || (allow_file && is_file_url(url)))) {
                err = "URL not allowed (requireHttpsUploads=true)";
                ok = false;
            }
        }

        if (err.empty()) {
            if (is_file_url(url)) {
                auto src = file_url_to_path(url);
                if (!src || !has_nonempty_file(*src)) {
                    ok = false;
                    err = "file:// source missing";
                } else if (!cfg_.upload_allow_file_targets) {
                    ok = false;
                    err = "file:// targets disabled";
                } else {
                    ok = fs::copy_file(*src, download_path, fs::copy_options::overwrite_existing);
                }
            } else {
                ok = curl_download_to_file(url, download_path, cfg_.upload_connect_timeout_s,
                                           cfg_.upload_transfer_timeout_s, cfg_.upload_max_bytes, &err);
            }
        }

        if (!ok) {
            EVLOG_error << "Signed firmware download failed: " << err;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "DownloadFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", err}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::DownloadFailed);
            }
            release_fw_job();
            return;
        }

        if (!verify_signature_over_file(signing_cert, signature, download_path, &err)) {
            EVLOG_error << "Signed firmware signature verification failed: " << err;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", "signature verification failed: " + err},
                                            {"stagingPath", download_path.string()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        write_fw_state(fw_state_path_, {
                                           {"updatedAt", timestamp_compact()},
                                           {"stage", "Downloaded"},
                                           {"signed", true},
                                           {"requestId", request.requestId},
                                           {"location", url},
                                           {"stagingPath", download_path.string()},
                                       });
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Downloaded);
        }

        if (install_at) {
            const auto now = std::chrono::steady_clock::now();
            if (*install_at > now) {
                std::this_thread::sleep_for(*install_at - now);
            }
        }

        const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(cfg_.firmware_update.max_wait_seconds);
        while (callbacks_.any_active_transaction && callbacks_.any_active_transaction() &&
               std::chrono::steady_clock::now() < wait_deadline) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (callbacks_.any_active_transaction && callbacks_.any_active_transaction()) {
            EVLOG_error << "Signed firmware install blocked by active transaction(s) beyond maxWaitSeconds="
                        << cfg_.firmware_update.max_wait_seconds;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", "active transaction(s)"},
                                            {"stagingPath", download_path.string()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        write_fw_state(fw_state_path_, {
                                           {"updatedAt", timestamp_compact()},
                                           {"stage", "Installing"},
                                           {"signed", true},
                                           {"requestId", request.requestId},
                                           {"location", url},
                                           {"stagingPath", download_path.string()},
                                       });
        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Installing);
        }

        fs::path target = cfg_.firmware_update.target_binary_path;
        if (target.empty()) {
            std::error_code ec;
            const fs::path self("/proc/self/exe");
            if (fs::exists(self, ec) && !ec) {
                target = fs::read_symlink(self, ec);
            }
        }
        target = resolve_install_target(target);
        if (target.empty()) {
            EVLOG_error << "Signed firmware install failed: targetBinaryPath not configured and /proc/self/exe unavailable";
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", "targetBinaryPath unavailable"},
                                            {"stagingPath", download_path.string()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        std::error_code ec;
        fs::create_directories(target.parent_path(), ec);
        if (ec) {
            EVLOG_error << "Signed firmware install failed: mkdir " << target.parent_path() << " error=" << ec.message();
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", ec.message()},
                                            {"stagingPath", download_path.string()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        const fs::path new_path = target.string() + ".new";
        const fs::path bak_path = target.string() + ".bak";

        if (!copy_file_fsync(download_path, new_path, 0755, &err)) {
            EVLOG_error << "Signed firmware install failed: " << err;
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", err},
                                            {"stagingPath", download_path.string()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        if (fs::exists(target, ec) && !ec) {
            std::string bak_err;
            if (!copy_file_fsync(target, bak_path, 0755, &bak_err)) {
                EVLOG_error << "Signed firmware install failed: backup creation failed: " << bak_err;
                write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                                {"stage", "InstallationFailed"},
                                                {"signed", true},
                                                {"requestId", request.requestId},
                                                {"location", url},
                                                {"error", "backup failed: " + bak_err},
                                                {"stagingPath", download_path.string()}});
                if (callbacks_.on_firmware_status) {
                    callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
                }
                release_fw_job();
                return;
            }
        }

        if (!rename_overwrite_posix(new_path, target, &err)) {
            EVLOG_error << "Signed firmware install failed: " << err;
            fs::remove(new_path, ec);
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", err},
                                            {"stagingPath", download_path.string()}});
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            release_fw_job();
            return;
        }

        int restart_rc = 0;
        const std::string svc = cfg_.firmware_update.systemd_service_name;
        if (!svc.empty()) {
            restart_rc = callbacks_.restart_service ? callbacks_.restart_service(svc) : default_restart_service(svc);
            if (restart_rc != 0) {
                EVLOG_error << "Signed firmware restart failed: systemctl rc=" << restart_rc << " service=" << svc;
            }
        }
        if (restart_rc != 0) {
            // Roll back using the previously created backup, if present.
            if (fs::exists(bak_path, ec) && !ec) {
                const fs::path rollback_path = target.string() + ".rollback";
                std::string rb_err;
                if (copy_file_fsync(bak_path, rollback_path, 0755, &rb_err)) {
                    (void)rename_overwrite_posix(rollback_path, target, &rb_err);
                    fs::remove(rollback_path, ec);
                } else {
                    EVLOG_error << "Signed firmware rollback failed: " << rb_err;
                }
            }
            if (callbacks_.on_firmware_status) {
                callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::InstallationFailed);
            }
            write_fw_state(fw_state_path_, {{"updatedAt", timestamp_compact()},
                                            {"stage", "InstallationFailed"},
                                            {"signed", true},
                                            {"requestId", request.requestId},
                                            {"location", url},
                                            {"error", "restart failed"},
                                            {"stagingPath", download_path.string()}});
            release_fw_job();
            return;
        }

        if (callbacks_.on_firmware_status) {
            callbacks_.on_firmware_status(-1, ocpp::FirmwareStatusNotification::Installed);
        }
        clear_fw_state(fw_state_path_);
        release_fw_job();

        if (callbacks_.exit_process) {
            callbacks_.exit_process(0);
        }
    });

    return ocpp::v16::UpdateFirmwareStatusEnumType::Accepted;
}

} // namespace charger
