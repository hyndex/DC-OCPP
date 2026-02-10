#include "maintenance_manager.hpp"
#include "evse_security_file.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>

using namespace charger;
namespace fs = std::filesystem;

namespace {

fs::path make_temp_dir() {
    auto base = fs::temp_directory_path() / ("dc_ocpp_maint_" + std::to_string(::getpid()));
    fs::create_directories(base);
    return base;
}

std::string pem_from_x509(X509* cert) {
    BIO* bio = BIO_new(BIO_s_mem());
    assert(bio);
    assert(PEM_write_bio_X509(bio, cert) == 1);
    char* data = nullptr;
    long len = BIO_get_mem_data(bio, &data);
    std::string out;
    if (len > 0 && data) {
        out.assign(data, static_cast<std::size_t>(len));
    }
    BIO_free(bio);
    return out;
}

EVP_PKEY* generate_rsa_key() {
    EVP_PKEY* pkey = nullptr;
    EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, nullptr);
    assert(ctx);
    assert(EVP_PKEY_keygen_init(ctx) == 1);
    assert(EVP_PKEY_CTX_set_rsa_keygen_bits(ctx, 2048) == 1);
    assert(EVP_PKEY_keygen(ctx, &pkey) == 1);
    EVP_PKEY_CTX_free(ctx);
    assert(pkey);
    return pkey;
}

X509* make_cert(EVP_PKEY* subject_key, X509* issuer, EVP_PKEY* issuer_key,
                const std::string& cn, bool is_ca, int days_valid) {
    X509* cert = X509_new();
    assert(cert);
    X509_set_version(cert, 2);
    ASN1_INTEGER_set(X509_get_serialNumber(cert), 1);
    X509_gmtime_adj(X509_get_notBefore(cert), 0);
    X509_gmtime_adj(X509_get_notAfter(cert), static_cast<long>(days_valid) * 24L * 3600L);
    assert(X509_set_pubkey(cert, subject_key) == 1);

    X509_NAME* name = X509_NAME_new();
    assert(name);
    X509_NAME_add_entry_by_txt(name, "C", MBSTRING_ASC,
                               reinterpret_cast<const unsigned char*>("IN"), -1, -1, 0);
    X509_NAME_add_entry_by_txt(name, "O", MBSTRING_ASC,
                               reinterpret_cast<const unsigned char*>("Joulepoint"), -1, -1, 0);
    X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                               reinterpret_cast<const unsigned char*>(cn.c_str()), -1, -1, 0);
    assert(X509_set_subject_name(cert, name) == 1);
    X509_NAME_free(name);

    if (issuer) {
        assert(X509_set_issuer_name(cert, X509_get_subject_name(issuer)) == 1);
    } else {
        assert(X509_set_issuer_name(cert, X509_get_subject_name(cert)) == 1);
    }

    {
        X509_EXTENSION* ext = X509V3_EXT_conf_nid(nullptr, nullptr, NID_basic_constraints,
                                                  is_ca ? "CA:TRUE" : "CA:FALSE");
        assert(ext);
        X509_add_ext(cert, ext, -1);
        X509_EXTENSION_free(ext);
    }
    {
        const char* ku = is_ca ? "keyCertSign,cRLSign" : "digitalSignature,keyEncipherment";
        X509_EXTENSION* ext = X509V3_EXT_conf_nid(nullptr, nullptr, NID_key_usage, ku);
        assert(ext);
        X509_add_ext(cert, ext, -1);
        X509_EXTENSION_free(ext);
    }

    assert(X509_sign(cert, issuer_key ? issuer_key : subject_key, EVP_sha256()) > 0);
    return cert;
}

std::string base64_encode(const std::vector<unsigned char>& data) {
    BIO* mem = BIO_new(BIO_s_mem());
    BIO* b64 = BIO_new(BIO_f_base64());
    assert(mem && b64);
    BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
    BIO_push(b64, mem);
    assert(BIO_write(b64, data.data(), static_cast<int>(data.size())) > 0);
    assert(BIO_flush(b64) == 1);
    char* out = nullptr;
    long len = BIO_get_mem_data(mem, &out);
    std::string s;
    if (len > 0 && out) {
        s.assign(out, static_cast<std::size_t>(len));
    }
    BIO_free_all(b64);
    return s;
}

std::string sign_file_sha256_b64(EVP_PKEY* signing_key, const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    assert(in);
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    assert(ctx);
    assert(EVP_DigestSignInit(ctx, nullptr, EVP_sha256(), nullptr, signing_key) == 1);
    std::array<char, 64 * 1024> buf{};
    while (in) {
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto n = in.gcount();
        if (n <= 0) break;
        assert(EVP_DigestSignUpdate(ctx, buf.data(), static_cast<std::size_t>(n)) == 1);
    }
    std::size_t sig_len = 0;
    assert(EVP_DigestSignFinal(ctx, nullptr, &sig_len) == 1);
    std::vector<unsigned char> sig(sig_len);
    assert(EVP_DigestSignFinal(ctx, sig.data(), &sig_len) == 1);
    sig.resize(sig_len);
    EVP_MD_CTX_free(ctx);
    return base64_encode(sig);
}

std::string read_file(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    assert(in);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

int main() {
    const auto tmp = make_temp_dir();
    const auto db = tmp / "db";
    const auto logs = tmp / "logs";
    const auto fw = tmp / "fw";
    const auto bin = tmp / "bin";
    fs::create_directories(db);
    fs::create_directories(logs);
    fs::create_directories(fw);
    fs::create_directories(bin);

    // Seed a log file so bundles include something deterministic.
    {
        std::ofstream out(logs / "app.log");
        out << "hello-log\n";
    }

    ChargerConfig cfg{};
    cfg.charge_point_id = "maint-test";
    cfg.database_dir = db;
    cfg.message_log_path = logs;
    cfg.require_https_uploads = true;
    cfg.upload_allow_file_targets = true;
    cfg.upload_max_bytes = 1024 * 1024;
    cfg.upload_connect_timeout_s = 2;
    cfg.upload_transfer_timeout_s = 5;
    cfg.firmware_update.enabled = true;
    cfg.firmware_update.allow_unsigned = true;
    cfg.firmware_update.staging_dir = fw;
    cfg.firmware_update.systemd_service_name = ""; // use exit fallback (stubbed in tests)
    cfg.firmware_update.target_binary_path = bin / "dc_ocpp";
    cfg.firmware_update.max_wait_seconds = 2;

    std::vector<std::string> log_status;
    std::vector<ocpp::FirmwareStatusNotification> fw_status;

    MaintenanceManager::Callbacks cb{};
    cb.on_log_status = [&](int, const std::string& st) { log_status.push_back(st); };
    cb.on_firmware_status = [&](int, ocpp::FirmwareStatusNotification st) { fw_status.push_back(st); };
    cb.any_active_transaction = []() { return false; };
    cb.restart_service = [](const std::string&) { return 0; };
    cb.exit_process = [](int) {}; // never exit during unit tests

    // Security backend for SignedUpdateFirmware verification.
    const auto ca_bundle = tmp / "mf_ca.pem";
    EVP_PKEY* ca_key = generate_rsa_key();
    X509* ca_cert = make_cert(ca_key, nullptr, nullptr, "MF Test CA", true, 365);
    {
        std::ofstream out(ca_bundle);
        out << pem_from_x509(ca_cert);
    }

    EVP_PKEY* leaf_key = generate_rsa_key();
    X509* leaf_cert = make_cert(leaf_key, ca_cert, ca_key, "MF Signing Leaf", false, 30);
    const auto signing_cert_pem = pem_from_x509(leaf_cert);

    cfg.security.mf_ca_bundle = ca_bundle;
    auto sec_ptr = std::make_shared<FileEvseSecurity>(FileEvseSecurity::Paths{.charge_point_id = cfg.charge_point_id, .security = cfg.security}, false);

    MaintenanceManager mgr(cfg, sec_ptr, cb);

    // 1) Diagnostics upload to file:// target.
    const auto diag_target = tmp / "diag_uploaded.txt";
    ocpp::v16::GetDiagnosticsRequest diag_req;
    diag_req.location = "file://" + diag_target.string();
    auto diag_resp = mgr.handle_get_diagnostics(diag_req);
    assert(diag_resp.status == ocpp::v16::LogStatusEnumType::Accepted);
    assert(mgr.wait_for_idle(std::chrono::seconds(5)));
    assert(fs::exists(diag_target));
    const auto diag_body = read_file(diag_target);
    assert(diag_body.find("dc_ocpp log bundle") != std::string::npos);
    assert(!log_status.empty());
    assert(log_status.front() == "Uploading");
    assert(log_status.back() == "Uploaded");
    log_status.clear();

    // 2) Unsigned firmware update (file://) swaps the target binary.
    {
        std::ofstream out(cfg.firmware_update.target_binary_path);
        out << "OLD";
    }
    const auto fw_src = tmp / "fw_unsigned.bin";
    {
        std::ofstream out(fw_src);
        out << "NEW";
    }

    fw_status.clear();
    ocpp::v16::UpdateFirmwareRequest uf;
    uf.location = "file://" + fw_src.string();
    uf.retrieveDate = ocpp::DateTime(); // now
    mgr.handle_update_firmware(uf);
    assert(mgr.wait_for_idle(std::chrono::seconds(5)));
    assert(read_file(cfg.firmware_update.target_binary_path) == "NEW");
    assert(std::find(fw_status.begin(), fw_status.end(), ocpp::FirmwareStatusNotification::Installed) != fw_status.end());
    assert(!fs::exists(db / "fw_update_state.json"));

    // 3) Signed firmware update verifies signature and swaps the binary.
    const auto fw_signed_src = tmp / "fw_signed.bin";
    {
        std::ofstream out(fw_signed_src);
        out << "SIGNED";
    }
    const auto sig_b64 = sign_file_sha256_b64(leaf_key, fw_signed_src);

    fw_status.clear();
    ocpp::v16::SignedUpdateFirmwareRequest suf;
    suf.requestId = 42;
    suf.firmware.location = ocpp::CiString<512>("file://" + fw_signed_src.string());
    suf.firmware.retrieveDateTime = ocpp::DateTime();
    suf.firmware.signingCertificate = ocpp::CiString<5500>(signing_cert_pem);
    suf.firmware.signature = ocpp::CiString<800>(sig_b64);
    const auto accepted = mgr.handle_signed_update_firmware(suf);
    assert(accepted == ocpp::v16::UpdateFirmwareStatusEnumType::Accepted);
    assert(mgr.wait_for_idle(std::chrono::seconds(5)));
    assert(read_file(cfg.firmware_update.target_binary_path) == "SIGNED");
    assert(std::find(fw_status.begin(), fw_status.end(), ocpp::FirmwareStatusNotification::Installed) != fw_status.end());
    assert(!fs::exists(db / "fw_update_state.json"));

    X509_free(leaf_cert);
    EVP_PKEY_free(leaf_key);
    X509_free(ca_cert);
    EVP_PKEY_free(ca_key);

    std::cout << "Maintenance manager tests passed\n";
    return 0;
}
