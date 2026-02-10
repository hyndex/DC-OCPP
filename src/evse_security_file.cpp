// SPDX-License-Identifier: Apache-2.0
#include "evse_security_file.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <system_error>

#include <everest/logging.hpp>

#include <openssl/asn1.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/x509.h>
#include <openssl/x509_vfy.h>

namespace charger {
namespace {

namespace fs = std::filesystem;

bool has_nonempty_file(const fs::path& path) {
    if (path.empty()) {
        return false;
    }
    std::error_code ec;
    if (!fs::exists(path, ec) || ec) {
        return false;
    }
    const auto size = fs::file_size(path, ec);
    if (ec) {
        return false;
    }
    return size > 0;
}

fs::path system_ca_bundle() {
    static constexpr std::array<const char*, 6> kCandidates = {
        "/etc/ssl/certs/ca-certificates.crt", // Debian/Ubuntu/Raspbian
        "/etc/ssl/certs/ca-bundle.crt",        // Alpine (often symlinked)
        "/etc/pki/tls/certs/ca-bundle.crt",    // RHEL/CentOS/Fedora
        "/etc/ssl/ca-bundle.pem",              // OpenSUSE
        "/etc/ssl/cert.pem",                   // macOS (system OpenSSL compatibility bundle)
        "/etc/pki/tls/cert.pem",               // some distros
    };
    for (const auto* candidate : kCandidates) {
        const fs::path p(candidate);
        if (has_nonempty_file(p)) {
            return p;
        }
    }
    return {};
}

fs::path resolve_bundle_path(const fs::path& configured, const fs::path& fallback) {
    if (has_nonempty_file(configured)) {
        return configured;
    }
    if (has_nonempty_file(fallback)) {
        return fallback;
    }
    return {};
}

bool write_file_atomic(const fs::path& path, const std::string& content) {
    if (path.empty()) {
        return false;
    }
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    if (ec) {
        return false;
    }
    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            return false;
        }
        out.write(content.data(), static_cast<std::streamsize>(content.size()));
        out.flush();
        if (!out) {
            return false;
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

std::optional<std::string> read_file(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

struct ParsedCertChain {
    // leaf is always set on success.
    X509* leaf{nullptr};
    std::vector<X509*> chain; // intermediates, may be empty

    ~ParsedCertChain() {
        if (leaf) {
            X509_free(leaf);
            leaf = nullptr;
        }
        for (auto* c : chain) {
            X509_free(c);
        }
        chain.clear();
    }

    ParsedCertChain(const ParsedCertChain&) = delete;
    ParsedCertChain& operator=(const ParsedCertChain&) = delete;
    ParsedCertChain() = default;
};

std::unique_ptr<ParsedCertChain> parse_pem_chain(const std::string& pem) {
    auto out = std::make_unique<ParsedCertChain>();
    BIO* bio = BIO_new_mem_buf(pem.data(), static_cast<int>(pem.size()));
    if (!bio) {
        return nullptr;
    }
    // Read all certs from PEM; first is leaf.
    while (true) {
        X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
        if (!cert) {
            const unsigned long err = ERR_peek_last_error();
            if (err == 0) {
                break;
            }
            // Clear error queue.
            while (ERR_get_error() != 0) {}
            break;
        }
        if (!out->leaf) {
            out->leaf = cert;
        } else {
            out->chain.push_back(cert);
        }
    }
    BIO_free(bio);
    if (!out->leaf) {
        return nullptr;
    }
    return out;
}

std::unique_ptr<X509_STORE, decltype(&X509_STORE_free)> make_store_from_bundle(const fs::path& bundle) {
    std::unique_ptr<X509_STORE, decltype(&X509_STORE_free)> store(X509_STORE_new(), X509_STORE_free);
    if (!store) {
        return {nullptr, X509_STORE_free};
    }
    if (bundle.empty()) {
        return store;
    }
    // load both as file and as directory depending on what bundle points to.
    const std::string p = bundle.string();
    if (fs::is_directory(bundle)) {
        if (X509_STORE_load_locations(store.get(), nullptr, p.c_str()) != 1) {
            return {nullptr, X509_STORE_free};
        }
    } else {
        if (X509_STORE_load_locations(store.get(), p.c_str(), nullptr) != 1) {
            return {nullptr, X509_STORE_free};
        }
    }
    return store;
}

ocpp::CertificateValidationResult map_verify_error(int err) {
    switch (err) {
    case X509_V_OK:
        return ocpp::CertificateValidationResult::Valid;
    case X509_V_ERR_CERT_HAS_EXPIRED:
    case X509_V_ERR_CERT_NOT_YET_VALID:
        return ocpp::CertificateValidationResult::Expired;
    case X509_V_ERR_CERT_SIGNATURE_FAILURE:
    case X509_V_ERR_UNABLE_TO_DECRYPT_CERT_SIGNATURE:
        return ocpp::CertificateValidationResult::InvalidSignature;
    case X509_V_ERR_UNABLE_TO_GET_ISSUER_CERT:
    case X509_V_ERR_UNABLE_TO_GET_ISSUER_CERT_LOCALLY:
    case X509_V_ERR_UNABLE_TO_VERIFY_LEAF_SIGNATURE:
        return ocpp::CertificateValidationResult::IssuerNotFound;
    default:
        return ocpp::CertificateValidationResult::InvalidChain;
    }
}

ocpp::InstallCertificateResult map_install_result(ocpp::CertificateValidationResult v) {
    switch (v) {
    case ocpp::CertificateValidationResult::Valid:
        return ocpp::InstallCertificateResult::Accepted;
    case ocpp::CertificateValidationResult::Expired:
        return ocpp::InstallCertificateResult::Expired;
    case ocpp::CertificateValidationResult::InvalidSignature:
        return ocpp::InstallCertificateResult::InvalidSignature;
    case ocpp::CertificateValidationResult::IssuerNotFound:
    case ocpp::CertificateValidationResult::InvalidChain:
    default:
        return ocpp::InstallCertificateResult::InvalidCertificateChain;
    }
}

ocpp::CertificateValidationResult verify_chain_against_bundle(const std::string& pem, const fs::path& bundle) {
    auto chain = parse_pem_chain(pem);
    if (!chain) {
        return ocpp::CertificateValidationResult::Unknown;
    }
    auto store = make_store_from_bundle(bundle);
    if (!store) {
        return ocpp::CertificateValidationResult::Unknown;
    }
    STACK_OF(X509)* untrusted = sk_X509_new_null();
    if (!untrusted) {
        return ocpp::CertificateValidationResult::Unknown;
    }
    for (auto* c : chain->chain) {
        sk_X509_push(untrusted, c);
    }
    X509_STORE_CTX* ctx = X509_STORE_CTX_new();
    if (!ctx) {
        sk_X509_free(untrusted);
        return ocpp::CertificateValidationResult::Unknown;
    }
    if (X509_STORE_CTX_init(ctx, store.get(), chain->leaf, untrusted) != 1) {
        X509_STORE_CTX_free(ctx);
        sk_X509_free(untrusted);
        return ocpp::CertificateValidationResult::Unknown;
    }

    const int ok = X509_verify_cert(ctx);
    int err = X509_STORE_CTX_get_error(ctx);
    X509_STORE_CTX_free(ctx);
    sk_X509_free(untrusted);

    if (ok == 1) {
        err = X509_V_OK;
    }
    return map_verify_error(err);
}

std::string trim_copy(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

bool is_service_name_safe(const std::string& s) {
    if (s.empty()) return false;
    for (const char c : s) {
        const bool ok = std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_' || c == '@';
        if (!ok) return false;
    }
    return true;
}

} // namespace

FileEvseSecurity::FileEvseSecurity(Paths paths, bool permissive) : permissive_(permissive), paths_(std::move(paths)) {}

fs::path FileEvseSecurity::bundle_path_for_ca(const ocpp::CaCertificateType& certificate_type) const {
    switch (certificate_type) {
    case ocpp::CaCertificateType::CSMS:
        return resolve_bundle_path(paths_.security.csms_ca_bundle, system_ca_bundle());
    case ocpp::CaCertificateType::MF:
        return paths_.security.mf_ca_bundle;
    case ocpp::CaCertificateType::MO:
        return paths_.security.mo_ca_bundle;
    case ocpp::CaCertificateType::V2G:
        return paths_.security.v2g_ca_bundle;
    case ocpp::CaCertificateType::OEM:
        return {};
    }
    return {};
}

fs::path FileEvseSecurity::bundle_path_for_leaf(const ocpp::LeafCertificateType& certificate_type) const {
    switch (certificate_type) {
    case ocpp::LeafCertificateType::CSMS:
        return resolve_bundle_path(paths_.security.csms_ca_bundle, system_ca_bundle());
    case ocpp::LeafCertificateType::MF:
        return paths_.security.mf_ca_bundle;
    case ocpp::LeafCertificateType::MO:
        return paths_.security.mo_ca_bundle;
    case ocpp::LeafCertificateType::V2G:
        return paths_.security.v2g_ca_bundle;
    }
    return {};
}

fs::path FileEvseSecurity::station_client_cert_chain_path() const {
    return paths_.security.client_cert_dir / (paths_.charge_point_id + "_cert.pem");
}

fs::path FileEvseSecurity::station_client_cert_single_path() const {
    return paths_.security.client_cert_dir / (paths_.charge_point_id + "_cert_single.pem");
}

fs::path FileEvseSecurity::station_client_key_path() const {
    return paths_.security.client_key_dir / (paths_.charge_point_id + "_key.pem");
}

std::optional<fs::path> FileEvseSecurity::discover_single_match(const fs::path& dir, const std::string& suffix) const {
    std::error_code ec;
    if (dir.empty() || !fs::exists(dir, ec) || ec || !fs::is_directory(dir, ec)) {
        return std::nullopt;
    }
    std::optional<fs::path> found;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file(ec) || ec) continue;
        const auto p = entry.path();
        const auto name = p.filename().string();
        if (name.size() >= suffix.size() && name.rfind(suffix) == (name.size() - suffix.size())) {
            if (has_nonempty_file(p)) {
                if (found.has_value()) {
                    return std::nullopt; // ambiguous
                }
                found = p;
            }
        }
    }
    return found;
}

ocpp::InstallCertificateResult FileEvseSecurity::install_ca_certificate(const std::string& certificate,
                                                                        const ocpp::CaCertificateType& certificate_type) {
    if (permissive_) {
        return ocpp::InstallCertificateResult::Accepted;
    }
    const auto target = bundle_path_for_ca(certificate_type);
    if (target.empty()) {
        return ocpp::InstallCertificateResult::WriteError;
    }
    const auto cert_trim = trim_copy(certificate);
    if (cert_trim.empty()) {
        return ocpp::InstallCertificateResult::InvalidFormat;
    }
    std::string next;
    if (auto cur = read_file(target)) {
        if (cur->find(cert_trim) != std::string::npos) {
            return ocpp::InstallCertificateResult::Accepted;
        }
        next = *cur;
        if (!next.empty() && next.back() != '\n') next.push_back('\n');
        next += cert_trim;
        if (next.back() != '\n') next.push_back('\n');
    } else {
        next = cert_trim;
        if (next.back() != '\n') next.push_back('\n');
    }
    if (!write_file_atomic(target, next)) {
        return ocpp::InstallCertificateResult::WriteError;
    }
    return ocpp::InstallCertificateResult::Accepted;
}

ocpp::DeleteCertificateResult FileEvseSecurity::delete_certificate(const ocpp::CertificateHashDataType&) {
    if (permissive_) {
        return ocpp::DeleteCertificateResult::Accepted;
    }
    // Minimal implementation: explicit deletion by hash is not supported yet.
    return ocpp::DeleteCertificateResult::Failed;
}

ocpp::InstallCertificateResult
FileEvseSecurity::update_leaf_certificate(const std::string& certificate_chain,
                                          const ocpp::CertificateSigningUseEnum& certificate_type) {
    if (permissive_) {
        return ocpp::InstallCertificateResult::Accepted;
    }

    // Only implement ChargingStationCertificate for now (OCPP client TLS leaf).
    if (certificate_type != ocpp::CertificateSigningUseEnum::ChargingStationCertificate) {
        return ocpp::InstallCertificateResult::InvalidFormat;
    }

    const auto ca_bundle = bundle_path_for_ca(ocpp::CaCertificateType::CSMS);
    if (ca_bundle.empty() || !has_nonempty_file(ca_bundle)) {
        return ocpp::InstallCertificateResult::NoRootCertificateInstalled;
    }

    // Verify the leaf chain before installing.
    const auto vr = verify_chain_against_bundle(certificate_chain, ca_bundle);
    if (vr != ocpp::CertificateValidationResult::Valid) {
        return map_install_result(vr);
    }

    // Ensure a private key exists (required by libocpp for SecurityProfile 3).
    const auto key_path = station_client_key_path();
    if (!has_nonempty_file(key_path)) {
        EVLOG_error << "ChargingStationCertificate key missing: " << key_path;
        return ocpp::InstallCertificateResult::WriteError;
    }

    // Persist full chain and the single leaf (first cert).
    auto parsed = parse_pem_chain(certificate_chain);
    if (!parsed) {
        return ocpp::InstallCertificateResult::InvalidFormat;
    }

    // Write the full chain as provided.
    if (!write_file_atomic(station_client_cert_chain_path(), certificate_chain)) {
        return ocpp::InstallCertificateResult::WriteError;
    }

    // Serialize leaf cert to PEM.
    std::string leaf_pem;
    BIO* bio = BIO_new(BIO_s_mem());
    if (!bio) {
        return ocpp::InstallCertificateResult::WriteError;
    }
    if (PEM_write_bio_X509(bio, parsed->leaf) != 1) {
        BIO_free(bio);
        return ocpp::InstallCertificateResult::WriteError;
    }
    char* data = nullptr;
    long len = BIO_get_mem_data(bio, &data);
    if (len > 0 && data) {
        leaf_pem.assign(data, static_cast<std::size_t>(len));
    }
    BIO_free(bio);
    if (leaf_pem.empty()) {
        return ocpp::InstallCertificateResult::WriteError;
    }
    if (!write_file_atomic(station_client_cert_single_path(), leaf_pem)) {
        return ocpp::InstallCertificateResult::WriteError;
    }

    return ocpp::InstallCertificateResult::Accepted;
}

ocpp::CertificateValidationResult
FileEvseSecurity::verify_certificate(const std::string& certificate_chain,
                                     const ocpp::LeafCertificateType& certificate_type) {
    if (permissive_) {
        return ocpp::CertificateValidationResult::Valid;
    }
    const auto bundle = bundle_path_for_leaf(certificate_type);
    if (bundle.empty() || !has_nonempty_file(bundle)) {
        return ocpp::CertificateValidationResult::IssuerNotFound;
    }
    return verify_chain_against_bundle(certificate_chain, bundle);
}

ocpp::CertificateValidationResult
FileEvseSecurity::verify_certificate(const std::string& certificate_chain,
                                     const std::vector<ocpp::LeafCertificateType>& certificate_types) {
    if (permissive_) {
        return ocpp::CertificateValidationResult::Valid;
    }
    ocpp::CertificateValidationResult last = ocpp::CertificateValidationResult::Unknown;
    for (const auto& t : certificate_types) {
        const auto r = verify_certificate(certificate_chain, t);
        if (r == ocpp::CertificateValidationResult::Valid) {
            return r;
        }
        last = r;
    }
    return last;
}

std::vector<ocpp::CertificateHashDataChain>
FileEvseSecurity::get_installed_certificates(const std::vector<ocpp::CertificateType>&) {
    // Not required for TLS connectivity; implement when CSMS requires GetInstalledCertificateIds.
    return {};
}

std::vector<ocpp::OCSPRequestData> FileEvseSecurity::get_v2g_ocsp_request_data() { return {}; }

std::vector<ocpp::OCSPRequestData> FileEvseSecurity::get_mo_ocsp_request_data(const std::string&) { return {}; }

void FileEvseSecurity::update_ocsp_cache(const ocpp::CertificateHashDataType&, const std::string&) {}

bool FileEvseSecurity::is_ca_certificate_installed(const ocpp::CaCertificateType& certificate_type) {
    if (permissive_) {
        return true;
    }
    const auto bundle = bundle_path_for_ca(certificate_type);
    return has_nonempty_file(bundle);
}

ocpp::GetCertificateSignRequestResult
FileEvseSecurity::generate_certificate_signing_request(const ocpp::CertificateSigningUseEnum& certificate_type,
                                                       const std::string& country, const std::string& organization,
                                                       const std::string& common, bool) {
    if (permissive_) {
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }
    if (certificate_type != ocpp::CertificateSigningUseEnum::ChargingStationCertificate) {
        return {ocpp::GetCertificateSignRequestStatus::InvalidRequestedType, std::nullopt};
    }

    // Generate or reuse private key.
    EVP_PKEY* pkey = nullptr;
    const auto key_path = station_client_key_path();
    if (has_nonempty_file(key_path)) {
        if (auto key_pem = read_file(key_path)) {
            BIO* key_bio = BIO_new_mem_buf(key_pem->data(), static_cast<int>(key_pem->size()));
            if (key_bio) {
                pkey = PEM_read_bio_PrivateKey(key_bio, nullptr, nullptr, nullptr);
                BIO_free(key_bio);
            }
        }
    }
    if (!pkey) {
        EVP_PKEY_CTX* ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, nullptr);
        if (!ctx) {
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        if (EVP_PKEY_keygen_init(ctx) != 1) {
            EVP_PKEY_CTX_free(ctx);
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        if (EVP_PKEY_CTX_set_rsa_keygen_bits(ctx, 2048) != 1) {
            EVP_PKEY_CTX_free(ctx);
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        if (EVP_PKEY_keygen(ctx, &pkey) != 1) {
            EVP_PKEY_CTX_free(ctx);
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        EVP_PKEY_CTX_free(ctx);

        // Persist key to disk.
        BIO* out = BIO_new(BIO_s_mem());
        if (!out) {
            EVP_PKEY_free(pkey);
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        if (PEM_write_bio_PrivateKey(out, pkey, nullptr, nullptr, 0, nullptr, nullptr) != 1) {
            BIO_free(out);
            EVP_PKEY_free(pkey);
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        char* data = nullptr;
        long len = BIO_get_mem_data(out, &data);
        std::string key_str;
        if (len > 0 && data) {
            key_str.assign(data, static_cast<std::size_t>(len));
        }
        BIO_free(out);
        if (key_str.empty() || !write_file_atomic(key_path, key_str)) {
            EVP_PKEY_free(pkey);
            return {ocpp::GetCertificateSignRequestStatus::KeyGenError, std::nullopt};
        }
        // Best-effort permissions.
        std::error_code ec;
        fs::permissions(key_path, fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::replace, ec);
    }

    X509_REQ* req = X509_REQ_new();
    if (!req) {
        EVP_PKEY_free(pkey);
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }
    X509_REQ_set_pubkey(req, pkey);
    X509_NAME* name = X509_NAME_new();
    if (!name) {
        X509_REQ_free(req);
        EVP_PKEY_free(pkey);
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }
    if (!country.empty()) {
        X509_NAME_add_entry_by_txt(name, "C", MBSTRING_ASC,
                                   reinterpret_cast<const unsigned char*>(country.c_str()), -1, -1, 0);
    }
    if (!organization.empty()) {
        X509_NAME_add_entry_by_txt(name, "O", MBSTRING_ASC,
                                   reinterpret_cast<const unsigned char*>(organization.c_str()), -1, -1, 0);
    }
    if (!common.empty()) {
        X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                                   reinterpret_cast<const unsigned char*>(common.c_str()), -1, -1, 0);
    }
    X509_REQ_set_subject_name(req, name);
    X509_NAME_free(name);

    if (X509_REQ_sign(req, pkey, EVP_sha256()) <= 0) {
        X509_REQ_free(req);
        EVP_PKEY_free(pkey);
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }

    BIO* bio = BIO_new(BIO_s_mem());
    if (!bio) {
        X509_REQ_free(req);
        EVP_PKEY_free(pkey);
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }
    if (PEM_write_bio_X509_REQ(bio, req) != 1) {
        BIO_free(bio);
        X509_REQ_free(req);
        EVP_PKEY_free(pkey);
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }
    char* data = nullptr;
    long len = BIO_get_mem_data(bio, &data);
    std::string csr;
    if (len > 0 && data) {
        csr.assign(data, static_cast<std::size_t>(len));
    }
    BIO_free(bio);
    X509_REQ_free(req);
    EVP_PKEY_free(pkey);

    if (csr.empty()) {
        return {ocpp::GetCertificateSignRequestStatus::GenerationError, std::nullopt};
    }
    return {ocpp::GetCertificateSignRequestStatus::Accepted, csr};
}

ocpp::GetCertificateInfoResult
FileEvseSecurity::get_leaf_certificate_info(const ocpp::CertificateSigningUseEnum& certificate_type, bool) {
    ocpp::GetCertificateInfoResult res;
    if (permissive_) {
        res.status = ocpp::GetCertificateInfoStatus::NotFound;
        return res;
    }
    if (certificate_type != ocpp::CertificateSigningUseEnum::ChargingStationCertificate) {
        res.status = ocpp::GetCertificateInfoStatus::NotFound;
        return res;
    }

    fs::path chain = station_client_cert_chain_path();
    fs::path single = station_client_cert_single_path();
    fs::path key = station_client_key_path();

    // Fallback discovery if strict names are missing.
    if (!has_nonempty_file(chain)) {
        if (auto found = discover_single_match(paths_.security.client_cert_dir, "_cert.pem")) {
            chain = *found;
        }
    }
    if (!has_nonempty_file(key)) {
        if (auto found = discover_single_match(paths_.security.client_key_dir, "_key.pem")) {
            key = *found;
        }
    }
    if (!has_nonempty_file(chain) || !has_nonempty_file(key)) {
        res.status = !has_nonempty_file(key) ? ocpp::GetCertificateInfoStatus::PrivateKeyNotFound
                                             : ocpp::GetCertificateInfoStatus::NotFound;
        return res;
    }
    if (!has_nonempty_file(single)) {
        // If single leaf file is missing, we can still proceed; libocpp accepts chain path too.
        single.clear();
    }

    // Count certs in chain file.
    int count = 0;
    if (auto pem = read_file(chain)) {
        BIO* bio = BIO_new_mem_buf(pem->data(), static_cast<int>(pem->size()));
        if (bio) {
            while (true) {
                X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
                if (!cert) {
                    break;
                }
                count++;
                X509_free(cert);
            }
            BIO_free(bio);
            while (ERR_get_error() != 0) {}
        }
    }

    ocpp::CertificateInfo info;
    info.certificate_path = chain;
    if (!single.empty()) {
        info.certificate_single_path = single;
    }
    info.certificate_count = std::max(1, count);
    info.key_path = key;
    res.status = ocpp::GetCertificateInfoStatus::Accepted;
    res.info = info;
    return res;
}

bool FileEvseSecurity::update_certificate_links(const ocpp::CertificateSigningUseEnum&) {
    return false;
}

std::string FileEvseSecurity::get_verify_file(const ocpp::CaCertificateType& certificate_type) {
    const auto path = bundle_path_for_ca(certificate_type);
    return path.string();
}

std::string FileEvseSecurity::get_verify_location(const ocpp::CaCertificateType& certificate_type) {
    // libocpp's websocket backend uses this as the single input for SSL_CTX_load_verify_locations and
    // checks if it is a directory. Return the file path so bundles work without c_rehash.
    const auto path = bundle_path_for_ca(certificate_type);
    return path.string();
}

int FileEvseSecurity::get_leaf_expiry_days_count(const ocpp::CertificateSigningUseEnum& certificate_type) {
    if (permissive_) {
        return -1;
    }
    if (certificate_type != ocpp::CertificateSigningUseEnum::ChargingStationCertificate) {
        return -1;
    }
    const auto info = get_leaf_certificate_info(certificate_type, false);
    if (info.status != ocpp::GetCertificateInfoStatus::Accepted || !info.info.has_value()) {
        return -1;
    }
    const auto cert_path = info.info->certificate_single_path ? *info.info->certificate_single_path
                                                              : info.info->certificate_path.value();
    if (cert_path.empty() || !has_nonempty_file(cert_path)) {
        return -1;
    }
    auto pem = read_file(cert_path);
    if (!pem) {
        return -1;
    }
    auto parsed = parse_pem_chain(*pem);
    if (!parsed) {
        return -1;
    }
    const ASN1_TIME* not_after = X509_get0_notAfter(parsed->leaf);
    if (!not_after) {
        return -1;
    }
    struct tm tm_exp{};
    if (ASN1_TIME_to_tm(not_after, &tm_exp) != 1) {
        return -1;
    }
    const auto exp = std::chrono::system_clock::from_time_t(timegm(&tm_exp));
    const auto now = std::chrono::system_clock::now();
    const auto diff = exp - now;
    const auto days = std::chrono::duration_cast<std::chrono::hours>(diff).count() / 24;
    return static_cast<int>(days);
}

} // namespace charger
