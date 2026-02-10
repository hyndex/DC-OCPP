#include "evse_security_file.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <unistd.h>

#include <openssl/bn.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>

using namespace charger;
namespace fs = std::filesystem;

namespace {

fs::path make_temp_dir() {
    auto base = fs::temp_directory_path() / ("dc_ocpp_tests_" + std::to_string(::getpid()));
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

std::string pem_from_pkey(EVP_PKEY* pkey) {
    BIO* bio = BIO_new(BIO_s_mem());
    assert(bio);
    assert(PEM_write_bio_PrivateKey(bio, pkey, nullptr, nullptr, 0, nullptr, nullptr) == 1);
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
    EVP_PKEY* pkey = EVP_PKEY_new();
    assert(pkey);
    RSA* rsa = RSA_new();
    assert(rsa);
    BIGNUM* e = BN_new();
    assert(e);
    assert(BN_set_word(e, RSA_F4) == 1);
    assert(RSA_generate_key_ex(rsa, 2048, e, nullptr) == 1);
    BN_free(e);
    assert(EVP_PKEY_assign_RSA(pkey, rsa) == 1); // pkey takes ownership
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

    // Extensions
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

    const EVP_MD* md = EVP_sha256();
    assert(X509_sign(cert, issuer_key ? issuer_key : subject_key, md) > 0);
    return cert;
}

} // namespace

int main() {
    const auto tmp = make_temp_dir();
    const auto ca_bundle = tmp / "mf_ca.pem";
    const auto cert_dir = tmp / "certs";
    const auto key_dir = tmp / "keys";
    fs::create_directories(cert_dir);
    fs::create_directories(key_dir);

    EVP_PKEY* ca_key = generate_rsa_key();
    X509* ca_cert = make_cert(ca_key, nullptr, nullptr, "MF Test CA", true, 365);
    const auto ca_pem = pem_from_x509(ca_cert);
    {
        std::ofstream out(ca_bundle);
        out << ca_pem;
    }

    EVP_PKEY* leaf_key = generate_rsa_key();
    X509* leaf_cert = make_cert(leaf_key, ca_cert, ca_key, "MF Signing Leaf", false, 30);
    const auto leaf_pem = pem_from_x509(leaf_cert);

    SecurityConfig sec{};
    sec.mf_ca_bundle = ca_bundle;
    sec.client_cert_dir = cert_dir;
    sec.client_key_dir = key_dir;

    FileEvseSecurity evse(FileEvseSecurity::Paths{.charge_point_id = "cp-test", .security = sec}, false);
    const auto vr = evse.verify_certificate(leaf_pem, ocpp::LeafCertificateType::MF);
    assert(vr == ocpp::CertificateValidationResult::Valid);

    // Bad CA should fail.
    SecurityConfig sec_bad = sec;
    sec_bad.mf_ca_bundle = tmp / "missing.pem";
    FileEvseSecurity evse_bad(FileEvseSecurity::Paths{.charge_point_id = "cp-test", .security = sec_bad}, false);
    const auto vr_bad = evse_bad.verify_certificate(leaf_pem, ocpp::LeafCertificateType::MF);
    assert(vr_bad == ocpp::CertificateValidationResult::IssuerNotFound);

    // ChargingStationCertificate discovery.
    const fs::path station_cert = cert_dir / "cp-test_cert.pem";
    const fs::path station_key = key_dir / "cp-test_key.pem";
    {
        std::ofstream out(station_cert);
        out << leaf_pem;
    }
    {
        std::ofstream out(station_key);
        out << pem_from_pkey(leaf_key);
    }

    const auto info = evse.get_leaf_certificate_info(ocpp::CertificateSigningUseEnum::ChargingStationCertificate, false);
    assert(info.status == ocpp::GetCertificateInfoStatus::Accepted);
    assert(info.info.has_value());
    assert(info.info->certificate_path.has_value());
    assert(info.info->certificate_path.value() == station_cert);
    assert(!info.info->key_path.empty());
    assert(info.info->key_path == station_key);

    const int days = evse.get_leaf_expiry_days_count(ocpp::CertificateSigningUseEnum::ChargingStationCertificate);
    assert(days >= 0);

    X509_free(leaf_cert);
    EVP_PKEY_free(leaf_key);
    X509_free(ca_cert);
    EVP_PKEY_free(ca_key);

    std::cout << "EVSE security file tests passed\n";
    return 0;
}
