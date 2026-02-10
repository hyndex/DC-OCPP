// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ocpp/common/evse_security.hpp>

namespace charger {

// File-based implementation of libocpp's EvseSecurity interface.
//
// Goals:
// - Enable OCPP 1.6 TLS (SecurityProfile 2/3) using CA bundles + client leaf certs from disk.
// - Provide MF certificate validation for SignedUpdateFirmware signature verification.
// - Support basic OCPP certificate provisioning flows (InstallCertificate / CertificateSigned).
class FileEvseSecurity : public ocpp::EvseSecurity {
public:
    struct Paths {
        std::string charge_point_id;
        SecurityConfig security;
    };

    // `permissive=true` is dev-only: bypasses verification and accepts writes.
    explicit FileEvseSecurity(Paths paths, bool permissive = false);
    ~FileEvseSecurity() override = default;

    ocpp::InstallCertificateResult install_ca_certificate(const std::string& certificate,
                                                          const ocpp::CaCertificateType& certificate_type) override;

    ocpp::DeleteCertificateResult delete_certificate(const ocpp::CertificateHashDataType& certificate_hash_data) override;

    ocpp::InstallCertificateResult update_leaf_certificate(const std::string& certificate_chain,
                                                           const ocpp::CertificateSigningUseEnum& certificate_type) override;

    ocpp::CertificateValidationResult verify_certificate(const std::string& certificate_chain,
                                                         const ocpp::LeafCertificateType& certificate_type) override;

    ocpp::CertificateValidationResult verify_certificate(
        const std::string& certificate_chain, const std::vector<ocpp::LeafCertificateType>& certificate_types) override;

    std::vector<ocpp::CertificateHashDataChain>
    get_installed_certificates(const std::vector<ocpp::CertificateType>& certificate_types) override;

    std::vector<ocpp::OCSPRequestData> get_v2g_ocsp_request_data() override;
    std::vector<ocpp::OCSPRequestData> get_mo_ocsp_request_data(const std::string& certificate_chain) override;
    void update_ocsp_cache(const ocpp::CertificateHashDataType& certificate_hash_data,
                           const std::string& ocsp_response) override;

    bool is_ca_certificate_installed(const ocpp::CaCertificateType& certificate_type) override;

    ocpp::GetCertificateSignRequestResult generate_certificate_signing_request(
        const ocpp::CertificateSigningUseEnum& certificate_type, const std::string& country,
        const std::string& organization, const std::string& common, bool use_tpm) override;

    ocpp::GetCertificateInfoResult get_leaf_certificate_info(const ocpp::CertificateSigningUseEnum& certificate_type,
                                                             bool include_ocsp = false) override;

    bool update_certificate_links(const ocpp::CertificateSigningUseEnum& certificate_type) override;

    std::string get_verify_file(const ocpp::CaCertificateType& certificate_type) override;
    std::string get_verify_location(const ocpp::CaCertificateType& certificate_type) override;

    int get_leaf_expiry_days_count(const ocpp::CertificateSigningUseEnum& certificate_type) override;

private:
    std::filesystem::path bundle_path_for_ca(const ocpp::CaCertificateType& certificate_type) const;
    std::filesystem::path bundle_path_for_leaf(const ocpp::LeafCertificateType& certificate_type) const;

    // ChargingStationCertificate paths (OCPP TLS client certificate for SecurityProfile 3)
    std::filesystem::path station_client_cert_chain_path() const;
    std::filesystem::path station_client_cert_single_path() const;
    std::filesystem::path station_client_key_path() const;

    // Discover cert/key paths even if the strict naming scheme isn't used (fallback).
    std::optional<std::filesystem::path> discover_single_match(const std::filesystem::path& dir,
                                                               const std::string& suffix) const;

    bool permissive_{false};
    Paths paths_;
};

} // namespace charger

