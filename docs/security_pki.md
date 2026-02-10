TLS / PKI Provisioning Guide
============================

This charger supports OCPP 1.6 over TLS (SecurityProfile 2 and 3). Populate the certificate and
key material below before enabling production security profiles.

Notes
-----
- ISO 15118 contract-certificate management via the OCPP "PnC" profile is **not enabled** in this
  release. Autocharge (MAC/EVCCID/EMAID sourced from the PLC) is supported instead.
- HLC/SECC TLS (vehicle side) is controlled by the PLC stack; this document focuses on OCPP TLS.

Paths (defaulted from `configs/charger.json`)
---------------------------------------------
Paths are configured under `security` in `configs/charger.json` and resolved relative to that file.

- CSMS trust (CA bundle): `security.csmsCaBundle`
  - Default: `data/certs/ca/csms/CSMS_ROOT_CA.pem`
- Firmware signing trust (CA bundle): `security.mfCaBundle`
  - Used to validate `SignedUpdateFirmware` signing certificates (MF CA)
- Station TLS client certificate/key (SecurityProfile 3):
  - `security.clientCertDir`, `security.clientKeyDir`
  - File naming (required):
    - `${clientCertDir}/${charge_point_id}_cert.pem` (PEM chain)
    - `${clientKeyDir}/${charge_point_id}_key.pem` (PEM private key)
  - Fallback: if the exact file is missing, the charger will use the single `*_cert.pem` and
    `*_key.pem` in the respective directories when there is exactly one match.
- Optional (PLC/HLC):
  - MO/V2G trust: `security.moCaBundle`, `security.v2gCaBundle`
  - SECC cert/key directories: `security.seccCertDir`, `security.seccKeyDir`

Provisioning steps
------------------
1) Place the CSMS root/intermediate CA chain into `security.csmsCaBundle` (PEM concatenated).
2) For SecurityProfile 3, place the station client certificate chain and private key into the
   configured `security.clientCertDir` / `security.clientKeyDir` using the naming rules above.
3) If you will use `SignedUpdateFirmware`, place the MF root/intermediate CA chain into
   `security.mfCaBundle` (PEM concatenated).
4) Optional (PLC/HLC): place MO/V2G CA bundles and SECC key material into their configured paths.
5) Ensure file permissions restrict private keys to the charger process owner (e.g., `chmod 600`).

Enabling TLS
------------
- Set `chargePoint.centralSystemURI` in `configs/charger.json` to a TLS endpoint:
  - `wss://host:port/path`, or
  - scheme-less `host:port/path` (libocpp will use `wss://` for SecurityProfile 2/3).
- Set `ocpp.Security.SecurityProfile` in your base OCPP config:
  - `2`: TLS with server authentication (CA bundle required, no client cert)
  - `3`: mutual TLS (CA bundle + station client cert/key required)
- The charger rejects `ws://...` endpoints when SecurityProfile is 2 or 3.
- `DC_OCPP_STUB_SECURITY=1` is a dev-only bypass for SecurityProfile 0 and is **ignored** for TLS
  profiles (2/3).

Key rotation and renewal
------------------------
- Replace PEM files in-place and restart the charger to reload.
- Maintain overlap during rotations by keeping old CAs in the bundle until all stations/CSMS nodes
  are updated.

ISO 15118 / HLC notes
---------------------
- HLC TLS handshake uses the SECC certificate/key pair and MO/V2G CA bundles above.
- OCPP ISO 15118 contract-certificate management ("PnC") is intentionally disabled; do not enable
  the OCPP PnC feature profile unless the full contract-cert lifecycle is implemented and tested.
