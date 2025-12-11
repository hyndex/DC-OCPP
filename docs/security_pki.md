TLS / PKI Provisioning Guide
============================

This charger can run fully over TLS for both OCPP 1.6 and ISO 15118 HLC. Populate the certificate
and key material below before enabling production security profiles.

Paths (defaulted from `configs/charger.json`)
---------------------------------------------
- OCPP CSMS trust: `data/certs/ca/csms/CSMS_ROOT_CA.pem`
- OCPP client cert/key (per station): `data/certs/client/csms/`
- MO/V2G trust: `data/certs/ca/mo/MO_ROOT_CA.pem`, `data/certs/ca/v2g/V2G_ROOT_CA.pem`
- SECC client cert/key (used by PLC/HLC): `data/certs/client/cso/`

Provisioning steps
------------------
1) Place the CSMS root/intermediate CA chain into `csms/CSMS_ROOT_CA.pem` (PEM concatenated).
2) Drop the station client certificate and private key (PEM) into `data/certs/client/csms/`.
   - Key filename suffix `_key.pem`, certificate suffix `_cert.pem`.
3) Populate SECC client certificate/key in `data/certs/client/cso/` (same PEM naming rule).
4) For Plug & Charge testing, place the MO and V2G CA bundles into the `mo/` and `v2g/` paths.
5) Ensure file permissions restrict keys to the charger process owner (chmod 600).

Enabling TLS
------------
- Set the OCPP endpoint in `configs/charger.json` to `wss://...` and update `ocpp16-config.json`
  `SecurityProfile` as required by your CSMS.
- The adapter touches all certificate/key files on boot so libocpp can open them; missing files are
  created empty but must be populated with real material before connecting to production.

Key rotation and renewal
------------------------
- Replace PEM files in-place and restart the charger to reload.
- Maintain overlap during rotations by keeping old CAs in the bundle until all stations/CSMS nodes
  are updated.

ISO 15118 / HLC notes
---------------------
- HLC TLS handshake uses the SECC certificate/key pair and MO/V2G CA bundles above.
- PnC is not enabled in this release; certificates are staged for future enablement and to allow
  lab validation only.
