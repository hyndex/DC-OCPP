#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _nonempty_file(path: Path) -> bool:
    try:
        return path.is_file() and path.stat().st_size > 0
    except FileNotFoundError:
        return False


def _resolve_from(base_dir: Path, p: str) -> Path:
    # Match the charger's behavior: paths in configs are resolved relative to the config file.
    if not p:
        return (base_dir).resolve()
    pp = Path(p)
    if pp.is_absolute():
        return pp
    return (base_dir / pp).resolve(strict=False)


def _parse_charge_point_id(cfg: dict) -> str:
    cp = cfg.get("chargePoint", {}) if isinstance(cfg.get("chargePoint", {}), dict) else {}
    cp_id = str(cp.get("id", "chargepoint-1"))

    # Mirror charger_config.cpp behavior for ocppEndpointToBackend override.
    endpoint_override = cp.get("ocppEndpointToBackend") or cfg.get("OCPPEndpointToBackend") or ""
    endpoint_override = str(endpoint_override).strip()
    if endpoint_override:
        endpoint = endpoint_override.rstrip("/")
        scheme_pos = endpoint.find("://")
        after_scheme = 0 if scheme_pos < 0 else scheme_pos + 3
        last_slash = endpoint.rfind("/")
        if last_slash <= after_scheme or last_slash + 1 >= len(endpoint):
            raise SystemExit(f"Invalid ocppEndpointToBackend: {endpoint_override}")
        cp_id = endpoint[last_slash + 1 :]
    return cp_id


def _run(cmd: list[str], *, input_text: str | None = None) -> None:
    try:
        subprocess.run(
            cmd,
            check=True,
            input=(input_text.encode("utf-8") if input_text is not None else None),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except subprocess.CalledProcessError as e:
        out = (e.stdout or b"").decode("utf-8", errors="replace")
        raise SystemExit(f"Command failed ({' '.join(cmd)}):\n{out}") from e


def _chmod_private_key(path: Path) -> None:
    try:
        os.chmod(path, 0o600)
    except PermissionError:
        # Best-effort: service user might still own the key on the target device.
        pass


def _write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _gen_dev_ca(ca_key: Path, ca_cert: Path, *, days: int) -> None:
    ca_key.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.NamedTemporaryFile("w", delete=False) as f:
        f.write(
            "\n".join(
                [
                    "[ req ]",
                    "default_bits = 2048",
                    "prompt = no",
                    "distinguished_name = dn",
                    "x509_extensions = v3_ca",
                    "",
                    "[ dn ]",
                    "O = Joulepoint",
                    "CN = Joulepoint Dev OCPP CA",
                    "",
                    "[ v3_ca ]",
                    "basicConstraints = critical,CA:TRUE",
                    "keyUsage = critical,keyCertSign,cRLSign",
                    "subjectKeyIdentifier = hash",
                    "authorityKeyIdentifier = keyid:always,issuer",
                    "",
                ]
            )
        )
        openssl_cfg = f.name

    try:
        _run(
            [
                "openssl",
                "req",
                "-x509",
                "-new",
                "-nodes",
                "-keyout",
                str(ca_key),
                "-out",
                str(ca_cert),
                "-days",
                str(days),
                "-sha256",
                "-config",
                openssl_cfg,
            ]
        )
    finally:
        try:
            os.unlink(openssl_cfg)
        except FileNotFoundError:
            pass

    _chmod_private_key(ca_key)


def _gen_key_and_csr(key_path: Path, csr_path: Path, *, common_name: str, force_key: bool) -> None:
    csr_path.parent.mkdir(parents=True, exist_ok=True)
    subj = f"/O=Joulepoint/CN={common_name}"
    if _nonempty_file(key_path) and not force_key:
        _run(["openssl", "req", "-new", "-key", str(key_path), "-out", str(csr_path), "-subj", subj])
        return

    _run(
        [
            "openssl",
            "req",
            "-new",
            "-newkey",
            "rsa:2048",
            "-nodes",
            "-keyout",
            str(key_path),
            "-out",
            str(csr_path),
            "-subj",
            subj,
        ]
    )
    _chmod_private_key(key_path)


def _sign_client_cert(
    *,
    ca_key: Path,
    ca_cert: Path,
    csr_path: Path,
    cert_out: Path,
    days: int,
) -> None:
    cert_out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", delete=False) as f:
        f.write(
            "\n".join(
                [
                    "basicConstraints = critical,CA:FALSE",
                    "keyUsage = critical,digitalSignature,keyEncipherment",
                    "extendedKeyUsage = clientAuth",
                    "",
                ]
            )
        )
        extfile = f.name

    try:
        _run(
            [
                "openssl",
                "x509",
                "-req",
                "-in",
                str(csr_path),
                "-CA",
                str(ca_cert),
                "-CAkey",
                str(ca_key),
                "-CAcreateserial",
                "-out",
                str(cert_out),
                "-days",
                str(days),
                "-sha256",
                "-extfile",
                extfile,
            ]
        )
    finally:
        try:
            os.unlink(extfile)
        except FileNotFoundError:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Provision an OCPP TLS client certificate/key in the naming scheme expected by dc_ocpp "
            "(SecurityProfile 3)."
        )
    )
    ap.add_argument(
        "--config",
        "-c",
        default="configs/charger.json",
        help="Path to charger.json (default: configs/charger.json)",
    )
    ap.add_argument(
        "--dev-ca",
        action="store_true",
        help="Generate a local dev CA and a client cert/key signed by it (for development only).",
    )
    ap.add_argument(
        "--csr-only",
        action="store_true",
        help="Only generate/refresh the private key and CSR (no certificate).",
    )
    ap.add_argument(
        "--force",
        action="store_true",
        help="Overwrite the existing client certificate/CSR if present (keeps existing private key by default).",
    )
    ap.add_argument(
        "--force-key",
        action="store_true",
        help="Regenerate the private key as well (dangerous; will invalidate an installed certificate).",
    )
    ap.add_argument("--days", type=int, default=365, help="Client certificate validity in days (default: 365).")
    ap.add_argument("--ca-days", type=int, default=3650, help="Dev CA validity in days (default: 3650).")
    args = ap.parse_args()

    if args.csr_only and args.dev_ca:
        raise SystemExit("Choose either --csr-only or --dev-ca (not both).")

    if shutil.which("openssl") is None:
        raise SystemExit("openssl not found in PATH; install openssl and retry.")

    cfg_path = Path(args.config).resolve(strict=False)
    if not cfg_path.exists():
        raise SystemExit(f"Config not found: {cfg_path}")
    base_dir = cfg_path.parent
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)

    charge_point_id = _parse_charge_point_id(cfg)
    security = cfg.get("security", {}) if isinstance(cfg.get("security", {}), dict) else {}

    client_cert_dir = _resolve_from(base_dir, str(security.get("clientCertDir", "data/certs/client/csms")))
    client_key_dir = _resolve_from(base_dir, str(security.get("clientKeyDir", "data/certs/client/csms")))

    client_cert_path = client_cert_dir / f"{charge_point_id}_cert.pem"
    client_cert_single_path = client_cert_dir / f"{charge_point_id}_cert_single.pem"
    client_key_path = client_key_dir / f"{charge_point_id}_key.pem"

    # Keep dev CA material alongside the configured client cert dir, so it's always writable and easy to locate.
    ca_dir = client_cert_dir / "_dev_ca"
    ca_key = ca_dir / "DEV_OCPP_CA.key"
    ca_cert = ca_dir / "DEV_OCPP_CA.pem"
    csr_path = ca_dir / f"{charge_point_id}.csr"
    leaf_cert_tmp = ca_dir / f"{charge_point_id}.leaf.pem"

    client_cert_dir.mkdir(parents=True, exist_ok=True)
    client_key_dir.mkdir(parents=True, exist_ok=True)
    ca_dir.mkdir(parents=True, exist_ok=True)

    if args.csr_only:
        if _nonempty_file(csr_path) and not args.force:
            print(f"CSR already exists: {csr_path}")
            return 0
        _gen_key_and_csr(client_key_path, csr_path, common_name=charge_point_id, force_key=args.force_key)
        print(f"Wrote key: {client_key_path}")
        print(f"Wrote CSR: {csr_path}")
        return 0

    if not args.dev_ca:
        raise SystemExit("Nothing to do. Use --dev-ca to autogenerate a dev certificate, or --csr-only.")

    if (_nonempty_file(client_cert_path) or _nonempty_file(client_cert_single_path)) and not args.force:
        print(f"Client certificate already exists: {client_cert_path}")
        print("Use --force to reissue it (keeps the existing private key unless --force-key is set).")
        return 0

    if not (_nonempty_file(ca_key) and _nonempty_file(ca_cert)):
        _gen_dev_ca(ca_key, ca_cert, days=args.ca_days)

    _gen_key_and_csr(client_key_path, csr_path, common_name=charge_point_id, force_key=args.force_key)
    _sign_client_cert(ca_key=ca_key, ca_cert=ca_cert, csr_path=csr_path, cert_out=leaf_cert_tmp, days=args.days)

    leaf_pem = leaf_cert_tmp.read_text(encoding="utf-8")
    ca_pem = ca_cert.read_text(encoding="utf-8")

    # Full chain for dc_ocpp/libocpp (leaf + issuing CA). The server still needs to trust this CA.
    _write_text(client_cert_path, leaf_pem.rstrip() + "\n" + ca_pem.rstrip() + "\n")
    _write_text(client_cert_single_path, leaf_pem.rstrip() + "\n")
    _chmod_private_key(client_key_path)

    print(f"Wrote client key:  {client_key_path}")
    print(f"Wrote client cert: {client_cert_path}")
    print(f"Wrote leaf cert:   {client_cert_single_path}")
    print("")
    print("CSMS trust note:")
    print(f"- Your CSMS must trust the issuing CA: {ca_cert}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

