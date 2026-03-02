#!/usr/bin/env python3
"""Standalone Maxwell MXR module test utility.

Workflow:
1) Scan bus for MXR modules (address + group).
2) User selects one module from discovered list.
3) User enters voltage (V), current (A), and duration (s).
4) Script drives module with ratio-based current command (0x0022) and
   prints telemetry every 2 seconds.

Protocol mapping follows the in-tree Maxwell driver in
src/power_module_controller.cpp.
"""

from __future__ import annotations

import argparse
import errno
import math
import select
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_MTU = struct.calcsize("=IB3x8s")
CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")

MAXWELL_PROT_NO = 0x060
MAXWELL_CONTROLLER_ADDR = 0xF0
MAXWELL_FUNC_SET = 0x03
MAXWELL_FUNC_READ = 0x10
MAXWELL_TYPE_FLOAT = 0x41
MAXWELL_TYPE_INT = 0x42
MAXWELL_OK = 0xF0

REG_GET_VOLTAGE = 0x0001
REG_GET_CURRENT = 0x0002
REG_GET_CURRENT_LIMIT_POINT = 0x0003
REG_GET_TEMP = 0x0004
REG_GET_RATED_CURRENT = 0x0012
REG_SET_CURRENT_ABS = 0x001B
REG_SET_VOLTAGE = 0x0021
REG_SET_CURRENT_LIMIT_RATIO = 0x0022
REG_ONOFF = 0x0030
REG_GET_STATUS = 0x0040
REG_GET_GROUP_ADDR = 0x0043
REG_GET_INPUT_POWER = 0x0048
REG_GET_INPUT_MODE = 0x004B
MAXWELL_CURRENT_LIMIT_RATIO_MAX_DEFAULT = 3.5


def log(msg: str) -> None:
    print(msg, flush=True)


def parse_int_auto(value: str) -> int:
    return int(value, 0)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def encode_float_be(value: float) -> int:
    return struct.unpack(">I", struct.pack(">f", value))[0]


def decode_float_be(raw: int) -> float:
    return struct.unpack(">f", raw.to_bytes(4, byteorder="big", signed=False))[0]


def build_can_id(dst_addr: int, group: int, src_addr: int = MAXWELL_CONTROLLER_ADDR, ptp: bool = True) -> int:
    can_id = 0
    can_id |= (MAXWELL_PROT_NO & 0x1FF) << 20
    can_id |= (1 if ptp else 0) << 19
    can_id |= (dst_addr & 0xFF) << 11
    can_id |= (src_addr & 0xFF) << 3
    can_id |= group & 0x7
    return can_id | CAN_EFF_FLAG


def parse_can_id(can_id: int) -> Tuple[int, int, int, int, int]:
    raw = can_id & CAN_EFF_MASK
    prot = (raw >> 20) & 0x1FF
    ptp = (raw >> 19) & 0x1
    dst_addr = (raw >> 11) & 0xFF
    src_addr = (raw >> 3) & 0xFF
    group = raw & 0x7
    return prot, ptp, dst_addr, src_addr, group


def send_can(sock: socket.socket, can_id: int, payload: bytes, retries: int = 5, retry_delay_s: float = 0.02) -> None:
    if len(payload) != 8:
        raise ValueError("CAN payload must be exactly 8 bytes")
    frame = CAN_FRAME_STRUCT.pack(can_id, 8, payload)
    attempt = 0
    while True:
        try:
            sock.send(frame)
            return
        except OSError as exc:
            if exc.errno != errno.ENOBUFS or attempt >= retries:
                raise
            attempt += 1
            time.sleep(retry_delay_s)


def send_read(sock: socket.socket, dst_addr: int, group: int, reg: int) -> None:
    payload = bytes(
        [
            MAXWELL_FUNC_READ,
            0x00,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )
    send_can(sock, build_can_id(dst_addr=dst_addr, group=group), payload)


def send_set_float(sock: socket.socket, dst_addr: int, group: int, reg: int, value: float) -> None:
    raw = encode_float_be(value)
    payload = bytes(
        [
            MAXWELL_FUNC_SET,
            0x00,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
            (raw >> 24) & 0xFF,
            (raw >> 16) & 0xFF,
            (raw >> 8) & 0xFF,
            raw & 0xFF,
        ]
    )
    send_can(sock, build_can_id(dst_addr=dst_addr, group=group), payload)


def send_set_int(sock: socket.socket, dst_addr: int, group: int, reg: int, value: int) -> None:
    raw = value & 0xFFFFFFFF
    payload = bytes(
        [
            MAXWELL_FUNC_SET,
            0x00,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
            (raw >> 24) & 0xFF,
            (raw >> 16) & 0xFF,
            (raw >> 8) & 0xFF,
            raw & 0xFF,
        ]
    )
    send_can(sock, build_can_id(dst_addr=dst_addr, group=group), payload)


def recv_one(sock: socket.socket, timeout_s: float) -> Optional[Tuple[int, bytes]]:
    ready, _, _ = select.select([sock], [], [], timeout_s)
    if not ready:
        return None
    frame = sock.recv(CAN_MTU)
    if len(frame) < CAN_MTU:
        return None
    can_id, can_dlc, data = CAN_FRAME_STRUCT.unpack(frame[:CAN_MTU])
    return can_id, data[:can_dlc]


@dataclass
class ModuleInfo:
    address: int
    group: int
    last_seen: float = 0.0
    status: Optional[int] = None
    voltage_v: Optional[float] = None
    current_a: Optional[float] = None
    current_limit_point: Optional[float] = None
    temp_c: Optional[float] = None
    input_mode: Optional[int] = None
    input_power_w: Optional[float] = None
    reported_group: Optional[int] = None
    reported_address: Optional[int] = None
    rated_current_a: Optional[float] = None

    def key(self) -> Tuple[int, int]:
        return self.address, self.group


def parse_maxwell_response(can_id: int, data: bytes) -> Optional[Dict[str, object]]:
    if len(data) < 8:
        return None
    if (can_id & CAN_EFF_FLAG) == 0:
        return None

    prot, _ptp, dst_addr, src_addr, group = parse_can_id(can_id)
    if prot != MAXWELL_PROT_NO:
        return None
    if dst_addr != MAXWELL_CONTROLLER_ADDR:
        return None

    resp_type = data[0]
    status = data[1]
    reg = (data[2] << 8) | data[3]
    raw = int.from_bytes(data[4:8], byteorder="big", signed=False)

    if resp_type == MAXWELL_TYPE_FLOAT:
        value: object = decode_float_be(raw)
    elif resp_type == MAXWELL_TYPE_INT:
        value = raw
    else:
        return None

    return {
        "src": src_addr,
        "group": group,
        "type": resp_type,
        "status": status,
        "reg": reg,
        "raw": raw,
        "value": value,
    }


def apply_response(module: ModuleInfo, resp: Dict[str, object]) -> None:
    status = int(resp["status"])
    if status != MAXWELL_OK:
        return
    reg = int(resp["reg"])
    value = resp["value"]
    module.last_seen = time.monotonic()

    if reg == REG_GET_STATUS and isinstance(value, int):
        module.status = value
    elif reg == REG_GET_VOLTAGE and isinstance(value, float) and math.isfinite(value):
        module.voltage_v = max(0.0, value)
    elif reg == REG_GET_CURRENT and isinstance(value, float) and math.isfinite(value):
        module.current_a = max(0.0, value)
    elif reg == REG_GET_CURRENT_LIMIT_POINT and isinstance(value, float) and math.isfinite(value):
        module.current_limit_point = max(0.0, value)
    elif reg == REG_GET_TEMP and isinstance(value, float) and math.isfinite(value):
        module.temp_c = value
    elif reg == REG_GET_INPUT_MODE and isinstance(value, int):
        module.input_mode = value
    elif reg == REG_GET_INPUT_POWER and isinstance(value, int):
        module.input_power_w = max(0.0, float(value))
    elif reg == REG_GET_GROUP_ADDR and isinstance(value, int):
        module.reported_group = (value >> 16) & 0xFFFF
        module.reported_address = value & 0xFFFF
    elif reg == REG_GET_RATED_CURRENT and isinstance(value, float) and math.isfinite(value) and value > 0.0:
        module.rated_current_a = value


def parse_group_spec(raw: str) -> List[int]:
    out: List[int] = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            lo_txt, hi_txt = token.split("-", 1)
            lo = parse_int_auto(lo_txt.strip())
            hi = parse_int_auto(hi_txt.strip())
            if hi < lo:
                lo, hi = hi, lo
            out.extend(range(lo, hi + 1))
        else:
            out.append(parse_int_auto(token))
    unique = sorted(set(out))
    for g in unique:
        if g < 0 or g > 7:
            raise ValueError(f"Invalid group {g}; valid range is 0..7")
    return unique


def scan_modules(
    sock: socket.socket,
    groups: List[int],
    addr_start: int,
    addr_end: int,
    send_gap_s: float,
    listen_s: float,
) -> List[ModuleInfo]:
    found: Dict[Tuple[int, int], ModuleInfo] = {}
    for group in groups:
        for addr in range(addr_start, addr_end + 1):
            send_read(sock, dst_addr=addr, group=group, reg=REG_GET_STATUS)
            if send_gap_s > 0:
                time.sleep(send_gap_s)

        deadline = time.monotonic() + listen_s
        while True:
            rem = deadline - time.monotonic()
            if rem <= 0:
                break
            pkt = recv_one(sock, min(0.05, rem))
            if pkt is None:
                continue
            can_id, data = pkt
            resp = parse_maxwell_response(can_id, data)
            if not resp:
                continue
            key = (int(resp["src"]), int(resp["group"]))
            mod = found.get(key)
            if mod is None:
                mod = ModuleInfo(address=key[0], group=key[1])
                found[key] = mod
            apply_response(mod, resp)

    return sorted(found.values(), key=lambda m: (m.group, m.address))


def poll_responses_until(sock: socket.socket, module: ModuleInfo, deadline: float) -> None:
    while True:
        rem = deadline - time.monotonic()
        if rem <= 0:
            return
        pkt = recv_one(sock, min(0.05, rem))
        if pkt is None:
            continue
        can_id, data = pkt
        resp = parse_maxwell_response(can_id, data)
        if not resp:
            continue
        if int(resp["src"]) != module.address or int(resp["group"]) != module.group:
            continue
        apply_response(module, resp)


def refresh_module_details(sock: socket.socket, module: ModuleInfo, response_window_s: float = 0.45) -> None:
    regs = [
        REG_GET_GROUP_ADDR,
        REG_GET_INPUT_MODE,
        REG_GET_RATED_CURRENT,
        REG_GET_STATUS,
        REG_GET_INPUT_POWER,
        REG_GET_VOLTAGE,
        REG_GET_CURRENT,
        REG_GET_CURRENT_LIMIT_POINT,
        REG_GET_TEMP,
    ]
    for reg in regs:
        send_read(sock, dst_addr=module.address, group=module.group, reg=reg)
        time.sleep(0.002)
    poll_responses_until(sock, module, time.monotonic() + response_window_s)


def status_flags(status: Optional[int]) -> List[str]:
    if status is None:
        return []
    flags: List[str] = []
    if status & (1 << 22):
        flags.append("OFF")
    if status & (1 << 23):
        flags.append("PWR_LIMIT")
    if status & (1 << 24):
        flags.append("TEMP_DERATE")
    if status & (1 << 25):
        flags.append("AC_LIMIT")
    if status & (1 << 17):
        flags.append("CUR_IMBAL")
    if status & (1 << 16):
        flags.append("CAN_FAIL")
    if status & (1 << 14):
        flags.append("AC_UNDERV")
    if status & (1 << 9):
        flags.append("AC_ABNORMAL")
    if status & (1 << 5):
        flags.append("MODE_MISMATCH")
    if status & (1 << 4):
        flags.append("MODE_DETECT_ERR")
    return flags


def fmt_opt(value: Optional[float], width: int, precision: int) -> str:
    if value is None or not math.isfinite(value):
        return "n/a".rjust(width)
    return f"{value:{width}.{precision}f}"


def classify_control_regime(
    measured_v: Optional[float],
    measured_i: Optional[float],
    target_v: float,
    target_i: float,
) -> str:
    if measured_v is None or measured_i is None or not math.isfinite(measured_v) or not math.isfinite(measured_i):
        return "unknown"
    if target_i <= 0.0:
        return "idle"
    v_tol = max(2.0, target_v * 0.01)
    i_tol = max(2.0, target_i * 0.10)
    v_close = abs(measured_v - target_v) <= v_tol
    i_close = abs(measured_i - target_i) <= i_tol
    if v_close and not i_close and measured_i < target_i:
        return "cv_load_limited"
    if i_close:
        return "current_tracking"
    return "transition"


def format_module_summary(module: ModuleInfo) -> str:
    status_text = f"0x{module.status:08X}" if module.status is not None else "n/a"
    flags = status_flags(module.status)
    flags_text = ",".join(flags) if flags else "-"
    rated_text = f"{module.rated_current_a:.1f}A" if module.rated_current_a is not None else "n/a"
    rep_text = (
        f"{module.reported_group}/{module.reported_address}"
        if module.reported_group is not None and module.reported_address is not None
        else "n/a"
    )
    return (
        f"addr={module.address:02d} grp={module.group} "
        f"V={fmt_opt(module.voltage_v, 7, 2)} "
        f"I={fmt_opt(module.current_a, 7, 2)} "
        f"Ilim={fmt_opt(module.current_limit_point, 6, 3)} "
        f"Pin={fmt_opt(module.input_power_w, 7, 0)}W "
        f"T={fmt_opt(module.temp_c, 6, 1)} "
        f"status={status_text} flags={flags_text} "
        f"mode={module.input_mode if module.input_mode is not None else 'n/a'} "
        f"ratedI={rated_text} rep={rep_text}"
    )


def print_discovered_modules(modules: List[ModuleInfo]) -> None:
    log("\nDiscovered modules:")
    for idx, mod in enumerate(modules):
        log(f"  [{idx}] {format_module_summary(mod)}")


def prompt_select_module(max_index: int) -> int:
    while True:
        raw = input(f"\nSelect module index [0..{max_index}] (q to quit): ").strip().lower()
        if raw in {"q", "quit", "exit"}:
            raise KeyboardInterrupt()
        try:
            idx = int(raw, 10)
        except ValueError:
            log("Invalid index.")
            continue
        if 0 <= idx <= max_index:
            return idx
        log("Index out of range.")


def prompt_float(prompt: str, default: Optional[float] = None, min_value: Optional[float] = None) -> float:
    while True:
        suffix = f" [{default}]" if default is not None else ""
        raw = input(f"{prompt}{suffix}: ").strip()
        if not raw and default is not None:
            value = default
        else:
            try:
                value = float(raw)
            except ValueError:
                log("Invalid number.")
                continue
        if min_value is not None and value < min_value:
            log(f"Value must be >= {min_value}")
            continue
        return value


def prompt_choice(prompt: str, choices: List[str], default: Optional[str] = None) -> str:
    valid = {c.strip().lower(): c.strip().lower() for c in choices}
    if not valid:
        raise ValueError("choices must not be empty")
    if default is None:
        default = choices[0]
    default_key = default.strip().lower()
    if default_key not in valid:
        raise ValueError("default must be one of choices")
    while True:
        raw = input(f"{prompt} [{default_key}] ({'/'.join(valid.keys())}): ").strip().lower()
        key = raw if raw else default_key
        if key in valid:
            return key
        log(f"Invalid choice. Valid: {', '.join(valid.keys())}")


def query_telemetry_snapshot(sock: socket.socket, module: ModuleInfo, response_window_s: float) -> None:
    regs = [
        REG_GET_STATUS,
        REG_GET_INPUT_POWER,
        REG_GET_VOLTAGE,
        REG_GET_CURRENT,
        REG_GET_CURRENT_LIMIT_POINT,
        REG_GET_TEMP,
    ]
    for reg in regs:
        send_read(sock, dst_addr=module.address, group=module.group, reg=reg)
        time.sleep(0.001)
    poll_responses_until(sock, module, time.monotonic() + response_window_s)


def module_on(sock: socket.socket, module: ModuleInfo, voltage_v: float, ratio: float) -> None:
    send_set_int(sock, module.address, module.group, REG_ONOFF, 0x00000000)
    time.sleep(0.01)
    send_set_float(sock, module.address, module.group, REG_SET_VOLTAGE, voltage_v)
    time.sleep(0.01)
    send_set_float(sock, module.address, module.group, REG_SET_CURRENT_LIMIT_RATIO, ratio)


def module_on_absolute(sock: socket.socket, module: ModuleInfo, voltage_v: float, current_a: float, scale: float) -> int:
    current_raw = int(round(max(0.0, current_a) * scale))
    current_raw = max(0, min(current_raw, 0xFFFFFFFF))
    send_set_int(sock, module.address, module.group, REG_ONOFF, 0x00000000)
    time.sleep(0.01)
    send_set_float(sock, module.address, module.group, REG_SET_VOLTAGE, voltage_v)
    time.sleep(0.01)
    send_set_int(sock, module.address, module.group, REG_SET_CURRENT_ABS, current_raw)
    return current_raw


def module_off(sock: socket.socket, module: ModuleInfo) -> None:
    send_set_float(sock, module.address, module.group, REG_SET_CURRENT_LIMIT_RATIO, 0.0)
    time.sleep(0.01)
    send_set_int(sock, module.address, module.group, REG_ONOFF, 0x00010000)


def soft_reset_defaults(sock: socket.socket, module: ModuleInfo, input_mode: int = 3) -> None:
    # Maxwell V1.50 has no explicit factory-reset opcode; this applies a safe reset/default sequence.
    module_off(sock, module)
    time.sleep(0.08)

    # Clear resettable fault latches.
    send_set_int(sock, module.address, module.group, 0x0031, 0x00010000)  # overvoltage reset
    time.sleep(0.02)
    send_set_int(sock, module.address, module.group, 0x0031, 0x00000000)
    time.sleep(0.02)
    send_set_int(sock, module.address, module.group, 0x0044, 0x00010000)  # short-circuit reset
    time.sleep(0.02)
    send_set_int(sock, module.address, module.group, 0x0044, 0x00000000)
    time.sleep(0.02)

    # Re-apply known-good defaults.
    send_set_int(sock, module.address, module.group, 0x003E, 0x00000000)  # OVP relevance enable
    time.sleep(0.02)
    send_set_int(sock, module.address, module.group, 0x0017, 1000)  # altitude default
    time.sleep(0.02)
    send_set_int(sock, module.address, module.group, 0x0046, input_mode)  # input mode
    time.sleep(0.02)
    send_set_float(sock, module.address, module.group, 0x0020, 1.0)  # output power limit point
    time.sleep(0.02)
    send_set_float(sock, module.address, module.group, 0x0023, 1000.0)  # output voltage upper limit
    time.sleep(0.02)
    send_set_float(sock, module.address, module.group, REG_SET_CURRENT_LIMIT_RATIO, 0.0)
    time.sleep(0.04)

    refresh_module_details(sock, module, response_window_s=0.5)


def run_constant_output_test(
    sock: socket.socket,
    module: ModuleInfo,
    voltage_v: float,
    current_a: float,
    duration_s: float,
    rated_current_a: float,
    current_control_mode: str,
    ratio_max: float,
    abs_current_scale: float,
    control_interval_s: float,
    telemetry_interval_s: float,
    telemetry_response_window_s: float,
) -> None:
    mode = current_control_mode.strip().lower()
    if mode not in {"ratio", "absolute"}:
        raise ValueError(f"Unsupported current control mode: {current_control_mode}")
    ratio = 0.0 if current_a <= 0.0 else clamp(current_a / rated_current_a, 0.0, ratio_max)
    requested_ratio = 0.0 if current_a <= 0.0 else (current_a / rated_current_a)
    abs_current_raw = int(round(max(0.0, current_a) * abs_current_scale))
    abs_current_raw = max(0, min(abs_current_raw, 0xFFFFFFFF))
    log("\nStarting test:")
    log(f"  module       : addr={module.address} group={module.group}")
    log(f"  current mode : {mode}")
    log(f"  set voltage  : {voltage_v:.2f} V")
    log(f"  set current  : {current_a:.2f} A")
    if mode == "ratio":
        log(f"  rated current: {rated_current_a:.2f} A")
        log(f"  ratio (0x0022): {ratio:.6f}")
    else:
        log(f"  abs register (0x001B): {abs_current_raw} (scale={abs_current_scale:g})")
    log(f"  duration     : {duration_s:.1f} s")
    if mode == "ratio" and requested_ratio > ratio_max:
        log(f"  NOTE: requested ratio {requested_ratio:.6f} exceeded max ratio {ratio_max:.6f}; clamped")

    started = False
    t0 = time.monotonic()
    next_control = t0
    next_telemetry = t0

    try:
        if mode == "ratio":
            module_on(sock, module, voltage_v, ratio)
        else:
            module_on_absolute(sock, module, voltage_v, current_a, abs_current_scale)
        started = True
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= duration_s:
                break

            if now >= next_control:
                if mode == "ratio":
                    module_on(sock, module, voltage_v, ratio)
                else:
                    module_on_absolute(sock, module, voltage_v, current_a, abs_current_scale)
                next_control += control_interval_s

            if now >= next_telemetry:
                query_telemetry_snapshot(sock, module, response_window_s=telemetry_response_window_s)
                status_text = f"0x{module.status:08X}" if module.status is not None else "n/a"
                flags = status_flags(module.status)
                flags_text = ",".join(flags) if flags else "-"
                log(
                    f"[TEL +{elapsed:6.1f}s] "
                    f"V={fmt_opt(module.voltage_v, 7, 2)}V "
                    f"I={fmt_opt(module.current_a, 7, 2)}A "
                    f"Ilim={fmt_opt(module.current_limit_point, 6, 3)} "
                    f"Pin={fmt_opt(module.input_power_w, 7, 0)}W "
                    f"T={fmt_opt(module.temp_c, 6, 1)}C "
                    f"status={status_text} flags={flags_text} "
                    f"regime={classify_control_regime(module.voltage_v, module.current_a, voltage_v, current_a)}"
                )
                next_telemetry += telemetry_interval_s

            poll_responses_until(sock, module, time.monotonic() + 0.03)
    finally:
        if started:
            log("\nStopping module output...")
            for _ in range(2):
                module_off(sock, module)
                time.sleep(0.08)
            query_telemetry_snapshot(sock, module, response_window_s=0.25)
            log(f"Final telemetry: {format_module_summary(module)}")


def warn_if_dc_ocpp_running() -> None:
    try:
        proc = subprocess.run(
            ["pgrep", "-x", "dc_ocpp"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return
    if proc.returncode == 0 and proc.stdout.strip():
        log(
            "[WARN] dc_ocpp appears to be running. "
            "For clean standalone module testing, stop dc_ocpp first."
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Standalone Maxwell MXR module test (scan/select/constant output).")
    parser.add_argument("--iface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument(
        "--scan-groups",
        default="0-7",
        help="Groups to scan, e.g. '0-7' or '1,2' (default: 0-7)",
    )
    parser.add_argument(
        "--addr-start",
        type=parse_int_auto,
        default=0,
        help="Start address for scan (default: 0)",
    )
    parser.add_argument(
        "--addr-end",
        type=parse_int_auto,
        default=63,
        help="End address for scan (default: 63)",
    )
    parser.add_argument(
        "--scan-send-gap-ms",
        type=float,
        default=1.0,
        help="Delay between probe reads during scan (default: 1.0 ms)",
    )
    parser.add_argument(
        "--scan-listen-ms",
        type=float,
        default=300.0,
        help="Listen window after each group scan sweep (default: 300 ms)",
    )
    parser.add_argument(
        "--default-rated-current-a",
        type=float,
        default=100.0,
        help="Fallback rated current used for ratio conversion when module readback is unavailable (default: 100 A)",
    )
    parser.add_argument(
        "--current-control",
        choices=["ratio", "absolute"],
        default="ratio",
        help="Current-control mode: ratio uses 0x0022, absolute uses 0x001B (default: ratio)",
    )
    parser.add_argument(
        "--ratio-max",
        type=float,
        default=MAXWELL_CURRENT_LIMIT_RATIO_MAX_DEFAULT,
        help="Maximum current-limit ratio (0x0022) allowed by this tool (default: 3.5)",
    )
    parser.add_argument(
        "--absolute-current-scale",
        type=float,
        default=1024.0,
        help="Absolute-current encoding scale for 0x001B (raw = A * scale, default: 1024)",
    )
    parser.add_argument(
        "--soft-reset-before-test",
        action="store_true",
        help="Apply safe soft-reset/default sequence before starting the output test",
    )
    parser.add_argument(
        "--control-interval-ms",
        type=float,
        default=500.0,
        help="Setpoint refresh period while test is running (default: 500 ms)",
    )
    parser.add_argument(
        "--telemetry-interval-s",
        type=float,
        default=2.0,
        help="Telemetry print interval during test (default: 2 s)",
    )
    parser.add_argument(
        "--telemetry-response-ms",
        type=float,
        default=250.0,
        help="Telemetry read response window (default: 250 ms)",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.addr_start < 0 or args.addr_start > 63 or args.addr_end < 0 or args.addr_end > 63:
        log("[ERROR] Address range must be within 0..63")
        return 2
    if args.addr_end < args.addr_start:
        log("[ERROR] addr-end must be >= addr-start")
        return 2
    if args.default_rated_current_a <= 0.0:
        log("[ERROR] default-rated-current-a must be > 0")
        return 2
    if args.ratio_max <= 0.0:
        log("[ERROR] ratio-max must be > 0")
        return 2
    if args.absolute_current_scale <= 0.0:
        log("[ERROR] absolute-current-scale must be > 0")
        return 2
    if args.control_interval_ms <= 0.0:
        log("[ERROR] control-interval-ms must be > 0")
        return 2
    if args.telemetry_interval_s <= 0.0:
        log("[ERROR] telemetry-interval-s must be > 0")
        return 2
    if args.telemetry_response_ms <= 0.0:
        log("[ERROR] telemetry-response-ms must be > 0")
        return 2

    try:
        groups = parse_group_spec(args.scan_groups)
    except ValueError as exc:
        log(f"[ERROR] {exc}")
        return 2

    warn_if_dc_ocpp_running()

    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.iface,))
    except OSError as exc:
        log(f"[ERROR] Failed to open SocketCAN on {args.iface}: {exc}")
        return 1

    try:
        log(
            f"Scanning MXR modules on {args.iface} "
            f"(groups={groups}, addrs={args.addr_start}..{args.addr_end}) ..."
        )
        modules = scan_modules(
            sock,
            groups=groups,
            addr_start=args.addr_start,
            addr_end=args.addr_end,
            send_gap_s=args.scan_send_gap_ms / 1000.0,
            listen_s=args.scan_listen_ms / 1000.0,
        )
        if not modules:
            log("No MXR modules discovered.")
            return 3

        for mod in modules:
            refresh_module_details(sock, mod)
        print_discovered_modules(modules)

        idx = prompt_select_module(len(modules) - 1)
        module = modules[idx]
        refresh_module_details(sock, module)
        log(f"\nSelected: {format_module_summary(module)}")

        if args.soft_reset_before_test:
            desired_mode = module.input_mode if module.input_mode in {1, 2, 3} else 3
            log("\nApplying soft reset/default sequence before test...")
            soft_reset_defaults(sock, module, input_mode=desired_mode)
            log(f"Post-reset: {format_module_summary(module)}")

        rated_default = module.rated_current_a if module.rated_current_a is not None else args.default_rated_current_a
        rated_current_a = prompt_float(
            "Enter rated current for ratio scaling (A)",
            default=round(rated_default, 3),
            min_value=0.1,
        )
        current_control_mode = prompt_choice(
            "Choose current control mode",
            choices=["ratio", "absolute"],
            default=args.current_control,
        )
        voltage_v = prompt_float("Enter target voltage (V)", min_value=0.0)
        current_a = prompt_float("Enter target current (A)", min_value=0.0)
        duration_s = prompt_float("Enter test duration (s)", min_value=0.5)

        run_constant_output_test(
            sock=sock,
            module=module,
            voltage_v=voltage_v,
            current_a=current_a,
            duration_s=duration_s,
            rated_current_a=rated_current_a,
            current_control_mode=current_control_mode,
            ratio_max=args.ratio_max,
            abs_current_scale=args.absolute_current_scale,
            control_interval_s=args.control_interval_ms / 1000.0,
            telemetry_interval_s=args.telemetry_interval_s,
            telemetry_response_window_s=args.telemetry_response_ms / 1000.0,
        )
        log("\nTest completed.")
        return 0
    except KeyboardInterrupt:
        log("\nInterrupted by user.")
        return 130
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
