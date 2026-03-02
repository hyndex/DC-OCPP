#!/usr/bin/env python3
"""Standalone Tonhe module CAN bus ramp test.

Discovers Tonhe modules on SocketCAN, then for each module:
1) start command
2) voltage ramp 200V -> 1000V in 50V steps every 2s
3) voltage ramp down to 0V in 50V steps every 2s
4) stop command

Protocol fields are aligned with the in-tree Tonhe driver in
src/power_module_controller.cpp.
"""

from __future__ import annotations

import argparse
import errno
import select
import socket
import struct
import sys
import time
from typing import Dict, Iterable, List, Optional, Tuple

CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

TONHE_CMD_STOP = 0x55
TONHE_CMD_START = 0xAA

TONHE_PGN_STATE = 0x000100
TONHE_PGN_CONFIRM = 0x000200
TONHE_PGN_STARTSTOP = 0x000600
TONHE_PGN_AC_INFO = 0x000B00
TONHE_PGN_EXT_STATUS = 0x009100
TONHE_PGN_INPUT_MODE = 0x00AA00

CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")
CAN_MTU = CAN_FRAME_STRUCT.size


def log(msg: str) -> None:
    print(msg, flush=True)


def parse_int_auto(value: str) -> int:
    return int(value, 0)


def build_tonhe_id(priority: int, pgn: int, dest: int, src: int) -> int:
    pf = (pgn >> 8) & 0xFF
    can_id = ((priority & 0x7) << 26) | (pf << 16) | ((dest & 0xFF) << 8) | (src & 0xFF)
    return can_id | CAN_EFF_FLAG


def parse_j1939(can_id: int) -> Tuple[int, int, int]:
    raw = can_id & CAN_EFF_MASK
    pf = (raw >> 16) & 0xFF
    ps = (raw >> 8) & 0xFF
    src = raw & 0xFF
    pgn = (pf << 8) if pf < 0xF0 else ((pf << 8) | ps)
    dest = ps
    return pgn, dest, src


def u16_le(buf: bytes, idx: int) -> int:
    return int(buf[idx]) | (int(buf[idx + 1]) << 8)


def decode_state_frame(can_id: int, data: bytes) -> Optional[Dict[str, float]]:
    if (can_id & CAN_EFF_FLAG) == 0:
        return None
    pgn, dest, src = parse_j1939(can_id)
    if pgn != TONHE_PGN_STATE:
        return None
    if len(data) < 8:
        return None

    state = int(data[0])
    voltage_v = u16_le(data, 1) * 0.1
    current_a = u16_le(data, 3) * 0.01
    fault_bits = u16_le(data, 5)
    pfc_fault = int(data[7])

    return {
        "src": float(src),
        "dest": float(dest),
        "pgn": float(pgn),
        "state": float(state),
        "voltage_v": voltage_v,
        "current_a": current_a,
        "fault_bits": float(fault_bits),
        "pfc_fault": float(pfc_fault),
    }


def decode_any_tonhe_frame(can_id: int, data: bytes) -> Optional[Dict[str, float]]:
    if (can_id & CAN_EFF_FLAG) == 0:
        return None
    pgn, dest, src = parse_j1939(can_id)
    if pgn not in {TONHE_PGN_STATE, TONHE_PGN_CONFIRM, TONHE_PGN_AC_INFO, TONHE_PGN_EXT_STATUS}:
        return None
    out = {
        "src": float(src),
        "dest": float(dest),
        "pgn": float(pgn),
    }
    if pgn == TONHE_PGN_STATE and len(data) >= 8:
        state = decode_state_frame(can_id, data)
        if state:
            out.update(state)
    return out


def send_can(sock: socket.socket, can_id: int, payload: bytes, retries: int = 5, retry_delay_s: float = 0.05) -> None:
    if len(payload) != 8:
        raise ValueError("payload must be 8 bytes")
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


def send_input_mode(sock: socket.socket, source_addr: int, mode: int) -> None:
    # mode: 0=DC, 1=AC (aligned with Tonhe driver)
    can_id = build_tonhe_id(priority=6, pgn=TONHE_PGN_INPUT_MODE, dest=0xFF, src=source_addr)
    payload = bytes([mode & 0xFF, 0, 0, 0, 0, 0, 0, 0])
    send_can(sock, can_id, payload)


def send_startstop(
    sock: socket.socket, source_addr: int, module_addr: int, enable: bool, voltage_v: float, current_a: float
) -> None:
    can_id = build_tonhe_id(priority=2, pgn=TONHE_PGN_STARTSTOP, dest=module_addr, src=source_addr)
    v_raw = max(0, min(int(round(voltage_v * 10.0)), 0xFFFF))
    i_raw = max(0, min(int(round(current_a * 100.0)), 0xFFFF))
    payload = bytes(
        [
            TONHE_CMD_START if enable else TONHE_CMD_STOP,
            0x00,
            v_raw & 0xFF,
            (v_raw >> 8) & 0xFF,
            i_raw & 0xFF,
            (i_raw >> 8) & 0xFF,
            0x00,
            0x00,
        ]
    )
    send_can(sock, can_id, payload)


def recv_one(sock: socket.socket, timeout_s: float) -> Optional[Tuple[int, bytes]]:
    ready, _, _ = select.select([sock], [], [], timeout_s)
    if not ready:
        return None
    frame = sock.recv(CAN_MTU)
    if len(frame) < CAN_MTU:
        return None
    can_id, can_dlc, data = CAN_FRAME_STRUCT.unpack(frame[:CAN_MTU])
    return can_id, data[:can_dlc]


def discover_modules(sock: socket.socket, seconds: float) -> Dict[int, Dict[str, float]]:
    found: Dict[int, Dict[str, float]] = {}
    end_t = time.monotonic() + seconds
    while time.monotonic() < end_t:
        rem = max(0.0, end_t - time.monotonic())
        pkt = recv_one(sock, rem)
        if pkt is None:
            continue
        can_id, data = pkt
        st = decode_state_frame(can_id, data)
        if not st:
            continue
        src = int(st["src"])
        found[src] = st
    return found


def active_scan_modules(
    sock: socket.socket,
    source_addr: int,
    start_addr: int,
    end_addr: int,
    wait_ms: int,
) -> Dict[int, Dict[str, float]]:
    found: Dict[int, Dict[str, float]] = {}
    probe_wait_s = max(0.005, wait_ms / 1000.0)

    for module_addr in range(start_addr, end_addr + 1):
        # Probe by sending a STOP command. Present modules normally emit
        # state/confirm/extended frames that reveal SRC address.
        send_startstop(sock, source_addr, module_addr, False, 0.0, 0.0)
        deadline = time.monotonic() + probe_wait_s

        while time.monotonic() < deadline:
            rem = max(0.0, deadline - time.monotonic())
            pkt = recv_one(sock, rem)
            if pkt is None:
                continue
            can_id, data = pkt
            info = decode_any_tonhe_frame(can_id, data)
            if not info:
                continue
            src = int(info["src"])
            dest = int(info["dest"])
            if src != module_addr:
                continue
            if dest not in {source_addr & 0xFF, 0xFF}:
                continue
            found[src] = info
            break
    return found


def wait_module_state(sock: socket.socket, module_addr: int, timeout_s: float) -> Optional[Dict[str, float]]:
    end_t = time.monotonic() + timeout_s
    last: Optional[Dict[str, float]] = None
    while time.monotonic() < end_t:
        rem = max(0.0, end_t - time.monotonic())
        pkt = recv_one(sock, rem)
        if pkt is None:
            continue
        can_id, data = pkt
        st = decode_state_frame(can_id, data)
        if not st:
            continue
        if int(st["src"]) != module_addr:
            continue
        last = st
    return last


def stream_module_telemetry(
    sock: socket.socket,
    module_addr: int,
    duration_s: float,
    min_emit_interval_s: float,
) -> Optional[Dict[str, float]]:
    end_t = time.monotonic() + max(0.0, duration_s)
    last_emit_t = 0.0
    last_state: Optional[Dict[str, float]] = None

    while time.monotonic() < end_t:
        rem = max(0.0, end_t - time.monotonic())
        pkt = recv_one(sock, min(0.1, rem))
        if pkt is None:
            continue
        can_id, data = pkt
        st = decode_state_frame(can_id, data)
        if not st:
            continue
        if int(st["src"]) != module_addr:
            continue
        last_state = st
        now = time.monotonic()
        if now - last_emit_t >= max(0.0, min_emit_interval_s):
            log(f"[TEL] module=0x{module_addr:02X} {format_state(st)}")
            last_emit_t = now
    return last_state


def format_state(st: Optional[Dict[str, float]]) -> str:
    if not st:
        return "no state frame"
    return (
        f"state=0x{int(st['state']):02X} "
        f"V={st['voltage_v']:.1f} "
        f"I={st['current_a']:.2f} "
        f"fault=0x{int(st['fault_bits']):04X} "
        f"pfc=0x{int(st['pfc_fault']):02X}"
    )


def parse_module_list(raw: Optional[str]) -> List[int]:
    if not raw:
        return []
    out: List[int] = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        out.append(parse_int_auto(token))
    return sorted(set(out))


def voltage_up_down(start_v: int, stop_v: int, step_v: int) -> Iterable[int]:
    if step_v <= 0:
        raise ValueError("step_v must be > 0")
    v = start_v
    while v <= stop_v:
        yield v
        v += step_v
    v = stop_v - step_v
    while v >= 0:
        yield v
        v -= step_v


def run_test(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.iface,))

    mode = 1 if args.input_mode.lower() == "ac" else 0
    if args.send_input_mode:
        send_input_mode(sock, args.source_addr, mode)
        log(f"[INFO] Sent input-mode command: mode={args.input_mode.upper()} ({mode}) src=0x{args.source_addr:02X}")

    modules = parse_module_list(args.modules)
    explicit_modules = len(modules) > 0
    discovered = discover_modules(sock, args.discover_seconds)

    if discovered:
        log("[INFO] Passive Tonhe discovery:")
        for addr in sorted(discovered):
            st = discovered[addr]
            log(
                f"  - module=0x{addr:02X} dest=0x{int(st['dest']):02X} "
                f"{format_state(st)}"
            )
    else:
        log("[WARN] No Tonhe state frames seen during passive discovery window.")

    if args.active_scan and not explicit_modules:
        scanned = active_scan_modules(
            sock=sock,
            source_addr=args.source_addr,
            start_addr=args.scan_start,
            end_addr=args.scan_end,
            wait_ms=args.scan_wait_ms,
        )
        if scanned:
            log("[INFO] Active address scan found:")
            for addr in sorted(scanned):
                info = scanned[addr]
                pgn = int(info["pgn"])
                state_text = format_state(info if pgn == TONHE_PGN_STATE else None)
                log(
                    f"  - module=0x{addr:02X} dest=0x{int(info['dest']):02X} "
                    f"pgn=0x{pgn:06X} {state_text}"
                )
                if addr not in discovered:
                    discovered[addr] = info
        else:
            log(
                f"[WARN] Active scan did not find modules in range "
                f"0x{args.scan_start:02X}-0x{args.scan_end:02X}."
            )

    if not explicit_modules:
        for addr in discovered.keys():
            if addr not in modules:
                modules.append(addr)
    modules = sorted(set(modules))

    if not modules:
        log("[ERROR] No module addresses available. Pass --modules or ensure modules are broadcasting state.")
        return 2

    log(f"[INFO] Test module list: {', '.join(f'0x{m:02X}' for m in modules)}")
    log(
        f"[INFO] Ramp profile: {args.start_v}V -> {args.max_v}V -> 0V, "
        f"step={args.step_v}V every {args.interval_s:.1f}s, current={args.current_a:.2f}A"
    )

    all_ok = True
    for module in modules:
        log(f"\n[INFO] ===== Module 0x{module:02X} test start =====")
        module_ok = True
        module_fault_bits = 0
        no_state_steps = 0
        for v in voltage_up_down(args.start_v, args.max_v, args.step_v):
            send_startstop(sock, args.source_addr, module, True, float(v), args.current_a)
            st = wait_module_state(sock, module, args.state_timeout_s)
            log(f"[CMD] module=0x{module:02X} ON Vset={v:4d} Iset={args.current_a:5.2f} | {format_state(st)}")
            if args.telemetry_live:
                tail = stream_module_telemetry(
                    sock,
                    module_addr=module,
                    duration_s=args.interval_s,
                    min_emit_interval_s=args.telemetry_emit_interval_s,
                )
                if tail is not None:
                    st = tail
            else:
                time.sleep(args.interval_s)

            if st is None:
                no_state_steps += 1
                if no_state_steps >= args.max_no_state_steps:
                    log(
                        f"[ERR] module=0x{module:02X} telemetry missing for "
                        f"{no_state_steps} consecutive steps; aborting module ramp"
                    )
                    module_ok = False
                    break
            else:
                no_state_steps = 0
                fault_bits = int(st.get("fault_bits", 0.0))
                if fault_bits != 0:
                    module_fault_bits |= fault_bits
                    log(f"[ERR] module=0x{module:02X} fault_bits=0x{fault_bits:04X}")
                    if args.stop_on_fault:
                        module_ok = False
                        break

        send_startstop(sock, args.source_addr, module, False, 0.0, 0.0)
        time.sleep(0.2)
        send_startstop(sock, args.source_addr, module, False, 0.0, 0.0)
        st = wait_module_state(sock, module, args.state_timeout_s)
        log(f"[CMD] module=0x{module:02X} OFF | {format_state(st)}")
        if st is not None and int(st.get("fault_bits", 0.0)) != 0:
            module_fault_bits |= int(st["fault_bits"])
            module_ok = False

        status = "PASS" if module_ok else "FAIL"
        if module_fault_bits != 0:
            log(f"[INFO] module=0x{module:02X} result={status} fault_mask=0x{module_fault_bits:04X}")
        else:
            log(f"[INFO] module=0x{module:02X} result={status}")
        log(f"[INFO] ===== Module 0x{module:02X} test end =====")
        all_ok = all_ok and module_ok

    return 0 if all_ok else 3


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Tonhe standalone CAN module ramp test (discover/start/ramp down/stop)."
    )
    p.add_argument("--iface", default="can0", help="SocketCAN interface (default: can0)")
    p.add_argument(
        "--source-addr",
        type=parse_int_auto,
        default=0xA0,
        help="Monitor/controller source address in CAN ID (default: 0xA0)",
    )
    p.add_argument(
        "--modules",
        default="",
        help="Comma-separated module addresses (hex or dec), e.g. 0x01,0x02. "
        "If omitted, passive discovery is used.",
    )
    p.add_argument(
        "--discover-seconds",
        type=float,
        default=6.0,
        help="Passive discovery window in seconds (default: 6)",
    )
    p.add_argument("--active-scan", action="store_true", default=True, help="Actively scan module addresses")
    p.add_argument("--no-active-scan", action="store_false", dest="active_scan", help="Disable active scan")
    p.add_argument(
        "--scan-start",
        type=parse_int_auto,
        default=0x01,
        help="Active scan start module address (default: 0x01)",
    )
    p.add_argument(
        "--scan-end",
        type=parse_int_auto,
        default=0x7F,
        help="Active scan end module address (default: 0x7F)",
    )
    p.add_argument(
        "--scan-wait-ms",
        type=int,
        default=40,
        help="Listen time per scanned address (default: 40 ms)",
    )
    p.add_argument("--send-input-mode", action="store_true", default=True, help="Send Tonhe input mode command")
    p.add_argument(
        "--no-send-input-mode", action="store_false", dest="send_input_mode", help="Do not send input mode command"
    )
    p.add_argument(
        "--input-mode",
        choices=["ac", "dc"],
        default="ac",
        help="Input mode payload for Tonhe command (default: ac)",
    )
    p.add_argument("--start-v", type=int, default=200, help="Ramp start voltage (default: 200)")
    p.add_argument("--max-v", type=int, default=1000, help="Ramp max voltage (default: 1000)")
    p.add_argument("--step-v", type=int, default=50, help="Ramp step voltage (default: 50)")
    p.add_argument("--interval-s", type=float, default=2.0, help="Delay between steps in seconds (default: 2)")
    p.add_argument("--telemetry-live", action="store_true", default=True, help="Print live module telemetry during ramp")
    p.add_argument(
        "--no-telemetry-live", action="store_false", dest="telemetry_live", help="Disable live telemetry prints"
    )
    p.add_argument(
        "--telemetry-emit-interval-s",
        type=float,
        default=0.5,
        help="Minimum spacing for telemetry print lines while streaming (default: 0.5)",
    )
    p.add_argument(
        "--current-a",
        type=float,
        default=2.0,
        help="Current setpoint to send with voltage setpoint (default: 2.0)",
    )
    p.add_argument(
        "--state-timeout-s",
        type=float,
        default=0.8,
        help="Wait time for state feedback after each command (default: 0.8)",
    )
    p.add_argument(
        "--max-no-state-steps",
        type=int,
        default=4,
        help="Fail module if this many consecutive ramp steps have no state frame (default: 4)",
    )
    p.add_argument(
        "--stop-on-fault",
        action="store_true",
        default=True,
        help="Stop module ramp immediately when fault bits appear",
    )
    p.add_argument(
        "--no-stop-on-fault",
        action="store_false",
        dest="stop_on_fault",
        help="Continue ramp even if fault bits appear",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.start_v < 0 or args.max_v < 0:
        print("[ERROR] start/max voltage must be >= 0")
        return 2
    if args.max_v < args.start_v:
        print("[ERROR] max voltage must be >= start voltage")
        return 2
    if args.step_v <= 0:
        print("[ERROR] step voltage must be > 0")
        return 2
    if args.current_a < 0:
        print("[ERROR] current must be >= 0")
        return 2
    if args.scan_start < 0 or args.scan_start > 0xFF or args.scan_end < 0 or args.scan_end > 0xFF:
        print("[ERROR] scan range must be between 0x00 and 0xFF")
        return 2
    if args.scan_end < args.scan_start:
        print("[ERROR] scan-end must be >= scan-start")
        return 2
    if args.scan_wait_ms <= 0:
        print("[ERROR] scan-wait-ms must be > 0")
        return 2

    try:
        return run_test(args)
    except KeyboardInterrupt:
        log("\n[INFO] Interrupted by user")
        return 130
    except OSError as exc:
        log(f"[ERROR] SocketCAN failure: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
