#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


CANDUMP_RE = re.compile(
    r"^\s*\((?P<t>[0-9.]+)\)\s+can0\s+(?P<dir>TX|RX)\s+-\s+-\s+(?P<id>[0-9A-Fa-f]+)\s+\[8\]\s+(?P<b>(?:[0-9A-Fa-f]{2}\s+){7}[0-9A-Fa-f]{2})\s*$"
)


def parse_u32_le(bs: List[int], offset: int) -> int:
    return bs[offset] | (bs[offset + 1] << 8) | (bs[offset + 2] << 16) | (bs[offset + 3] << 24)


@dataclasses.dataclass
class Frame:
    t: float
    direction: str
    can_id: int
    data: List[int]


@dataclasses.dataclass
class RfidAssembly:
    uid_len: int
    seg_cnt: int
    received: Dict[int, bytes] = dataclasses.field(default_factory=dict)

    def add(self, seg_idx: int, payload: bytes) -> None:
        if seg_idx not in self.received:
            self.received[seg_idx] = payload

    def complete(self) -> bool:
        return self.uid_len > 0 and self.seg_cnt > 0 and len(self.received) >= self.seg_cnt

    def uid(self) -> bytes:
        out = bytearray()
        for idx in range(self.seg_cnt):
            out.extend(self.received.get(idx, b""))
        return bytes(out[: self.uid_len])


@dataclasses.dataclass
class PlcTlmV3:
    cp_state: str
    hlc_stage: int
    fault_reason_l4: int
    relay_state_mask: int
    relay_fault_mask: int
    safety_ok: bool
    estop: bool
    earth_fault: bool
    comm_fault: bool
    lock_engaged: bool
    cable_checked: bool
    precharge_active: bool
    charge_complete: bool
    limits_rx_count_lsb: int
    ev_target_v: float
    ev_target_i: float


def decode_plc_tlm_v3(data: List[int]) -> PlcTlmV3:
    packed = 0
    for i in range(7):
        packed |= int(data[i]) << (8 * i)

    cp_enum = (packed >> 0) & 0x07
    cp_state = {
        0: "U",
        1: "A",
        2: "B",
        3: "C",
        4: "D",
        5: "E",
        6: "F",
    }.get(cp_enum, "U")

    ev_target_v_1v = (packed >> 35) & 0x03FF
    ev_target_i_0p5a = (packed >> 45) & 0x03FF

    return PlcTlmV3(
        cp_state=cp_state,
        hlc_stage=(packed >> 3) & 0x3F,
        fault_reason_l4=(packed >> 9) & 0x0F,
        relay_state_mask=(packed >> 13) & 0x07,
        relay_fault_mask=(packed >> 16) & 0x07,
        safety_ok=((packed >> 19) & 0x01) != 0,
        estop=((packed >> 20) & 0x01) != 0,
        earth_fault=((packed >> 21) & 0x01) != 0,
        comm_fault=((packed >> 22) & 0x01) != 0,
        lock_engaged=((packed >> 23) & 0x01) != 0,
        cable_checked=((packed >> 24) & 0x01) != 0,
        precharge_active=((packed >> 25) & 0x01) != 0,
        charge_complete=((packed >> 26) & 0x01) != 0,
        limits_rx_count_lsb=(packed >> 27) & 0xFF,
        ev_target_v=float(ev_target_v_1v),
        ev_target_i=float(ev_target_i_0p5a) * 0.5,
    )


def parse_candump(path: Path) -> List[Frame]:
    frames: List[Frame] = []
    for line in path.read_text(errors="ignore").splitlines():
        m = CANDUMP_RE.match(line)
        if not m:
            continue
        t = float(m.group("t"))
        direction = m.group("dir")
        can_id = int(m.group("id"), 16)
        data = [int(x, 16) for x in m.group("b").split()]
        frames.append(Frame(t=t, direction=direction, can_id=can_id, data=data))
    return frames


def parse_dc_tail(path: Path) -> List[str]:
    try:
        return path.read_text(errors="ignore").splitlines()
    except FileNotFoundError:
        return []


def summarize(frames: List[Frame], dc_lines: List[str]) -> int:
    if not frames:
        print("No CAN frames parsed (is this a candump -tz -x log?)")
        return 2

    t0, t1 = frames[0].t, frames[-1].t
    dt = max(t1 - t0, 1e-6)
    print(f"CAN capture: {len(frames)} frames, t={t0:.3f}s..{t1:.3f}s (Δ={t1 - t0:.3f}s)")

    # IDs (PLC_ID=0 assumed; adjust if needed)
    ID_EVSE_FAST = 0x0320
    ID_EVSE_SLOW = 0x0330
    ID_PLC_TLM_V3 = 0x0460
    ID_RFID = 0x0180
    ID_CONFIG_CMD = 0x0380
    ID_CONFIG_ACK = 0x01A0

    per_id: Dict[int, int] = {}
    for f in frames:
        per_id[f.can_id] = per_id.get(f.can_id, 0) + 1

    print("\nKey frame rates:")
    for can_id, name in (
        (ID_EVSE_FAST, "EVSE_FAST"),
        (ID_EVSE_SLOW, "EVSE_SLOW"),
        (ID_PLC_TLM_V3, "PLC_TLM_V3"),
        (ID_CONFIG_CMD, "ConfigCmd"),
        (ID_CONFIG_ACK, "ConfigAck"),
        (ID_RFID, "RFIDEvent"),
    ):
        count = per_id.get(can_id, 0)
        print(f"  {name:11s} id=0x{can_id:03X} count={count:5d} fps={count / dt:7.2f}")

    last_tlm: Optional[PlcTlmV3] = None
    first_nonzero_target: Optional[Tuple[float, float, float]] = None
    max_target_v = 0.0
    max_target_i = 0.0
    hlc_stage_changes: List[Tuple[float, int, int]] = []
    cp_changes: List[Tuple[float, str, str]] = []

    last_hlc_stage: Optional[int] = None
    last_cp_state: Optional[str] = None

    limits_auth_samples: List[Tuple[float, int, bool, bool, bool, bool]] = []

    rfid_by_event: Dict[int, RfidAssembly] = {}
    rfid_uids: List[Tuple[float, int, str]] = []

    for f in frames:
        bs = f.data

        if f.direction == "RX" and f.can_id == ID_PLC_TLM_V3:
            tlm = decode_plc_tlm_v3(bs)
            last_tlm = tlm

            if last_hlc_stage is None:
                last_hlc_stage = tlm.hlc_stage
            elif tlm.hlc_stage != last_hlc_stage:
                hlc_stage_changes.append((f.t, last_hlc_stage, tlm.hlc_stage))
                last_hlc_stage = tlm.hlc_stage

            if last_cp_state is None:
                last_cp_state = tlm.cp_state
            elif tlm.cp_state != last_cp_state:
                cp_changes.append((f.t, last_cp_state, tlm.cp_state))
                last_cp_state = tlm.cp_state

            if tlm.ev_target_v > 0 or tlm.ev_target_i > 0:
                if first_nonzero_target is None:
                    first_nonzero_target = (f.t, tlm.ev_target_v, tlm.ev_target_i)
                max_target_v = max(max_target_v, tlm.ev_target_v)
                max_target_i = max(max_target_i, tlm.ev_target_i)

        if f.direction == "TX" and f.can_id == ID_EVSE_SLOW:
            packed = 0
            for i in range(7):
                packed |= int(bs[i]) << (8 * i)
            auth_granted = ((packed >> 31) & 0x01) != 0
            auth_pending = ((packed >> 32) & 0x01) != 0
            hlc_enable = ((packed >> 33) & 0x01) != 0
            pnc_blocked = ((packed >> 34) & 0x01) != 0
            limits_auth_samples.append((f.t, (packed >> 36) & 0x0F, auth_granted, auth_pending, hlc_enable, pnc_blocked))

        if f.direction == "RX" and f.can_id == ID_RFID:
            uid_len = bs[0] & 0x0F
            event_type = (bs[0] >> 4) & 0x0F
            event_id = bs[1]
            seg_idx = bs[2] & 0x0F
            seg_cnt = (bs[2] >> 4) & 0x0F
            payload = bytes(bs[3:8])
            if event_type != 0 or uid_len == 0 or seg_cnt == 0:
                continue
            asm = rfid_by_event.get(event_id)
            if asm is None or asm.uid_len != uid_len or asm.seg_cnt != seg_cnt:
                asm = RfidAssembly(uid_len=uid_len, seg_cnt=seg_cnt)
                rfid_by_event[event_id] = asm
            asm.add(seg_idx=seg_idx, payload=payload)
            if asm.complete():
                rfid_uids.append((f.t, event_id, asm.uid().hex().upper()))
                del rfid_by_event[event_id]

    if cp_changes:
        print("\nCP transitions:")
        for t, a, b in cp_changes[-10:]:
            print(f"  t={t:.3f}s {a} -> {b}")

    if hlc_stage_changes:
        print("\nHLC stage transitions:")
        for t, a, b in hlc_stage_changes[-12:]:
            print(f"  t={t:.3f}s {a} -> {b}")

    if limits_auth_samples:
        print("\nEVSE_SLOW samples (last 8):")
        for t, proto_tag, ag, ap, he, pb in limits_auth_samples[-8:]:
            print(
                f"  t={t:.3f}s proto_tag={proto_tag} auth_granted={ag} auth_pending={ap} "
                f"hlc_enable={he} pnc_blocked={pb}"
            )

    if last_tlm:
        print("\nLast PLC_TLM_V3:")
        print(
            f"  cp={last_tlm.cp_state} hlc_stage={last_tlm.hlc_stage} safety_ok={last_tlm.safety_ok} "
            f"estop={last_tlm.estop} earth_fault={last_tlm.earth_fault} comm_fault={last_tlm.comm_fault}"
        )
        print(
            f"  relays=0b{last_tlm.relay_state_mask:03b} relay_faults=0b{last_tlm.relay_fault_mask:03b} "
            f"fault_l4={last_tlm.fault_reason_l4} limits_lsb={last_tlm.limits_rx_count_lsb}"
        )
        print(
            f"  targets: {last_tlm.ev_target_v:.1f}V / {last_tlm.ev_target_i:.1f}A "
            f"precharge={last_tlm.precharge_active} charge_complete={last_tlm.charge_complete}"
        )

    if first_nonzero_target:
        print("\nPLC_TLM_V3 target activity:")
        print(
            f"  first non-zero: t={first_nonzero_target[0]:.3f}s "
            f"V={first_nonzero_target[1]:.1f} I={first_nonzero_target[2]:.1f}"
        )
        print(f"  max seen: V={max_target_v:.1f} I={max_target_i:.1f}")
    else:
        print("\nPLC_TLM_V3 target activity: no non-zero EV targets seen")

    if rfid_uids:
        print("\nRFID events:")
        for t, eid, uid in rfid_uids[-8:]:
            print(f"  t={t:.3f}s event={eid} uid={uid}")

    if dc_lines:
        keys = ("Auth state connector", "Session ", "transaction", "RFID", "plugged_in", "Fault", "Modules", "Module")
        interesting = [ln for ln in dc_lines if any(k in ln for k in keys)]
        if interesting:
            print("\ndc_ocpp (tail):")
            for ln in interesting[-20:]:
                print(f"  {ln}")

    print("\nHeuristics:")
    if per_id.get(ID_PLC_TLM_V3, 0) == 0:
        print("  - No PLC_TLM_V3 received: PLC TX path or CAN routing likely broken.")
    if per_id.get(ID_EVSE_FAST, 0) == 0 or per_id.get(ID_EVSE_SLOW, 0) == 0:
        print("  - Missing EVSE_FAST/EVSE_SLOW frames: controller TX path or CAN filtering may be wrong.")
    if per_id.get(ID_RFID, 0) == 0:
        print("  - No RFID events seen in this capture.")
    if first_nonzero_target is None:
        print("  - EV targets stayed zero: HLC may not have reached current-demand/precharge state.")

    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Summarize a v3 PLC/controller CAN session capture (candump -tz -x).")
    ap.add_argument("--candump", required=True, type=Path, help="Path to candump log")
    ap.add_argument("--dc-tail", type=Path, default=None, help="Optional dc_ocpp tail log")
    args = ap.parse_args()

    frames = parse_candump(args.candump)
    dc_lines: List[str] = []
    if args.dc_tail:
        dc_lines = parse_dc_tail(args.dc_tail)
    return summarize(frames, dc_lines)


if __name__ == "__main__":
    raise SystemExit(main())
