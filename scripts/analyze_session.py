#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple


CANDUMP_RE = re.compile(
    r"^\s*\((?P<t>[0-9.]+)\)\s+can0\s+(?P<dir>TX|RX)\s+-\s+-\s+(?P<id>[0-9A-Fa-f]+)\s+\[8\]\s+(?P<b>(?:[0-9A-Fa-f]{2}\s+){7}[0-9A-Fa-f]{2})\s*$"
)


def parse_u32_le(bs: List[int], offset: int) -> int:
    return bs[offset] | (bs[offset + 1] << 8) | (bs[offset + 2] << 16) | (bs[offset + 3] << 24)


def parse_u16_le(bs: List[int], offset: int) -> int:
    return bs[offset] | (bs[offset + 1] << 8)


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
        if seg_idx in self.received:
            return
        self.received[seg_idx] = payload

    def complete(self) -> bool:
        return self.uid_len > 0 and self.seg_cnt > 0 and len(self.received) >= self.seg_cnt

    def uid(self) -> bytes:
        out = bytearray()
        for idx in range(self.seg_cnt):
            out.extend(self.received.get(idx, b""))
        return bytes(out[: self.uid_len])


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
    print(f"CAN capture: {len(frames)} frames, t={t0:.3f}s..{t1:.3f}s (Δ={(t1 - t0):.3f}s)")

    # IDs (PLC_ID=0 assumed; adjust if you use other PLC IDs)
    ID_CHARGEINFO = 0x0100
    ID_RFID = 0x0180
    ID_CONFIG_ACK = 0x01A0
    ID_EVDC_TARGETS = 0x0210
    ID_CHARGING_SESSION = 0x0410

    ID_EVSE_MAX = 0x0300
    ID_EVSE_REG = 0x0310
    ID_RELAY_CTRL = 0x0340
    ID_CONFIG_CMD = 0x0380

    # Track last-known key state
    last_cp: Optional[Tuple[str, int]] = None
    last_hlc_stage: Optional[int] = None
    last_auth_pending: Optional[bool] = None

    # ChargeInfo: stage + flags
    stages_seen = set()
    stage_changes: List[Tuple[float, int, int]] = []
    auth_flag_changes: List[Tuple[float, bool, bool]] = []

    # EVDC targets: detect first/last non-zero
    first_targets = None
    last_targets = None
    max_tgt_v = 0
    max_tgt_i = 0

    # RFID events: reconstruct UIDs
    rfid_by_event: Dict[int, RfidAssembly] = {}
    rfid_uids: List[Tuple[float, int, str]] = []

    # ConfigCmd/Ack: auth state / pending / lock
    config_cmds: List[Tuple[float, int, int, int]] = []
    config_acks: List[Tuple[float, int, int, int]] = []

    # EVSE present/limits telemetry
    evse_present_samples: List[Tuple[float, float, bool, bool, int]] = []

    for f in frames:
        bs = f.data

        if f.direction == "RX" and f.can_id == ID_CHARGING_SESSION:
            cp_state = chr(bs[4]) if 32 <= bs[4] <= 126 else "?"
            duty = bs[5]
            hlc_stage = bs[6]
            auth_pending = (bs[7] != 0)
            if last_cp != (cp_state, duty):
                print(f"CP: t={f.t:.3f}s state={cp_state} duty={duty}%")
                last_cp = (cp_state, duty)
            if last_hlc_stage != hlc_stage:
                print(f"HLC: t={f.t:.3f}s stage={hlc_stage}")
                last_hlc_stage = hlc_stage
            if last_auth_pending is None or last_auth_pending != auth_pending:
                print(f"HLC: t={f.t:.3f}s auth_pending={auth_pending}")
                last_auth_pending = auth_pending

        if f.direction == "RX" and f.can_id == ID_CHARGEINFO:
            stage = bs[0]
            flags = bs[1]
            auth_granted = (flags & 0x08) != 0
            auth_pending = (flags & 0x10) != 0
            stages_seen.add(stage)
            if last_hlc_stage is None:
                last_hlc_stage = stage
            elif stage != last_hlc_stage:
                stage_changes.append((f.t, last_hlc_stage, stage))
                last_hlc_stage = stage
            if last_auth_pending is None:
                last_auth_pending = auth_pending
            else:
                # only track ChargeInfo auth transitions if ChargingSession isn't present
                pass
            auth_flag_changes.append((f.t, auth_granted, auth_pending))

        if f.direction == "RX" and f.can_id == ID_EVDC_TARGETS:
            tgt_v = parse_u16_le(bs, 0)
            tgt_i = parse_u16_le(bs, 2)
            pres_v = parse_u16_le(bs, 4)
            pres_i = parse_u16_le(bs, 6)
            if tgt_v or tgt_i:
                if first_targets is None:
                    first_targets = (f.t, tgt_v / 10.0, tgt_i / 10.0, pres_v / 10.0, pres_i / 10.0)
                last_targets = (f.t, tgt_v / 10.0, tgt_i / 10.0, pres_v / 10.0, pres_i / 10.0)
                max_tgt_v = max(max_tgt_v, tgt_v)
                max_tgt_i = max(max_tgt_i, tgt_i)

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
                uid_hex = asm.uid().hex().upper()
                rfid_uids.append((f.t, event_id, uid_hex))
                # prevent duplicate emission
                del rfid_by_event[event_id]

        if f.direction == "TX" and f.can_id == ID_CONFIG_CMD:
            param = bs[0]
            op = bs[1]
            val = parse_u32_le(bs, 2)
            config_cmds.append((f.t, param, op, val))

        if f.direction == "RX" and f.can_id == ID_CONFIG_ACK:
            param = bs[0]
            status = bs[1]
            val = parse_u32_le(bs, 2)
            config_acks.append((f.t, param, status, val))

        if f.direction == "TX" and f.can_id == ID_EVSE_REG:
            v = parse_u16_le(bs, 0) / 10.0
            flags = bs[6]
            output_enabled = (flags & 0x01) != 0
            regulating = (flags & 0x02) != 0
            fault_bits = (flags >> 2) & 0x3F
            evse_present_samples.append((f.t, v, output_enabled, regulating, fault_bits))

    print()
    if rfid_uids:
        print("RFID:")
        for t, eid, uid in rfid_uids[-5:]:
            print(f"  t={t:.3f}s event={eid} uid={uid}")
    else:
        print("RFID: no UID frames seen")

    # Auth config summary
    auth_cmds = [c for c in config_cmds if c[1] in (20, 21)]
    if auth_cmds:
        print("\nConfigCmd (auth):")
        for t, param, _op, val in auth_cmds[-10:]:
            name = "AUTH_STATE" if param == 20 else "AUTH_PENDING"
            print(f"  t={t:.3f}s {name}={val}")
    else:
        print("\nConfigCmd (auth): none")

    auth_acks = [a for a in config_acks if a[1] in (20, 21)]
    if auth_acks:
        print("\nConfigAck (auth):")
        for t, param, status, val in auth_acks[-10:]:
            name = "AUTH_STATE" if param == 20 else "AUTH_PENDING"
            print(f"  t={t:.3f}s {name} status={status} value={val}")

    if first_targets:
        print("\nEVDC_Targets (non-zero):")
        print(f"  first: t={first_targets[0]:.3f}s tgt=({first_targets[1]}V,{first_targets[2]}A) pres=({first_targets[3]}V,{first_targets[4]}A)")
        if last_targets and last_targets != first_targets:
            print(f"  last : t={last_targets[0]:.3f}s tgt=({last_targets[1]}V,{last_targets[2]}A) pres=({last_targets[3]}V,{last_targets[4]}A)")
        print(f"  max  : {max_tgt_v/10.0}V {max_tgt_i/10.0}A")
    else:
        print("\nEVDC_Targets: no non-zero targets seen")

    if evse_present_samples:
        last_pres = evse_present_samples[-1]
        reg_true = sum(1 for _t, _v, _oe, reg, _fb in evse_present_samples if reg)
        oe_true = sum(1 for _t, _v, oe, _reg, _fb in evse_present_samples if oe)
        fault_nonzero = sum(1 for _t, _v, _oe, _reg, fb in evse_present_samples if fb != 0)
        print("\nEVSE_DC_Reg_Limits (TX):")
        print(
            f"  last : t={last_pres[0]:.3f}s V={last_pres[1]} output_enabled={last_pres[2]} regulating={last_pres[3]} fault_bits=0x{last_pres[4]:02X}"
        )
        print(f"  stats: output_enabled_true={oe_true} regulating_true={reg_true} fault_nonzero={fault_nonzero}")

    if dc_lines:
        # Pull the most relevant application-level markers from dc_ocpp tail
        keys = ("Auth state connector", "Session ", "transaction", "RFID", "plugged_in", "Fault", "Modules", "Module")
        interesting = [ln for ln in dc_lines if any(k in ln for k in keys)]
        if interesting:
            print("\ndc_ocpp (tail):")
            for ln in interesting[-20:]:
                print(f"  {ln}")

    print("\nHeuristics:")
    auth_granted_cmd = any(param == 20 and val == 1 for _t, param, _op, val in config_cmds)
    if not rfid_uids:
        print("  - No RFID seen on CAN: card tap may not be reaching PLC, or captured the wrong bus/interface.")
    if rfid_uids and not auth_granted_cmd:
        print("  - RFID seen but no AUTH_STATE=1 ConfigCmd sent: controller authorization flow is not propagating to PLC.")
    if auth_granted_cmd and not first_targets:
        print("  - AUTH_STATE=1 sent but EVDC_Targets stayed at 0: ISO15118 session did not reach precharge/current-demand (check HLC stage, SLAC/TCP).")
    if not auth_granted_cmd and not first_targets:
        print("  - No EV targets: EV will not charge until ISO15118 progresses; start by verifying AUTH flow + HLC stage progression.")

    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Summarize a dc-ocpp CAN session capture (candump -tz -x).")
    ap.add_argument("--candump", required=True, type=Path, help="Path to candump log (from capture_session.sh)")
    ap.add_argument("--dc-tail", type=Path, default=None, help="Optional dc_ocpp tail log (from capture_session.sh)")
    args = ap.parse_args()

    frames = parse_candump(args.candump)
    dc_lines: List[str] = []
    if args.dc_tail:
        dc_lines = parse_dc_tail(args.dc_tail)
    return summarize(frames, dc_lines)


if __name__ == "__main__":
    raise SystemExit(main())

