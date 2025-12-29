# Waveshare USB-CAN-B setup (ControlCAN bridge)

DC-OCPP expects a SocketCAN interface. The Waveshare USB-CAN-B exposes a vendor
ControlCAN API instead, so this repo includes a small bridge that maps ControlCAN
frames to a SocketCAN interface (usually a virtual `can0`).

## Prereqs

- `can-utils` (for `candump`/`cansend`)
- `g++` (to build the bridge)

```bash
sudo apt-get install can-utils g++
```

## Install the Waveshare ControlCAN library

```bash
./scripts/waveshare_usbcan_setup.sh
```

This downloads the vendor `libcontrolcan.so` and `controlcan.h` into:
`third_party/waveshare-usb-can/`.

## Build the bridge

```bash
./scripts/build_usbcan_bridge.sh
```

## Create a SocketCAN interface

If you do not have a kernel SocketCAN driver for the adapter, use a virtual CAN
interface for the bridge:

```bash
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set can0 up
```

If your system exposes a real SocketCAN interface already (e.g. `can0`), skip
the `vcan` commands and use that interface instead.

## Run the bridge

```bash
./tools/waveshare_usbcan/usbcan_bridge --iface can0 --device 0 --channel 0 --bitrate 125000 --verbose
```

Notes:
- The PLC contract uses **125 kbps** and **29-bit extended IDs**.
- Use `--channel 1` if your PLC is wired to CAN2 on the adapter.
- If you need custom ControlCAN timings, pass `--timing0`/`--timing1`.

## Loopback test

```bash
./scripts/can_loop_test.sh can0
```

This validates the SocketCAN path. For a physical bus check, connect a second
CAN node (or a hardware loopback setup) and confirm frames in `candump`.

## DC-OCPP config reminders

Ensure `configs/charger.json` (or your custom config) has:

- `chargePoint.usePLC: true`
- `chargePoint.canInterface: "can0"` (or the interface you used)
- Unique `connectors[].plcId` values for each PLC node
