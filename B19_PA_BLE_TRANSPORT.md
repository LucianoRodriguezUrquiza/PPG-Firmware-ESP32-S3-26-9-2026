# B19 PA-BLE binary transport v1

B19 is derived from the validated `B18_BLE.ino`. The existing Nordic UART
Service (NUS), `HELLO:2`, and the ASCII families `H/B/S/O/R/D` are unchanged.
B19 adds one optional Notify characteristic to the same service:

- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX command: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` (unchanged)
- TX B18 text: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` (unchanged)
- TX PA binary: `6E400004-B5A3-F393-E0A9-E50E24DCCA9E` (new, Notify)

An old B18 Android client can ignore UUID `...0004` and continue to negotiate
`HELLO:2` normally.

## Byte order and CRC

All integer fields are unsigned little-endian. Protocol version is `1`.

CRC32 is the standard reflected IEEE CRC-32:

- polynomial: `0xEDB88320`
- initial value: `0xFFFFFFFF`
- final XOR: `0xFFFFFFFF`
- coverage: exactly the 700 `uint32_t` IR samples serialized little-endian,
  in sample-index order (2800 bytes total)

Every application packet is contained in one GATT notification. The firmware
uses `min(MTU-3, 180)` as its maximum notification payload and adapts the DATA
sample count to the negotiated MTU.

## Common header

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | magic `0x50` (`P`) |
| 1 | 1 | magic `0x41` (`A`) |
| 2 | 1 | version = `1` |
| 3 | 1 | packet type: BEGIN=1, DATA=2, END=3 |
| 4 | 4 | `windowSeq` uint32 |

## BEGIN (type 1, 20 bytes)

| Offset | Size | Field |
|---:|---:|---|
| 0..7 | 8 | common header |
| 8 | 2 | `sampleCount` = 700 |
| 10 | 2 | `sampleRate` = 100 Hz |
| 12 | 4 | `spanMs` (accepted firmware window: 6500..7500 ms) |
| 16 | 4 | CRC32 of all 700 IR samples |

## DATA (type 2, 12 + 4*count bytes)

| Offset | Size | Field |
|---:|---:|---|
| 0..7 | 8 | common header |
| 8 | 2 | `startIndex` (0..699) |
| 10 | 2 | `count` |
| 12 | 4*count | consecutive raw IR samples (`uint32_t`) |

DATA packets must be contiguous. Android requires `startIndex` to equal the
next expected sample index. A hole, out-of-order block, or duplicate block is
rejected; Android never fills missing values or interpolates.

## END (type 3, 16 bytes)

| Offset | Size | Field |
|---:|---:|---|
| 0..7 | 8 | common header |
| 8 | 2 | `sampleCount` = 700 |
| 10 | 2 | reserved = 0 |
| 12 | 4 | CRC32, repeated from BEGIN |

Android accepts a window only when BEGIN/DATA/END share the same `windowSeq`,
exactly 700 samples were received, and both CRC values match the locally
recomputed CRC32.

## Scheduling / acquisition priority

- MAX30102 acquisition, BPM, SpO2, PRV, IMU, LCD, and the B18 NUS transport are
  never blocked waiting for PA notifications.
- The existing BP15 ring continues to be filled by the original 100 Hz path.
- A valid 700-sample window is frozen into the existing `bpTx[700]` buffer.
- The BLE worker computes CRC and sends PA asynchronously.
- B/O/R/D, commands, and the 20 Hz S stream retain scheduling priority over PA.
- PA sends at most one binary notification every 10 ms when B18 is otherwise idle.
- Any PA notify/congestion error drops that PA window only; B18 NUS is not reset.
- A new PA window is queued no more often than every 3 s and only if PA Notify
  is subscribed. If PA is busy, acquisition continues and PA is postponed.

The ESP32 does not calculate PAS/PAD. Android executes the validated pipeline:
`700 raw IR @100 Hz -> resample_poly(5,4) -> Butterworth 0.5-8 Hz -> filtfilt -> z-score -> float32 -> TFLite -> PAS/PAD`.

