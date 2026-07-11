# Column radio link — design plan (ESP-NOW)

Status: **step 1 implemented as a proof of concept** — `main/comms/column_link.c`
broadcasts a counted "ping" from the leader at 5 Hz, followers receive and log
it, and followers auto-scan channels 1-13 to find the sender (so the channel
management section below is already live). Steps 2-4 (real beacon payload,
E-STOP, heartbeat rules in the follower loop) remain to be done.

## Why

Today the column holds together only through vision. The paper (VI.B) already
names the failure mode: when several followers lose the LEDs at once, each
spins in search mode and the column disintegrates. A lightweight
robot-to-robot radio channel gives:

1. **Column-wide E-STOP** — one button on the pilot's gamepad stops every
   robot within milliseconds, even those that see nothing.
2. **Heartbeat fail-safe** — a follower that hears no leader beacons for
   N ms stops instead of searching forever.
3. **Search feed-forward** — a follower that lost the LEDs turns toward
   where the leader *says* it is going, not where it last guessed.

## Why ESP-NOW and not Bluetooth

| | ESP-NOW | BLE advertising via btstack |
|---|---|---|
| Radio | Wi-Fi (2.4 GHz) | Bluetooth (2.4 GHz) |
| Conflicts with PS4 gamepad | no (BT untouched; SW coexist already enabled) | yes — Bluepad32 owns GAP scanning on the leader |
| Payload | 250 B | ~26 B |
| Latency | 1–2 ms | ~advertising interval (≥100 ms practical) |
| Pairing | none (broadcast to FF:FF:FF:FF:FF:FF) | none for adv, but scan config clashes |
| Code | ~80 lines total | btstack GAP plumbing, fragile |

Decision: **ESP-NOW broadcast**, leader → all followers. Followers do not
transmit in v1 (no ACKs needed; loss-tolerant beacon stream).

## Protocol v1

Broadcast frame, little-endian, 16 bytes, sent by the **leader at 20 Hz**
(every control tick) and on every state change (immediately):

```c
typedef struct __attribute__((packed)) {
    uint8_t  magic;      // 0xC0 — column protocol marker
    uint8_t  version;    // 1
    uint8_t  flags;      // bit0: ESTOP, bit1: manual mode active
    uint8_t  reserved;
    uint32_t seq;        // monotonically increasing, wraps
    int16_t  speed;      // leader's current speed command, PWM units
    int16_t  turn;       // leader's current turn command, PWM units (+ = right)
    uint32_t uptime_ms;  // for debugging/latency estimates
} column_beacon_t;
```

Follower-side rules (integration with the current control loop, in
priority order):

1. `ESTOP` flag set → all motors 0 until the flag clears. Overrides
   everything, including manual experiments.
2. No valid beacon for `COLUMN_HEARTBEAT_TIMEOUT_MS` (default 1500 ms)
   **and** no visual lock → stop motors (replaces endless search only in
   the "radio says leader is gone" case; pure-vision behaviour without
   radio stays exactly as today).
3. Visual tracking lost but beacons alive → search turn direction taken
   from the sign of the last `turn` field instead of the follower's own
   last turn.
4. Visual tracking active → beacons are ignored for driving (vision wins);
   they only feed the fail-safe timer.

E-STOP source on the leader: a dedicated gamepad button in
`my_platform_on_controller_data` (proposed: PS button short-press toggles,
or L1+R1 held). Exact mapping to be picked when implementing.

## Channel management

ESP-NOW requires all peers on one Wi-Fi channel.

- **Field mode** (`UGV_ENABLE_WEB_STREAM=n`, default): Wi-Fi started in STA
  mode without connecting; fixed channel from Kconfig
  (`UGV_ESPNOW_CHANNEL`, default 1). Same constant on every robot.
- **Debug mode** (stream enabled): the STA joins the hotspot and lands on
  the AP's channel; ESP-NOW then uses that channel automatically. All
  robots must be on the same hotspot — already true when streaming.

## Security (later, cheap)

ESP-NOW supports per-peer AES keys, but broadcast frames are unencrypted.
V1 mitigations: `magic`+`version` filtering and dropping non-monotonic
`seq` (replay). V2 option: switch from broadcast to a static peer list
with LMK encryption, or XOR-MAC the payload with a shared secret from
Kconfig. Not a blocker for lab use; revisit before any field deployment
that matters.

## Implementation steps (~half a day)

1. `main/comms/column_link.h/.c`: `column_link_init(channel)`,
   `column_link_broadcast(const column_beacon_t*)`,
   `column_link_get_last(column_beacon_t*, uint32_t *age_ms)` (mutex-guarded
   copy, mirroring `dot_detection_get_last`).
2. Leader: fill beacon from the ramped `speed`/`turn` in
   `my_platform_on_controller_data`; 20 Hz timer task broadcasts it.
   E-STOP button toggles `flags.ESTOP`.
3. Follower: init link in `follower_main()`; apply rules 1–4 in
   `follower_loop()` (small, isolated edits around the existing logic).
4. Kconfig: `UGV_ESPNOW_CHANNEL`, `UGV_HEARTBEAT_TIMEOUT_MS`.
5. Bench test: leader + one follower on blocks (wheels up), verify
   beacon age < 100 ms, E-STOP latency, heartbeat stop on leader power-off.

## Open questions

- Which gamepad button for E-STOP (needs a decision from the team).
- Should followers re-broadcast beacons (relay) for long columns where the
  tail is out of radio range of the leader? V1: no; revisit if range tests
  fail (ESP-NOW LOS range is typically 100 m+, columns are 10–35 cm apart).
- Follower → leader telemetry (battery, lock status) for the pilot's UI:
  nice-to-have, needs a decision on where to display it.
