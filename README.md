# Ground Column of Robots (UGV-swarm)

## Team
**Mentor:**
- Andrii Ozhovych

**Students:**
- Maryna Ohinska
- Oleksii Lasiichuck
- Nestor Leyko
- Andrii Kulbaba

## The Idea
The project focuses on realizing the concept of the robot column. The project idea was inspired by many facts of using drones and robots in the current Russian-Ukrainian war.

The concept developed in the project can be helpful in the realization of a column of UGV (unmanned ground vehicle) to develop rescue drones or kamikaze drones. The idea of the column simplifies the operator's task of managing different units simultaneously. Instead of controlling several units, the operator leads only one; others follow and perform basic tasks and algorithms.

> **Новачок у проєкті?** Почни з
> [documentation/ARCHITECTURE.md](documentation/ARCHITECTURE.md) — там
> простою мовою розписано, як усе працює і куди лізти для типових задач.

---

## Firmware

- **Target:** ESP32
- **Framework:** ESP-IDF **v5.5.1**
- **Extras:** BTstack + Bluepad32 (PS4/PS5 controller over Bluetooth), `esp32-camera`, Wi-Fi streaming, dot-detection driving logic.

You do **not** need to install ESP-IDF, the Xtensa toolchain, or Python locally. Everything needed to compile lives inside a Docker image based on the official `espressif/idf:v5.5.1` image, so the build is identical on **Windows, Linux, and macOS**.

### Firmware modes

One binary contains both roles (see the paper, section IV.B):

- **Leader** — driven by the pilot over Bluetooth (PS4/PS5 gamepad via Bluepad32). Pair by holding **PS + Share**.
- **Follower** — autonomous: tracks the three red LEDs on the back of the preceding robot (detection pipeline in `main/object-detection/`, control loop in `main/main.c`).

The role is selected at boot by the SW1 switch. On boards without the switch,
flip **`DEFAULT_ROLE_LEADER`** at the top of [`main/main.c`](main/main.c)
(`0` = follower, `1` = leader) and rebuild. Hardware options live in
`idf.py menuconfig` → **UGV column configuration**:

| Option | Meaning |
|--------|---------|
| `UGV_MODE_SWITCH_GPIO` | GPIO of SW1 (read at boot: 1 = leader, 0 = follower). `-1` if the board has no switch |
| `UGV_ENABLE_WEB_STREAM` | MJPEG debug stream over Wi-Fi (adds latency — keep off during runs) |
| `UGV_WIFI_SSID` / `UGV_WIFI_PASSWORD` | Wi-Fi credentials for the debug stream (no longer hardcoded) |
| `UGV_HOSTNAME` | Stable stream address `http://<name>.local/`; empty = auto `ugv-XXXX` from the MAC |
| `UGV_ESPNOW_CHANNEL` | Anchor Wi-Fi channel for the robot-to-robot link when not on a hotspot |

### Debug web pages

With the stream enabled, every robot advertises a **permanent mDNS address**
(printed in its boot log), e.g. `http://ugv-3f7c.local/` — the same URL on
every boot, regardless of DHCP. Open it in any browser on the same Wi-Fi:

| Page | What it shows |
|------|---------------|
| `/` | Live camera stream. Green boxes = the three LEDs the detector locked onto; the small square top-left is the status: green = locked, red = lost |
| `/mask` | Same stream, but every pixel that passes the "red" thresholds is painted green — literally what the detector sees. Use it to tune thresholds |
| `/log` | Live device log (last ~6 KB), auto-refreshing every second. No cable needed |

`.local` names resolve out of the box on macOS, Linux, iOS/Android and
Windows 10+; on older Windows install Bonjour, or just use the IP from the
boot log.

### Robot-to-robot link (ESP-NOW)

Robots also talk to each other directly over ESP-NOW (no router needed).
Currently a proof of concept: the leader broadcasts a counted "ping" at 5 Hz,
followers log every received message (see it on `/log`). Followers find the
sender's channel automatically by scanning; the only rule is: **if you use a
hotspot, put the leader on it too**. Full protocol design and next steps:
[documentation/ESPNOW_COLUMN_LINK_PLAN.md](documentation/ESPNOW_COLUMN_LINK_PLAN.md).

---

## Quick start (all platforms)

The whole cycle is wrapped in one helper script — build in Docker, flash and
monitor from the host (Docker has no USB access on macOS/Windows). Host
prerequisites: Docker + `pip install esptool` (pyserial comes with it).

| Action | macOS / Linux | Windows (PowerShell) |
|--------|---------------|----------------------|
| build + flash + monitor | `./ugv.sh` | `.\ugv.ps1` |
| build only | `./ugv.sh build` | `.\ugv.ps1 build` |
| flash last build | `./ugv.sh flash` | `.\ugv.ps1 flash` |
| serial monitor | `./ugv.sh monitor` | `.\ugv.ps1 monitor` |
| full clean | `./ugv.sh clean` | `.\ugv.ps1 clean` |
| shell in the container | `./ugv.sh shell` | `.\ugv.ps1 shell` |

The serial port is auto-detected on every platform; pass it explicitly if you
have several adapters (`./ugv.sh flash /dev/ttyUSB0`, `.\ugv.ps1 flash COM5`).

Platform notes:
- **Windows**: if scripts are blocked, run `powershell -ExecutionPolicy Bypass -File ugv.ps1`.
  The monitor exits with `Ctrl+]`. Works in plain PowerShell — no WSL needed
  (in Git Bash / WSL you can use `./ugv.sh` instead).
- **Linux**: add yourself to the serial group once: `sudo usermod -aG dialout $USER`
  (then re-login).
- **macOS/Linux**: optional [`tio`](https://github.com/tio/tio) gives a nicer
  monitor (`Ctrl-t q` to quit); the script falls back to pyserial automatically.

---

## Build with Docker (Windows / Linux / macOS)

### 1. Prerequisites (all platforms)

| Platform | Install |
|----------|---------|
| **Windows** | [Docker Desktop](https://www.docker.com/products/docker-desktop/) (WSL 2 backend recommended) |
| **macOS** | [Docker Desktop](https://www.docker.com/products/docker-desktop/) (Apple Silicon & Intel both work) |
| **Linux** | [Docker Engine](https://docs.docker.com/engine/install/) + the Compose plugin |

Verify Docker is running:

```bash
docker --version
docker compose version    # or: docker-compose version
```

> **Why Docker?** A container is an isolated environment that ships the exact
> toolchain, cmake, and Python the firmware was built against. On Windows and
> macOS, Docker runs this Linux environment inside a lightweight VM — so the
> *same* build works everywhere. See `Docker Guide.pdf` for the concepts.

### 2. Clone

```bash
git clone <repo-url>
cd UGV-swarm
```

> The BTstack integration script that runs during the image build must keep
> Unix (LF) line endings. `.gitattributes` enforces this automatically, so a
> Windows clone builds correctly with no manual `git config` needed.

### 3. Build the firmware

The build image is defined in [`Dockerfile`](Dockerfile) and orchestrated by
[`docker-compose.yml`](docker-compose.yml). One command builds everything:

```bash
docker compose run --rm build
```

- `--rm` removes the container after it exits (keeps things tidy).
- On the **first** run Docker builds the image (downloads ESP-IDF, integrates
  BTstack) — this takes several minutes. Subsequent runs are cached and fast.
- Your project folder is bind-mounted into the container, so the compiled
  binaries land back on the host under `build/`:
  - `build/bootloader/bootloader.bin`
  - `build/partition_table/partition-table.bin`
  - `build/robots.bin`

> If your Compose is the older standalone binary, use `docker-compose run --rm build` instead.

### 4. Other build commands

Anything after the service name is passed to `idf.py` inside the container:

```bash
docker compose run --rm build idf.py fullclean      # clean build
docker compose run --rm build idf.py menuconfig      # configure (interactive)
docker compose run --rm build idf.py size            # binary size report
docker compose run --rm build bash                   # shell inside the image
```

---

## Flashing the board

USB passthrough into Docker Desktop is **not** available on Windows or macOS, so
the recommended flow is: **build in Docker, flash from the host.** Install
`esptool` on the host once (`pip install esptool`).

### macOS

```bash
ls /dev/tty.*                      # find your board, e.g. /dev/tty.usbserial-10
./flash.sh /dev/tty.usbserial-10   # helper script in this repo
```

### Linux

You can flash from the host, or pass the device straight into the container:

```bash
# From the host:
ls /dev/ttyUSB* /dev/ttyACM*
./flash.sh /dev/ttyUSB0

# ...or flash from inside the container (Linux only) by adding the device:
docker compose run --rm --device=/dev/ttyUSB0 build \
  idf.py -p /dev/ttyUSB0 flash monitor
```

### Windows

Flash from the host (PowerShell), where the COM port is visible:

```powershell
pip install esptool
# COMx is shown in Device Manager -> Ports (COM & LPT)
esptool --chip esp32 -p COM5 -b 460800 --before default-reset --after hard-reset ^
  write-flash --flash-mode dio --flash-size detect --flash-freq 40m ^
  0x1000 build/bootloader/bootloader.bin ^
  0x8000 build/partition_table/partition-table.bin ^
  0x10000 build/robots.bin
```

After flashing, connect a PS4/PS5 controller by holding **PS + Share** until it
pairs with the board.

### Serial monitor

```bash
# macOS / Linux
idf.py -p <PORT> monitor          # if IDF installed on host
# or use any serial monitor (screen, minicom, PuTTY) at 115200 baud
```

---

## Project layout

| Path | Purpose |
|------|---------|
| `main/main.c` | Entry point: role selection, follower control loop, leader startup |
| `main/camera-logic/` | Camera init (OV3660, YUV422/CIF, 180° rotation) |
| `main/object-detection/` | Red LED triple detection (see `main/DOT_DETECTION.md`) |
| `main/driving-logic/` | Motor PWM layer + Bluetooth gamepad handling |
| `main/wifi/` | Wi-Fi, MJPEG stream, `/mask` view, `/log` page, mDNS |
| `main/comms/` | ESP-NOW robot-to-robot link |
| `documentation/` | Architecture guide, ESP-NOW protocol plan |
| `src/components/` | Bluepad32 + NVS/system console components |
| `btstack/` | BTstack Bluetooth stack (integrated into IDF at image-build time) |
| `Dockerfile` / `docker-compose.yml` | Reproducible build environment |
| `ugv.sh` / `ugv.ps1` | One-command build/flash/monitor helpers |
| `sdkconfig.defaults` | Board configuration (PSRAM, Bluetooth, 240 MHz, -O2); `sdkconfig` is generated from it on the first build |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Could not open ... port is busy` when flashing | Something holds the serial port — usually your own monitor. Quit `tio` (`Ctrl-t q`) or close the terminal, then flash again. |
| `Camera init failed` + `I2C hardware NACK` in the log | Hardware, not code: the camera shares the I2C bus (GPIO 26/27) with other peripherals. A hung or half-connected device (e.g. the LCD) breaks sensor init. Power-cycle the board fully, reseat the camera ribbon, check peripheral wiring. |
| Follower detects nothing (`/` shows a red square) | Are the LEDs actually powered? Open `/mask` — lit LEDs must show as green blobs. If not, tune thresholds in `main/object-detection/dot_detection.c`. If blobs are green but no lock, check `/log` for `rejects dy=... sym=...` and adjust geometry limits. |
| Robots don't hear each other (ESP-NOW) | They must share a Wi-Fi channel. Same hotspot for everyone — or no hotspot for everyone. A follower on a hotspot cannot hear a leader that isn't on it. |
| `bad interpreter: /usr/bin/env python3^M` during image build | CRLF line endings. `.gitattributes` prevents this; re-clone or run `git add --renormalize .`. |
| First build is very slow | Expected — it downloads ESP-IDF and integrates BTstack. Later builds are cached. |
| Board not found when flashing | Check the port (`ls /dev/cu.usbserial-*` / Device Manager) and the USB cable (charge-only cables don't enumerate). |
| Permission denied on `/dev/ttyUSB0` (Linux) | Add your user to the `dialout` group: `sudo usermod -aG dialout $USER`, then re-login. |
| Gamepad won't pair | Hold **PS + Share** until the light bar double-flashes. Note: pairing keys are erased on every boot (`uni_bt_del_keys_unsafe` in `my_platform.c`), so re-pair after each restart. |

---

## Planned / ideas for the next iteration

1. **Full column protocol over ESP-NOW** — the link works (ping PoC); next
   is the real beacon `{speed, turn, estop}` with column-wide E-STOP and
   heartbeat fail-safe. Design is ready:
   [documentation/ESPNOW_COLUMN_LINK_PLAN.md](documentation/ESPNOW_COLUMN_LINK_PLAN.md).
2. **Find the SW1 switch GPIO** and set `UGV_MODE_SWITCH_GPIO`, so the role
   is chosen by the physical switch instead of the define in `main.c`.
3. **CI build** — a GitHub Actions workflow running `docker compose run --rm build`
   on every push would catch broken builds automatically.
4. **1602 LCD status display** (paper IV.B: "Following / Searching / Lost") —
   the hardware exists, the code doesn't yet.
5. **Trim the vendored BTstack tree** (`btstack/test`, `btstack/example`) to
   shrink clones, and drop the `cmd_*_4.4` copies in `src/components/`.
