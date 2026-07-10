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

---

## Quick start (macOS / Linux)

The whole cycle is wrapped in one script — build in Docker, flash and monitor
from the host (Docker has no USB access on macOS/Windows):

```bash
./ugv.sh            # build + flash + monitor (port auto-detected)
./ugv.sh build      # just compile
./ugv.sh flash      # just flash the last build
./ugv.sh monitor    # serial log (tio if installed, else pyserial)
./ugv.sh shell      # interactive idf.py shell inside the container
```

Host prerequisites: Docker + `pip install esptool` (pyserial comes with it).
Optional: [`tio`](https://github.com/tio/tio) for a nicer serial monitor.

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
| `main/` | Application entry point and logic (camera, driving, object detection, Wi-Fi) |
| `src/components/` | Bluepad32 + NVS/system console components |
| `btstack/` | BTstack Bluetooth stack (integrated into IDF at image-build time) |
| `Dockerfile` | ESP-IDF build image definition |
| `docker-compose.yml` | Build orchestration |
| `flash.sh` | Host-side flashing helper (macOS/Linux) |
| `sdkconfig.defaults` | Board configuration (PSRAM, Bluetooth, 240 MHz, -O2); `sdkconfig` is generated from it on the first build |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `failed to read dockerfile` / build can't find Dockerfile on Linux | Ensure the file is named `Dockerfile` (capital D) — fixed in this repo. |
| `bad interpreter: /usr/bin/env python3^M` during image build | CRLF line endings. `.gitattributes` prevents this; re-clone or run `git add --renormalize .`. |
| First build is very slow | Expected — it downloads ESP-IDF and integrates BTstack. Later builds are cached. |
| Board not found when flashing | Check the port (`ls /dev/tty.*` / Device Manager) and that no other program holds the serial port. |
| Permission denied on `/dev/ttyUSB0` (Linux) | Add your user to the `dialout` group: `sudo usermod -aG dialout $USER`, then re-login. |

---

## Suggested improvements

1. **Pin dependency versions.** `main/idf_component.yml` uses `espressif/esp32-camera: '*'` and `esp_jpeg: '*'`. The `'*'` will silently pull newer major versions and can break the build. Pin them (`dependencies.lock` already resolved `esp32-camera 2.1.4`, `esp_jpeg 1.3.1`).
2. **CI build.** Add a GitHub Actions workflow that runs `docker compose run --rm build` on every push so regressions are caught automatically.
3. **Consolidate the console component variants.** `src/components/` ships both `cmd_nvs`/`cmd_nvs_4.4` and `cmd_system`/`cmd_system_4.4`. Since the project targets IDF 5.5, drop the `*_4.4` copies to reduce confusion.
4. **Trim the vendored BTstack tree.** Only a subset of `btstack/` is used. Excluding `btstack/test`, `btstack/example`, and unused ports from the repo (or at least from the Docker context) shrinks clones and build context.
5. **Multi-arch note.** The `espressif/idf` image is multi-arch, so it runs natively on Apple Silicon and x86 — no `platform:` override needed. Keep it that way when bumping IDF versions.
