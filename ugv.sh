#!/bin/bash
# UGV helper: one command for the whole build->flash->monitor cycle.
#
#   ./ugv.sh build            build firmware in Docker
#   ./ugv.sh flash [PORT]     flash from the host with esptool
#   ./ugv.sh monitor [PORT]   serial monitor (tio if installed, else pyserial)
#   ./ugv.sh all [PORT]       build + flash + monitor
#   ./ugv.sh clean            full clean (removes all build artifacts)
#   ./ugv.sh shell            interactive idf.py shell inside the container
#
# PORT is auto-detected (first USB serial adapter) when omitted.
set -e
cd "$(dirname "$0")"

BAUD_FLASH=460800
BAUD_MONITOR=115200

detect_port() {
    ls /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial-* \
       /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1
}

compose() {
    if docker compose version >/dev/null 2>&1; then
        docker compose "$@"
    else
        docker-compose "$@"
    fi
}

esptool_cmd() {
    if command -v esptool >/dev/null 2>&1; then
        esptool "$@"
    else
        python3 -m esptool "$@"
    fi
}

do_build() {
    compose run --rm build
}

do_flash() {
    local port=$1
    [ -n "$port" ] || { echo "No serial port found. Plug in the board or pass it: ./ugv.sh flash /dev/cu.usbserial-XXX"; exit 1; }
    echo "Flashing on $port"
    esptool_cmd -p "$port" -b $BAUD_FLASH \
        --before default-reset --after hard-reset --chip esp32 \
        write-flash --flash-mode dio --flash-size detect --flash-freq 40m \
        0x1000 build/bootloader/bootloader.bin \
        0x8000 build/partition_table/partition-table.bin \
        0x10000 build/robots.bin
}

do_monitor() {
    local port=$1
    [ -n "$port" ] || { echo "No serial port found."; exit 1; }
    if command -v tio >/dev/null 2>&1; then
        tio -b $BAUD_MONITOR "$port"
    else
        # pyserial ships together with esptool
        python3 -m serial.tools.miniterm --raw "$port" $BAUD_MONITOR
    fi
}

cmd=${1:-all}
port=${2:-$(detect_port)}

case "$cmd" in
    build)   do_build ;;
    flash)   do_flash "$port" ;;
    monitor) do_monitor "$port" ;;
    all)     do_build; do_flash "$port"; do_monitor "$port" ;;
    clean)   compose run --rm build idf.py fullclean ;;
    shell)   compose run --rm build bash ;;
    *)       echo "Usage: ./ugv.sh [build|flash|monitor|all|clean|shell] [PORT]"; exit 1 ;;
esac
