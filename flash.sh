#!/bin/bash

# Default port
PORT=${1:-/dev/tty.usbserial-10}

echo "Flashing ESP32 on port: $PORT"

if [ ! -e "$PORT" ]; then
    echo "Error: Port $PORT does not exist!"
    echo "Check your connection or run: ls /dev/tty.*"
    exit 1
fi

esptool -p "$PORT" -b 460800 \
  --before default-reset \
  --after hard-reset \
  --chip esp32 \
  write-flash \
  --flash-mode dio \
  --flash-size detect \
  --flash-freq 40m \
  0x1000 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/robots.bin