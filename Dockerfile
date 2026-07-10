# UGV-swarm ESP32 firmware build image.
# Based on the official Espressif ESP-IDF image so the toolchain, cmake,
# python and idf.py all match the version the firmware was written against.
FROM espressif/idf:v5.5.1

LABEL org.opencontainers.image.title="ugv-esp32-project" \
      org.opencontainers.image.description="ESP-IDF build environment for the UGV-swarm firmware" \
      org.opencontainers.image.source="https://github.com/"

ARG DEBIAN_FRONTEND=noninteractive
ENV LC_ALL=C.UTF-8 \
    LANG=C.UTF-8

WORKDIR /project

# The base image already ships git, wget, cmake and python3. We only add
# libusb (needed by esptool) and locales so the UTF-8 locale generates cleanly.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
        locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

COPY . /project

# Integrate BTstack into the IDF component tree (writes into /opt/esp, which is
# OUTSIDE /project, so it survives the runtime bind-mount). We invoke it with
# `python3` explicitly instead of `./integrate_btstack.py` so the build does not
# depend on the file's exec-bit or on a clean LF shebang (Windows checkouts).
RUN cd btstack/port/esp32 \
    && python3 integrate_btstack.py \
    && cp /opt/esp/idf/components/bt/host/bluedroid/external/sbc/decoder/include/oi_bt_spec.h \
          /opt/esp/idf/components/btstack/3rd-party/bluedroid/decoder/include/

# entrypoint.sh sources IDF's export.sh, then runs the CMD.
ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]
CMD [ "idf.py", "build" ]
