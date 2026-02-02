FROM espressif/idf:v5.5.1

ARG DEBIAN_FRONTEND=noninteractive
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

WORKDIR /project

RUN apt-get update && apt-get install -y \
    git \
    wget \
    cmake \
    libusb-1.0-0 \
    python3 \
    locales \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8


COPY . /project

RUN cd btstack/port/esp32 && \
    chmod +x integrate_btstack.py && \
    ./integrate_btstack.py && \
    cp /opt/esp/idf/components/bt/host/bluedroid/external/sbc/decoder/include/oi_bt_spec.h \
    /opt/esp/idf/components/btstack/3rd-party/bluedroid/decoder/include/

    ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD [ "idf.py", "build" ]