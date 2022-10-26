FROM mcr.microsoft.com/vscode/devcontainers/base:bullseye

ARG GCC_URL="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-aarch64-linux.tar.bz2"
ARG GCC_TAR="gcc-arm-none-eabi-10.3-2021.10-aarch64-linux.tar.bz2"

ARG SDK_URL="https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/sdks/nrf5/binaries/nrf5_sdk_17.1.0_ddde560.zip"
ARG SDK_ZIP="nrf5_sdk_17.1.0_ddde560.zip"

ARG ECC_GIT="https://github.com/kmackay/micro-ecc.git"

ARG DEPS=/opt

RUN \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        curl unzip tar make git dos2unix && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

RUN \
    cd "${DEPS}" && \
    curl -o "${GCC_TAR}" "${GCC_URL}" && \
    tar -xvf "${GCC_TAR}" && \
    rm "${GCC_TAR}"

ENV GNU_INSTALL_ROOT="${DEPS}/gcc-arm-none-eabi-10.3-2021.10/bin/"
ENV GNU_VERSION=10.3
ENV GNU_PREFIX=arm-none-eabi

RUN \
    cd "${DEPS}" && \
    curl -o "${SDK_ZIP}" "${SDK_URL}" && \
    unzip "${SDK_ZIP}" && \
    rm "${SDK_ZIP}"

ENV SDK_ROOT="${DEPS}/nRF5_SDK_17.1.0_ddde560"

RUN \
    cd "${SDK_ROOT}" && \
    git clone "${ECC_GIT}" external/micro-ecc/micro-ecc && \
    cd external/micro-ecc && \
    dos2unix build_all.sh && \
    bash build_all.sh
