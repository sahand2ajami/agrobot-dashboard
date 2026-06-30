# ─────────────────────────────────────────────────────────────────────────────
# Dual-Robot Dashboard — Dockerfile  (Jetson / L4T R36.x / JetPack 6.1)
#
# Target platform: NVIDIA Jetson, L4T R36.x (JetPack 6.1), CUDA 12.6,
#                  Ubuntu 22.04 Jammy, Python 3.10, aarch64
#
# ── Build ────────────────────────────────────────────────────────────────────
#   docker compose build           (uses docker-compose.yml)
#   docker build -t dual-robot-dashboard .
#
# ── Run ──────────────────────────────────────────────────────────────────────
#   docker compose up              (recommended — see docker-compose.yml)
#
# ── JetPack torch wheels ──────────────────────────────────────────────────────
#   The default ARGs below match JetPack 6.1 / L4T R36.3+ (CUDA 12.6).
#   If you are on a different JetPack, find your wheels at:
#     https://developer.download.nvidia.com/compute/redist/jp/
#   Then override at build time:
#     docker build \
#       --build-arg TORCH_WHL=<url> \
#       --build-arg TORCHVISION_WHL=<url> \
#       -t dual-robot-dashboard .
#
# ── ZED cameras ───────────────────────────────────────────────────────────────
#   pyzed 4.2 is installed in this image. The ZED C++ runtime (libsl_zed.so)
#   lives on the HOST at /usr/local/zed and is bind-mounted at runtime by
#   docker-compose.yml. The image itself has NO bundled ZED SDK.
#
# ── Dashboard ports ───────────────────────────────────────────────────────────
#   8766 main dashboard   8767 standalone PLC HMI
#   8768 AMR-PLC bridge   8769 dashboard-plc combined
#   With network_mode: host (docker-compose default) these are the host ports.
# ─────────────────────────────────────────────────────────────────────────────

# ── Stage 1: ROS 2 Humble base + apt packages ─────────────────────────────────
FROM ros:humble-ros-base-jammy AS ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        # Python build tooling
        python3-pip \
        python3-dev \
        # ROS 2 runtime packages
        ros-humble-std-msgs \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-nav-msgs \
        ros-humble-tf2-ros \
        ros-humble-rosbridge-server \
        ros-humble-robot-state-publisher \
        python3-colcon-common-extensions \
        # System tools used by launch scripts and serve.py
        curl \
        iproute2 \
        net-tools \
        psmisc \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ── Stage 2: Python pip dependencies ─────────────────────────────────────────
FROM ros-base AS python-deps

# Install everything from requirements.txt.
# ultralytics pulls CPU torch/torchvision as transitive deps — we overwrite
# them with the NVIDIA JetPack CUDA build in the next step.
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Replace CPU torch with NVIDIA's JetPack 6.1 / CUDA 12.6 build.
# These wheels match the installed host torch (2.5.0a0+872d972e41.nv24.8)
# and torchvision (0.20.0a0+afc54f7) on L4T R36.4.x / JetPack 6.1.
# Update the ARGs below if you upgrade JetPack or move to a different Jetson.
ARG TORCH_WHL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.09.16820938-cp310-cp310-linux_aarch64.whl
ARG TORCHVISION_WHL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torchvision-0.20.0a0+afc54cf-cp310-cp310-linux_aarch64.whl
RUN pip3 install --no-cache-dir "${TORCH_WHL}" "${TORCHVISION_WHL}" || \
    echo "[WARNING] JetPack CUDA torch install failed — YOLO inference will use CPU"

# ZED 2i Python bindings.
# Version must match the SDK installed on the host (currently 4.2.5 → pyzed 4.2).
# The C++ runtime (libsl_zed.so) is NOT baked into this image; it is provided
# at runtime via the /usr/local/zed bind-mount in docker-compose.yml.
RUN pip3 install --no-cache-dir "pyzed==4.2"

# ── Stage 3: Application ──────────────────────────────────────────────────────
FROM python-deps AS app

WORKDIR /app

# Copy the whole project (see .dockerignore for excluded paths)
COPY . /app/

# Build the agrobot ROS package (robot_base_node, odom_calculation, etc.)
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select avatar_robot_base && \
    rm -rf /app/build/ /app/log/

# Runtime data directories (overridden by bind-mounts in docker-compose.yml)
RUN mkdir -p /app/logs/dashboard /app/logs/gnss /app/logs/plants

# Entrypoint: source ROS 2 + install tree, expose ZED SDK libs and CUDA, exec CMD
RUN cat > /entrypoint.sh <<'EOF'
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
[ -f /app/install/setup.bash ] && source /app/install/setup.bash

# ZED SDK C++ runtime (bind-mounted from /usr/local/zed on the host)
export LD_LIBRARY_PATH="/usr/local/zed/lib:/usr/local/cuda-12.6/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

export PYTHONPATH="/app/dashboard:/app/scripts${PYTHONPATH:+:${PYTHONPATH}}"
exec "$@"
EOF
RUN chmod +x /entrypoint.sh

# ── Runtime environment ────────────────────────────────────────────────────────
ENV DASHBOARD_HEADLESS=1
ENV ROBOT_CHASSIS=agrobot

EXPOSE 8766 8767 8768 8769

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/app/dashboard/serve.py", "--chassis", "agrobot", "--port", "8766"]
