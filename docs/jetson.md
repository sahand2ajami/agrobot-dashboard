# The Jetson compute unit

This document describes the single computer that runs the Agrobot dual-robot
dashboard: an NVIDIA Jetson. If you are new to the project, read this first —
it explains what the hardware is, what software is on it, how it is wired to
the network, and which serial devices are attached. No prior context assumed.

## 1. Overview

The **Jetson** is the on-robot computer that runs *everything* in this project.
"Jetson" is NVIDIA's family of small, power-efficient computers built around an
ARM CPU plus an integrated GPU (graphics processor, also used here for AI
inference). This particular unit is a **Jetson AGX Orin** — the highest-end
model in the family.

On this one board the Jetson:

- serves the **web dashboard** (the browser UI a driver uses to teleoperate the
  robot),
- runs **ROS 2** (Robot Operating System 2 — the middleware that carries
  messages between robot software components),
- captures video from two **ZED stereo cameras** (front and rear),
- runs **YOLO** person detection on the GPU (an AI model that draws boxes around
  people in the camera feed),
- talks to the **chassis** (the drive base) over a USB serial cable, and
- talks to the **PLC** (Programmable Logic Controller — the industrial
  controller that runs the tree-planter's auger and planter) over Ethernet.

Everything below is the ground truth for that box.

## 2. Hardware at a glance

The physical machine and its main components.

| Component | Detail |
|-----------|--------|
| Board | NVIDIA Jetson AGX Orin Developer Kit |
| Module | Jetson AGX Orin 64 GB (P-Number `p3701-0005`) |
| CPU | 12-core ARM Cortex-A78AE, `aarch64` (64-bit ARM) |
| GPU | Integrated NVIDIA Ampere GPU (part of the Orin SoC) |
| RAM | 64 GB (~61 GiB usable) plus 30 GiB swap |
| Storage | 1.9 TB NVMe SSD (`/dev/nvme0n1p1`, mounted at `/`, ~15% used) |
| Default power mode | **MAXN** — maximum performance, all CPU cores and clocks unlocked |

Notes for a first-time reader:

- **SoC** means "System on a Chip": the CPU and GPU share one silicon die, so
  the GPU is not a separate plug-in card.
- **MAXN** is the least power-restricted profile. The Jetson can be set to lower
  power modes to save energy, but this unit runs unrestricted by default.

## 3. Software stack

NVIDIA ships a bundle called **JetPack** that pins the OS, the AI libraries, and
the GPU drivers together as one tested set. The table lists what is installed.

| Layer | Version / detail |
|-------|------------------|
| JetPack | 6.2.1 |
| L4T (Linux for Tegra) | 36.4.7 |
| Operating system | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| Kernel | 5.15.148-tegra |
| CUDA | 12.6.85 |
| cuDNN | 9.20.0.48 |
| TensorRT | 10.7.0.23 |
| VPI | 3.2.4 |
| Vulkan | 1.3.204 |
| OpenCV | 4.8.0 — **built WITHOUT CUDA** |
| ROS 2 | Humble (from `ros-humble-*` apt packages, never pip) |
| jetson-stats | 4.3.2 |

What the acronyms mean, and two things worth flagging:

- **L4T** ("Linux for Tegra") is NVIDIA's customized Linux that JetPack is built
  on; **Tegra** is the chip family the Orin belongs to.
- **CUDA / cuDNN / TensorRT / VPI / Vulkan** are the GPU compute and vision
  libraries. YOLO detection uses these to run on the GPU.
- **OpenCV is built WITHOUT CUDA.** This matters: OpenCV image operations run on
  the CPU here, not the GPU. Anyone expecting GPU-accelerated OpenCV calls will
  be surprised — the GPU acceleration in this project comes through the AI stack
  (CUDA/TensorRT), not through OpenCV.
- **ROS 2 packages come from apt (`ros-humble-*`), never from pip.** Installing
  ROS via pip on a Jetson breaks the tested JetPack combination.

## 4. Useful commands

Quick commands to inspect the machine's live state and versions.

| Command | What it shows |
|---------|---------------|
| `jtop` | Live dashboard of CPU / GPU / RAM / temperatures / power draw (from jetson-stats). Press `q` to quit. |
| `jetson_release` | One-shot summary of JetPack, L4T, CUDA, and module info. |
| `sudo nvpmodel -q` | The current power mode (e.g. MAXN). `nvpmodel` = NVIDIA power model tool. |
| `free -h` | RAM and swap usage in human-readable units. |
| `df -h` | Disk usage per mounted filesystem (look for `/` on `/dev/nvme0n1p1`). |

`jtop` and `jetson_release` come from the **jetson-stats** package; if they are
missing, that package is not installed.

## 5. Network interfaces

The Jetson has three network interfaces, each with a distinct job. Understanding
these is essential — one of them has a real-world gotcha described below.

| Interface | Type | Role |
|-----------|------|------|
| `eno1` | Wired Ethernet | Connects to the **PLC** (agrobot chassis) or the **Clearpath Jackal**, on the `192.168.1.0/24` subnet. The launch script gives the Jetson `192.168.1.100` here; the PLC CPU is at `192.168.1.2:502`. |
| `wlP1p1s0` | WiFi | How phones/laptops reach the dashboard in the field (e.g. via the field WiFi router). |
| `tailscale0` | Tailscale VPN | Remote access over the internet using a `100.x.y.z` address. |

Terminology: a **subnet** like `192.168.1.0/24` is a block of 256 addresses
(`192.168.1.0`–`192.168.1.255`) that are treated as being on the same local
network. `502` is the TCP port for **Modbus TCP**, the protocol the dashboard
uses to talk to the PLC.

### The WiFi / PLC subnet collision (important)

There is a genuine trap here. The field WiFi router **also
hands out `192.168.1.x` addresses** — the *same* subnet the wired PLC link uses.
If the Jetson claims the entire `192.168.1.0/24` block on the wired `eno1` port,
that route outranks the WiFi route, so replies meant for a WiFi client (a phone
or laptop viewing the dashboard) get sent out `eno1` instead — and `eno1` is
dead whenever the robot is powered off. The dashboard then becomes unreachable
over WiFi.

**The fix (already handled by the launch scripts):** when another interface is
already on `192.168.1.0/24` (i.e. WiFi is present), `eno1` is *not* given the
whole `/24`. Instead it gets its address as a `/32` (a single-host route) plus a
`/32` host route to just the PLC (`192.168.1.2`). A `/32` is more specific than
the WiFi `/24`, so traffic to the PLC still goes out `eno1` while all other
`192.168.1.x` traffic — including WiFi dashboard clients — stays on WiFi. If no
WiFi conflict exists, `eno1` claims the normal `/24` (needed for the Jackal's
DDS/ROS traffic).

For the full detail see the [PLC integration guide](plc.md) and the
`setup_robot_subnet` function in the repo's `launch_dashboard.sh`.

## 6. Serial & USB devices

Beyond the network, the Jetson talks to hardware over USB. Serial devices are
pinned to **stable `/dev` symlinks** by udev rules so their names never change
across reboots or replugs.

| Device path | What it is |
|-------------|-----------|
| `/dev/agrobot_base` | The **agrobot chassis controller** (FTDI USB-to-serial adapter). `robot_base_node` speaks Modbus RTU here at 38400 baud. Pinned by `config/udev/99-agrobot-serial.rules`. Fallback path: `/dev/ttyUSB0`. |
| `/dev/gnss` | The **GNSS receiver** (GeoAstra RTU608BT — a GPS/satellite positioning unit), as a USB udev symlink. Over Bluetooth it appears as `/dev/rfcomm0` instead. |
| ZED 2i cameras (×2) | Two ZED 2i **stereo cameras** on USB (front + rear), opened through the pyzed SDK. Used for video and depth. |

Terms: a **udev rule** is a Linux rule that assigns a fixed name to a device
based on its unique USB serial number; **Modbus RTU** is the serial version of
the Modbus protocol; **GNSS** ("Global Navigation Satellite System") is the
general term for GPS-style positioning.

### Critical safety note — do not share the chassis serial line

**Never let the GNSS reader grab a bare `/dev/ttyUSB*` device.** That is where
the **chassis** lives. A serial line supports only one master; if two programs
(the chassis driver and the GNSS reader) open the same `/dev/ttyUSB*` at once,
they collide and **knock the chassis offline**. Because the kernel assigns
`ttyUSB0`, `ttyUSB1`, … in plug order, the GNSS receiver can accidentally land
on the port the chassis expects.

Always use the dedicated symlinks — **`/dev/agrobot_base`** for the chassis and
**`/dev/gnss`** for the GNSS receiver — never a raw `/dev/ttyUSB*` path. The
udev rule in `config/udev/99-agrobot-serial.rules` exists precisely to remove this
race.

## 7. Where to go next

Now that you know the machine, these documents explain the software it runs.

- [Project README](../README.md) — start here for what the project does and how
  to run it.
- [Architecture guide](architecture.md) — the map of the codebase: layering,
  the dependency rule, and how to extend it safely.
- [PLC integration guide](plc.md) — how the dashboard talks to the PLC,
  including full detail on the network wiring above.
- [Developer guide (DEVELOPMENT.md)](../DEVELOPMENT.md) — build/test commands, the chassis
  abstraction, and the HTTP API reference.
