#!/usr/bin/env bash
# =============================================================================
# setup_jetson_host.sh — One-time host configuration for the NVIDIA Jetson
# =============================================================================
# Installs and configures three things:
#   1. Tailscale VPN       — gives the Jetson a stable IP reachable from anywhere
#   2. NoMachine server    — remote desktop access over Tailscale
#   3. WiFi network        — saves a network with a priority so the Jetson
#                            connects automatically on boot
#
# Run ONCE on the bare Jetson host before starting the dashboard.
# This script configures the host operating system — it does NOT run inside
# Docker. The Docker container uses host networking and inherits whatever
# the host has configured.
#
# Usage (from the project root):
#   bash scripts/setup_jetson_host.sh
#
# Requirements:
#   - Ubuntu 22.04 (NVIDIA Jetson, aarch64)
#   - A user account with sudo access
#   - An active internet connection
# =============================================================================

set -euo pipefail

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
info() { echo -e "${BLUE}[INFO]${NC}  $*"; }
ok()   { echo -e "${GREEN}[OK]${NC}    $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
die()  { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

# ── Architecture guard ────────────────────────────────────────────────────────
[[ "$(uname -m)" == "aarch64" ]] \
    || die "This script is for aarch64 (NVIDIA Jetson). Detected: $(uname -m)"

# ── NoMachine version pin ─────────────────────────────────────────────────────
# To upgrade NoMachine, update these two values to match the new .deb filename.
# Find the latest ARM64 .deb at:
#   https://www.nomachine.com/download/linux&id=30&s=Arm
# Example filename: nomachine_8.14.2_1_arm64.deb
#   NX_VERSION = "8.14.2"   (the version portion)
#   NX_BUILD   = "1"        (the build number between version and _arm64)
NX_VERSION="8.14.2"
NX_BUILD="1"
# The download URL is assembled automatically from these two values.
NX_DEB="nomachine_${NX_VERSION}_${NX_BUILD}_arm64.deb"
NX_URL="https://download.nomachine.com/download/${NX_VERSION%.*}/Arm/${NX_DEB}"
# ─────────────────────────────────────────────────────────────────────────────

echo ""
echo "========================================================"
echo "  Dual-Robot Dashboard — Jetson Host Setup"
echo "  Tailscale VPN · NoMachine Remote Desktop · WiFi"
echo "========================================================"
echo ""
echo "  This script is safe to re-run at any time."
echo "  Already-installed components are detected and skipped."
echo ""

# =============================================================================
# STEP 1 — TAILSCALE VPN
# =============================================================================
echo -e "${BLUE}── Step 1 / 3 : Tailscale VPN ──────────────────────────────${NC}"
echo ""
echo "  Tailscale assigns this Jetson a stable IP address in the 100.x.x.x"
echo "  range. Once set up, you can always reach the Jetson at that same IP"
echo "  regardless of which WiFi network it is on, even from a different"
echo "  city. Your other devices must also have Tailscale installed and be"
echo "  logged into the same account."
echo ""

if command -v tailscale &>/dev/null; then
    ok "Tailscale is already installed ($(tailscale version | head -1))."
else
    info "Installing Tailscale..."
    curl -fsSL https://tailscale.com/install.sh | sudo sh
    ok "Tailscale installed."
fi

TAILSCALE_IP=$(tailscale ip -4 2>/dev/null || true)
if [[ -n "$TAILSCALE_IP" ]]; then
    ok "Already connected to Tailscale. Jetson IP: ${TAILSCALE_IP}"
else
    info "Connecting to Tailscale..."
    info "Your browser will open so you can log in with the Tailscale account"
    info "that owns this device. Follow the link it prints if no browser opens."
    echo ""
    sudo tailscale up
    TAILSCALE_IP=$(tailscale ip -4 2>/dev/null || echo "<pending — run 'tailscale ip -4'>")
    ok "Tailscale connected. Jetson IP: ${TAILSCALE_IP}"
fi

echo ""

# =============================================================================
# STEP 2 — NOMACHINE REMOTE DESKTOP
# =============================================================================
echo -e "${BLUE}── Step 2 / 3 : NoMachine Remote Desktop ───────────────────${NC}"
echo ""
echo "  NoMachine is a remote desktop server. Once installed, you can open"
echo "  the Jetson's full desktop from your laptop over Tailscale — useful"
echo "  for debugging, file management, and browser access without a"
echo "  physical monitor attached to the Jetson."
echo ""

_install_nomachine() {
    local tmp="/tmp/${NX_DEB}"
    info "Downloading NoMachine ${NX_VERSION} for arm64..."
    info "  Source: ${NX_URL}"
    curl -fL --progress-bar -o "$tmp" "$NX_URL" \
        || die "Download failed. Check the URL or update NX_VERSION in this script."
    info "Installing NoMachine..."
    sudo dpkg -i "$tmp"
    rm -f "$tmp"
    ok "NoMachine ${NX_VERSION} installed."
}

if dpkg -l nomachine 2>/dev/null | grep -q "^ii"; then
    INSTALLED_NX=$(dpkg -l nomachine | awk '/^ii/{print $3}')
    if [[ "$INSTALLED_NX" == "${NX_VERSION}"* ]]; then
        ok "NoMachine ${NX_VERSION} is already installed — skipping."
    else
        warn "NoMachine ${INSTALLED_NX} is installed; this script pins ${NX_VERSION}."
        read -rp "  Upgrade to ${NX_VERSION}? [y/N]: " UPGRADE_NX
        if [[ "${UPGRADE_NX,,}" == "y" ]]; then
            info "Removing old version..."
            sudo dpkg -r nomachine || true
            _install_nomachine
        else
            ok "Keeping NoMachine ${INSTALLED_NX}."
        fi
    fi
else
    _install_nomachine
fi

# Ensure the NoMachine daemon is running
if sudo /usr/NX/bin/nxserver --status 2>/dev/null | grep -qi "running"; then
    ok "NoMachine server is running on port 4000 (NX protocol)."
else
    info "Starting NoMachine server..."
    sudo /usr/NX/bin/nxserver --startup 2>/dev/null || true
    ok "NoMachine server started on port 4000."
fi

echo ""

# =============================================================================
# STEP 3 — WIFI NETWORK
# =============================================================================
echo -e "${BLUE}── Step 3 / 3 : WiFi Network ────────────────────────────────${NC}"
echo ""
echo "  The Jetson uses NetworkManager to manage WiFi. Each saved network"
echo "  has a priority number — when multiple known networks are in range,"
echo "  it connects to the one with the highest number automatically."
echo ""
echo "  You can run this script again to add a second network (e.g. a field"
echo "  hotspot). Assign it a lower priority (e.g. 50) so the Jetson still"
echo "  prefers the main network when both are in range."
echo ""

command -v nmcli &>/dev/null \
    || die "nmcli not found. Install NetworkManager: sudo apt install network-manager"

read -rp "  WiFi SSID to add [default: Sahand]: " WIFI_SSID
WIFI_SSID="${WIFI_SSID:-Sahand}"

read -rsp "  Password for '${WIFI_SSID}': " WIFI_PASS
echo ""

read -rp "  Priority [default: 100, higher number = more preferred]: " WIFI_PRIO
WIFI_PRIO="${WIFI_PRIO:-100}"

echo ""

if nmcli con show "$WIFI_SSID" &>/dev/null; then
    warn "A saved connection named '${WIFI_SSID}' already exists — updating it."
    sudo nmcli con mod "$WIFI_SSID" \
        wifi-sec.psk "$WIFI_PASS" \
        connection.autoconnect-priority "$WIFI_PRIO"
    ok "Connection '${WIFI_SSID}' updated (priority: ${WIFI_PRIO})."
else
    sudo nmcli con add \
        type wifi \
        con-name "$WIFI_SSID" \
        ssid "$WIFI_SSID" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$WIFI_PASS" \
        connection.autoconnect yes \
        connection.autoconnect-priority "$WIFI_PRIO"
    ok "WiFi network '${WIFI_SSID}' saved (priority: ${WIFI_PRIO})."
fi

# =============================================================================
# SUMMARY
# =============================================================================
TAILSCALE_IP_FINAL=$(tailscale ip -4 2>/dev/null || echo "<run 'tailscale ip -4'>")

echo ""
echo "========================================================"
echo "  Setup complete!"
echo "========================================================"
echo ""
printf "  %-22s %s\n" "Tailscale IP:"       "$TAILSCALE_IP_FINAL"
printf "  %-22s %s\n" "NoMachine host:"     "$TAILSCALE_IP_FINAL"
printf "  %-22s %s\n" "NoMachine port:"     "4000"
printf "  %-22s %s\n" "NoMachine protocol:" "NX"
printf "  %-22s %s\n" "WiFi network:"       "'${WIFI_SSID}' (priority ${WIFI_PRIO})"
echo ""
echo "  ── Connect via NoMachine ─────────────────────────────"
echo "  1. Install NoMachine on your laptop: https://www.nomachine.com"
echo "  2. Add a new connection: host ${TAILSCALE_IP_FINAL}, port 4000, protocol NX"
echo "  3. Log in with the Jetson's Ubuntu username and password"
echo ""
echo "  ── Add another WiFi network ──────────────────────────"
echo "  Run this script again, or use nmcli directly:"
echo "    sudo nmcli con add type wifi con-name \"<SSID>\" ssid \"<SSID>\" \\"
echo "      wifi-sec.key-mgmt wpa-psk wifi-sec.psk \"<password>\" \\"
echo "      connection.autoconnect yes connection.autoconnect-priority <N>"
echo ""
echo "  ── List saved networks and priorities ────────────────"
echo "    nmcli -f NAME,DEVICE,AUTOCONNECT-PRIORITY con show"
echo ""
