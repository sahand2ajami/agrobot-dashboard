# Jetson Network Latency Report

**Generated:** Mon Jun 8 01:45 PM EDT 2026
**Host:** NVIDIA Jetson (Tegra)
**Purpose:** Characterize the new wired (TP-Link) connection path vs. the existing WiFi path.

---

## 1. Topology in use

```
Laptop (192.168.0.225, WiFi)
        |
        v  WiFi
  TP-Link router (192.168.0.1)
        |
        v  LAN cable
   Network switch
        |
        v  LAN cable (Gigabit)
   Jetson eno1 (192.168.0.50)   <-- NoMachine server :4000

  Jetson wlP1p1s0 (10.172.192.44)  <-- WiFi "Sahand", provides INTERNET only
```

- **Active NoMachine session:** `192.168.0.50:4000  <-  192.168.0.225:62654`
  → confirmed coming in over the **wired / TP-Link path**.
- **eno1 link speed:** 1000 Mbps (Gigabit), full duplex.

---

## 2. Latency measurements

| Path | Target | Samples | Min | **Avg** | Max | Jitter (mdev) | Loss |
|------|--------|:------:|:---:|:------:|:---:|:------:|:----:|
| **Wired** (Jetson → TP-Link router) | `192.168.0.1` | 20 | 0.344 ms | **0.822 ms** | 0.961 ms | 0.116 ms | 0% |
| **WiFi** (Jetson → Sahand gateway) | `10.172.192.34` | 20 | 2.252 ms | **5.967 ms** | 16.043 ms | 3.649 ms | 0% |
| **Internet** (via WiFi) | `8.8.8.8` | 10 | 12.114 ms | **26.781 ms** | 42.724 ms | 8.935 ms | 0% |

> Note: a direct ping to the laptop (`192.168.0.225`) returned 100% loss. This is the
> **laptop's firewall blocking ICMP** (default on Windows/macOS), *not* a connectivity
> problem — the NoMachine TCP session to it is established and healthy.

---

## 3. Interpretation

**The wired path is dramatically better than WiFi on every metric that matters for remote desktop:**

- **Latency:** 0.82 ms vs 5.97 ms — the wired link is ~**7x lower** latency to the first hop.
- **Jitter (consistency):** 0.116 ms vs 3.649 ms — the wired link is ~**30x more stable**.
  Jitter is what makes a remote desktop feel "laggy/jumpy"; the wired side is rock-solid.
- **Worst case:** wired max 0.96 ms vs WiFi max 16 ms — no latency spikes on the wire.
- **Packet loss:** 0% on all local paths.

**Where the bottleneck now lives:** the only remaining variable link is the **laptop ↔ TP-Link
WiFi hop**. The Jetson half of the connection (switch → Jetson) is sub-millisecond Gigabit and
is no longer a limiting factor.

**Internet dependency:** the NoMachine path (`192.168.0.x`) is fully local and does **not**
traverse the internet. If the "Sahand" WiFi/internet drops, this connection stays up; only the
Jetson's own internet access is affected.

---

## 4. Bottom line

- The connection to the Jetson is now **low-latency, low-jitter, and internet-independent.**
- The wired segment performs essentially at the physical limit (~0.8 ms, Gigabit).
- To improve the remaining hop, connect the **laptop via Ethernet to the switch** as well —
  that would put both ends on sub-millisecond Gigabit and beat any WiFi.

---

## 5. How the data was collected

```bash
# Wired path
ping -c20 -i0.2 192.168.0.1

# WiFi path
ping -c20 -i0.2 10.172.192.34

# Internet via WiFi
ping -c10 -i0.2 8.8.8.8

# Confirm session path
sudo ss -tnp | grep ':4000' | grep ESTAB
```
