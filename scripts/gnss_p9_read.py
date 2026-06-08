#!/usr/bin/env python3
"""
Columbus P-9 Race GNSS reader.
Parses NMEA 0183 from USB serial, prints live terminal display,
and writes /tmp/gnss_coords.json for the map viewer.
"""
import json
import os
import sys
import time
from datetime import datetime, timezone

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

COORDS_FILE = "/tmp/gnss_coords.json"
LOG_DIR_DEFAULT = os.path.expanduser("~/jackal_outdoor/logs/gnss")
BAUD_RATES = [115200, 9600, 4800, 38400]
CANDIDATE_PORTS = [
    "/dev/gnss",  # stable udev symlink (preferred)
    "/dev/ttyACM0", "/dev/ttyACM1",
    "/dev/ttyUSB0", "/dev/ttyUSB1",
]

FIX_LABELS = {
    0: "No Fix",
    1: "GPS",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "Dead Reckoning",
}


def find_device(hint=None):
    if hint:
        return hint
    for port in CANDIDATE_PORTS:
        if os.path.exists(port):
            return port
    return None


def nmea_to_decimal(value, direction):
    """NMEA DDMM.MMMM → decimal degrees."""
    if not value:
        return None
    dot = value.index(".")
    degrees = float(value[: dot - 2])
    minutes = float(value[dot - 2 :])
    dec = degrees + minutes / 60.0
    if direction in ("S", "W"):
        dec = -dec
    return dec


def validate_checksum(raw: str) -> bool:
    """Return True if the NMEA XOR checksum is correct (or absent)."""
    if "*" not in raw:
        return True  # no checksum field — accept
    try:
        body, tail = raw[1:].split("*", 1)  # strip leading $
        expected = int(tail[:2], 16)
        computed = 0
        for ch in body:
            computed ^= ord(ch)
        return computed == expected
    except (ValueError, IndexError):
        return False


def strip_checksum(sentence):
    if "*" in sentence:
        return sentence[: sentence.index("*")]
    return sentence


def parse_gngga(parts):
    if len(parts) < 10:
        return {}
    try:
        lat = nmea_to_decimal(parts[2], parts[3])
        lon = nmea_to_decimal(parts[4], parts[5])
        fix = int(parts[6]) if parts[6] else 0
        sats = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 99.9
        alt = float(parts[9]) if parts[9] else 0.0
        if lat is None or lon is None:
            return {}
        return {"lat": lat, "lon": lon, "fix": fix, "sats": sats, "hdop": hdop, "alt": alt}
    except (ValueError, IndexError):
        return {}


def parse_gnrmc(parts):
    if len(parts) < 8:
        return {}
    try:
        active = parts[2] == "A"
        speed_kmh = float(parts[7]) * 1.852 if parts[7] else 0.0
        heading = float(parts[8]) if len(parts) > 8 and parts[8] else 0.0
        return {"active": active, "speed_kmh": speed_kmh, "heading": heading}
    except (ValueError, IndexError):
        return {}


def parse_gsv(parts):
    """Return the 'satellites in view' count from a GSV sentence, or None.
    $xxGSV,total_msgs,msg_num,sats_in_view,...  (field 3 is the in-view count)."""
    if len(parts) < 4:
        return None
    try:
        return int(parts[3])
    except (ValueError, IndexError):
        return None


def open_serial(port, baud_rates):
    for baud in baud_rates:
        try:
            ser = serial.Serial(port, baud, timeout=2)
            # Validate we get NMEA
            for _ in range(5):
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if line.startswith("$G") or line.startswith("$P"):
                    return ser, baud
            ser.close()
        except serial.SerialException:
            continue
    return None, None


def render(state, port, baud, log_path):
    lat = f"{state['lat']:+.6f}°" if state.get("lat") is not None else "  waiting..."
    lon = f"{state['lon']:+.6f}°" if state.get("lon") is not None else "  waiting..."
    fix = FIX_LABELS.get(state.get("fix", 0), "Unknown")
    sats = state.get("sats", 0)
    hdop = state.get("hdop", 99.9)
    alt = state.get("alt", 0.0)
    speed = state.get("speed_kmh", 0.0)
    heading = state.get("heading", 0.0)
    active = "Active" if state.get("active") else "Void"
    ts = datetime.now(timezone.utc).strftime("%H:%M:%S UTC")

    print("\033[H\033[J", end="")  # clear screen, cursor home
    print(f"  Columbus P-9 Race  |  {port}  |  {baud} baud  |  {ts}")
    print(f"  {'─' * 50}")
    print(f"  Fix       : {fix}  ({active})")
    print(f"  Latitude  : {lat}")
    print(f"  Longitude : {lon}")
    print(f"  Altitude  : {alt:.1f} m")
    print(f"  Satellites: {sats}    HDOP: {hdop:.1f}")
    print(f"  Speed     : {speed:.1f} km/h   Heading: {heading:.1f}°")
    print(f"  {'─' * 50}")
    print(f"  Log  → {log_path}")
    print(f"  Map  → http://localhost:8765/gnss_map.html")
    print(f"\n  Ctrl-C to quit")


def open_log(log_dir):
    os.makedirs(log_dir, exist_ok=True)
    filename = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S_gnss.jsonl")
    path = os.path.join(log_dir, filename)
    return open(path, "a", buffering=1), path  # line-buffered


def build_payload(state, has_fix):
    """Assemble the JSON payload written to COORDS_FILE (and the log on a fix)."""
    ts = datetime.now(timezone.utc).isoformat()
    if has_fix:
        return {
            "lat": state["lat"],
            "lon": state["lon"],
            "fix": state.get("fix", 0),
            "fix_label": FIX_LABELS.get(state.get("fix", 0), "Unknown"),
            "sats": state.get("sats", 0),
            "sats_in_view": state.get("sats_in_view", 0),
            "hdop": state.get("hdop", 99.9),
            "alt": state.get("alt", 0.0),
            "speed_kmh": state.get("speed_kmh", 0.0),
            "heading": state.get("heading", 0.0),
            "has_fix": True,
            "ts": ts,
        }
    # Heartbeat — no position. Lets the dashboard show "connected — no signal"
    # and the live satellite counts.  Deliberately omits lat/lon.
    return {
        "fix": state.get("live_fix", 0),
        "fix_label": FIX_LABELS.get(state.get("live_fix", 0), "No Fix"),
        "sats": state.get("live_sats", 0),
        "sats_in_view": state.get("sats_in_view", 0),
        "has_fix": False,
        "ts": ts,
    }


def _write_coords(payload):
    try:
        tmp = COORDS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.replace(tmp, COORDS_FILE)
    except OSError:
        pass


def _read_loop(ser, port, baud, log_file, log_path, state):
    """Inner read loop. Raises serial.SerialException on disconnect."""
    last_render = 0.0
    last_write = 0.0
    while True:
        raw = ser.readline().decode("ascii", errors="ignore").strip()

        if not raw.startswith("$"):
            continue

        if not validate_checksum(raw):
            continue  # discard sentences with bad checksums silently

        sentence = strip_checksum(raw)
        parts = sentence.split(",")
        tag = parts[0][1:]  # strip leading $

        if tag in ("GNGGA", "GPGGA"):
            state.update(parse_gngga(parts))
            # Track fix-quality + sats-used even without a position, so we can
            # tell "connected, no fix" apart from "disconnected".
            try:
                state["live_fix"]  = int(parts[6]) if len(parts) > 6 and parts[6] else 0
                state["live_sats"] = int(parts[7]) if len(parts) > 7 and parts[7] else 0
            except (ValueError, IndexError):
                pass
        elif tag in ("GNRMC", "GPRMC"):
            state.update(parse_gnrmc(parts))
        elif tag.endswith("GSV"):
            n = parse_gsv(parts)
            if n is not None:
                # Sum in-view counts across constellations (GP/GL/GA/GB/GQ...).
                state.setdefault("gsv", {})[tag[:2]] = n
                state["sats_in_view"] = sum(state["gsv"].values())

        now = time.monotonic()
        has_fix = state.get("live_fix", 0) >= 1 and state.get("lat") is not None

        if has_fix:
            if now - last_write >= 0.2:           # throttle fix writes to ~5 Hz
                payload = build_payload(state, True)
                _write_coords(payload)
                try:
                    log_file.write(json.dumps(payload) + "\n")   # log real fixes only
                except OSError:
                    pass
                last_write = now
        elif now - last_write >= 1.0:             # heartbeat once per second
            _write_coords(build_payload(state, False))
            last_write = now

        if now - last_render >= 0.2:
            render(state, port, baud, log_path)
            last_render = now


def main():
    port_hint = sys.argv[1] if len(sys.argv) > 1 else None
    log_dir = sys.argv[2] if len(sys.argv) > 2 else LOG_DIR_DEFAULT
    port = find_device(port_hint)
    if not port:
        print("ERROR: No GNSS device found. Plug in the P-9 Race via USB.")
        print("       Or specify the port: gnss_p9_read.py /dev/ttyACM0")
        sys.exit(1)

    log_file, log_path = open_log(log_dir)
    print(f"  Logging to: {log_path}\n")

    state = {}
    retry_delay = 2.0  # seconds between reconnection attempts

    try:
        while True:
            print(f"Probing {port} ...")
            ser, baud = open_serial(port, BAUD_RATES)
            if ser is None:
                print(f"  Could not read NMEA from {port} — retrying in {retry_delay:.0f}s ...")
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, 30.0)
                continue

            retry_delay = 2.0  # reset on successful connect
            print(f"  Connected at {baud} baud. Reading GNSS data ...")

            try:
                _read_loop(ser, port, baud, log_file, log_path, state)
            except serial.SerialException as e:
                print(f"\n  GNSS serial error: {e} — reconnecting ...")
            except OSError as e:
                print(f"\n  GNSS I/O error: {e} — reconnecting ...")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

            time.sleep(retry_delay)

    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()
        print(f"\nDisconnected. Log saved: {log_path}")


if __name__ == "__main__":
    main()
