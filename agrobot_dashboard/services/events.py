"""Browser-visible event log.

A thread-safe ring buffer of structured events served via GET
/api/events?since=<unix_ts>. Every subsystem (cameras, PLC, ROS, GNSS)
reports operator-relevant conditions here with an actionable suggestion.
"""
import collections
import logging
import threading
import time

log = logging.getLogger("dashboard")

_event_log      = collections.deque(maxlen=500)
_event_log_lock = threading.Lock()
# Debounce: suppress identical (source, key) events within a time window so
# a permanently-down PLC or disconnected camera doesn't flood the log.
_event_debounce: dict = {}   # (source, key) → monotonic time of last emission


def log_event(level: str, source: str, message: str,
              suggestion: str = "", _key: str = "", _debounce_s: float = 0.0):
    """Append a structured event to the browser-visible ring buffer.

    level      : 'INFO' | 'WARN' | 'ERROR'
    source     : subsystem label ('PLC', 'Camera', 'GNSS', 'ROS', 'System', 'Network')
    message    : human-readable description
    suggestion : actionable fix hint shown in the log panel
    _key       : debounce key; if set with _debounce_s > 0, identical events are
                 suppressed within that many seconds
    """
    if _key and _debounce_s > 0:
        now = time.monotonic()
        dk  = (source, _key)
        # Called from capture threads, ROS callbacks and HTTP request threads
        # concurrently — the check-then-set must be atomic.
        with _event_log_lock:
            if now - _event_debounce.get(dk, 0.0) < _debounce_s:
                return
            _event_debounce[dk] = now
    entry = {
        "ts":         time.time(),
        "level":      level.upper(),
        "source":     source,
        "message":    message,
        "suggestion": suggestion,
    }
    with _event_log_lock:
        _event_log.append(entry)
    lvl = level.upper()
    if lvl == "ERROR":
        log.error("[%s] %s", source, message)
    elif lvl == "WARN":
        log.warning("[%s] %s", source, message)
    else:
        log.info("[%s] %s", source, message)


def events_since(since_ts: float):
    """All events newer than `since_ts` (unix time; 0 = everything)."""
    with _event_log_lock:
        return [e for e in _event_log if e['ts'] > since_ts]
