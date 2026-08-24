"""Cloud uploads (HTTP ingest for planted-seedling records).

Both the endpoint and the write key MUST come from the environment — there is
no built-in default endpoint, so uploads are OFF until AGROBOT_INGEST_URL and
AGROBOT_INGEST_KEY are both set. Records are always written locally regardless.
"""
import json
import logging
import os
import urllib.error
import urllib.request

from agrobot_dashboard.services.events import log_event

log = logging.getLogger("dashboard")

_INGEST_URL = os.environ.get("AGROBOT_INGEST_URL", "")
_INGEST_KEY = os.environ.get("AGROBOT_INGEST_KEY", "")


def push_seedling(entry: dict):
    """POST a planted-seedling record to the configured ingest endpoint.
    Run in a daemon thread so it never blocks the HTTP response."""
    if not (_INGEST_URL and _INGEST_KEY):
        _missing = "AGROBOT_INGEST_URL" if not _INGEST_URL else "AGROBOT_INGEST_KEY"
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload skipped — {_missing} is not set",
                  f"Export {_missing} before launching the dashboard. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=3600)
        return
    payload = json.dumps({"source": "robot", "records": [entry]}).encode()
    req = urllib.request.Request(
        _INGEST_URL,
        data=payload,
        headers={
            "Content-Type": "application/json",
            "x-agrobot-key":   _INGEST_KEY,
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode()
            log.info("[seedling] ingest OK: %s", body)
    except urllib.error.HTTPError as exc:
        log.warning("[seedling] ingest HTTP %s: %s", exc.code, exc.read().decode())
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload failed (HTTP {exc.code})",
                  "Check internet connection and the AGROBOT_INGEST_URL / AGROBOT_INGEST_KEY "
                  "environment variables. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=60)
    except Exception as exc:
        log.warning("[seedling] ingest failed: %s", exc)
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload error: {exc}",
                  "Check internet connection. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=60)
