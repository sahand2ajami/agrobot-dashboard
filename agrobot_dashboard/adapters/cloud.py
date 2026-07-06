"""Cloud uploads (Supabase ingest for planted-seedling records).

The write key MUST come from the environment — a previous revision committed
it to source, so that key is considered leaked and needs rotation on the
Supabase side.
"""
import json
import logging
import os
import urllib.error
import urllib.request

from agrobot_dashboard.services.events import log_event

log = logging.getLogger("dashboard")

_SUPABASE_INGEST_URL = os.environ.get(
    "AGROBOT_SUPABASE_URL",
    "https://ingest.invalid/functions/v1/ingest")
_SUPABASE_AGROBOT_KEY   = os.environ.get("AGROBOT_SUPABASE_KEY", "")


def push_seedling(entry: dict):
    """POST a planted-seedling record to the Supabase ingest endpoint.
    Run in a daemon thread so it never blocks the HTTP response."""
    if not _SUPABASE_AGROBOT_KEY:
        log_event("WARN", "GNSS",
                  "Seedling cloud upload skipped — AGROBOT_SUPABASE_KEY is not set",
                  "Export AGROBOT_SUPABASE_KEY before launching the dashboard. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=3600)
        return
    payload = json.dumps({"source": "robot", "records": [entry]}).encode()
    req = urllib.request.Request(
        _SUPABASE_INGEST_URL,
        data=payload,
        headers={
            "Content-Type": "application/json",
            "x-agrobot-key":   _SUPABASE_AGROBOT_KEY,
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode()
            log.info("[seedling] Supabase ingest OK: %s", body)
    except urllib.error.HTTPError as exc:
        log.warning("[seedling] Supabase ingest HTTP %s: %s", exc.code, exc.read().decode())
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload failed (HTTP {exc.code})",
                  "Check internet connection and the AGROBOT_SUPABASE_KEY environment variable. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=60)
    except Exception as exc:
        log.warning("[seedling] Supabase ingest failed: %s", exc)
        log_event("WARN", "GNSS",
                  f"Seedling cloud upload error: {exc}",
                  "Check internet connection. "
                  "The record is saved locally in logs/planted_seedlings/seedlings.jsonl.",
                  _key="seedling-push", _debounce_s=60)
