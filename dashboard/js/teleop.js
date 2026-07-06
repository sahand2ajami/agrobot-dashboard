/* Shared teleoperation transport — the ONLY code that may POST /api/cmd_vel.
 *
 * Both dashboard pages (index.html, plc_combined.html) load this file, so the
 * safety clamp exists in exactly one place. It clamps every command to the
 * chassis hard ceilings that the page stores on window.CFG.hardMaxLinear /
 * .hardMaxAngular (populated from GET /api/config) — a command past those
 * limits would be rejected by the server with a 400 and, worse, a page that
 * forgot the clamp could drift from the chassis limits (that drift shipped
 * once; this file is the fix).
 *
 * Pages may provide optional callbacks on window.TELEOP_HOOKS:
 *   onServerStatus(ok)   — connectivity indicator updates
 *   onReject(lin, ang)   — server rejected the command with 400
 *   onTimeout()          — request timed out / network lost
 * Errors are swallowed after the hooks run: teleop senders are fire-and-forget
 * at 20-50 Hz and the 0.5 s server-side deadman is the real safety net.
 */
'use strict';

const FETCH_TIMEOUT_MS = 3000;  // applies to all sensor API calls

function _fetchWithTimeout(url, opts, timeoutMs) {
  const ctrl  = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeoutMs ?? FETCH_TIMEOUT_MS);
  return fetch(url, { ...opts, signal: ctrl.signal })
    .finally(() => clearTimeout(timer));
}

function publishCmdVel(lin, ang) {
  const cfg   = window.CFG || {};
  const hooks = window.TELEOP_HOOKS || {};
  if (cfg.hardMaxLinear  != null) lin = Math.max(-cfg.hardMaxLinear,  Math.min(cfg.hardMaxLinear,  lin));
  if (cfg.hardMaxAngular != null) ang = Math.max(-cfg.hardMaxAngular, Math.min(cfg.hardMaxAngular, ang));
  return _fetchWithTimeout('/api/cmd_vel', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ linear_x: lin, angular_z: ang }),
  }).then(r => {
    if (hooks.onServerStatus) hooks.onServerStatus(r.ok || r.status < 500);
    if (r.status === 400 && hooks.onReject) hooks.onReject(lin, ang);
    return r;
  }).catch(() => {
    if (hooks.onServerStatus) hooks.onServerStatus(false);
    if (hooks.onTimeout) hooks.onTimeout();
  });
}

function publishStop() { return publishCmdVel(0, 0); }
