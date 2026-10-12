#!/usr/bin/env python3
"""
Live CSI dashboard: UDP listener + web visualizer in one process.

Protocol (binary v2, matches your ESP32 receiver):
    'C' | ver(1) | mac[6] | rssi(i8) | noise(i8) | sig_mode(u8) | rate(u8) |
    mcs(u8) | cwb(u8) | channel(u8) | secondary(u8) | len(u16 LE) | data[len] (i8)

Scales to any number of illuminators automatically -- each source MAC
becomes its own row in the dashboard.

Usage:
    python app.py --port 1201 --web-port 5000
"""
import argparse
import json
import os
import socket
import struct
import threading
import time
from collections import deque, defaultdict

from flask import Flask, render_template
from flask_socketio import SocketIO

HDR = 18
WINDOW_FRAMES = 50          # ~1s at ~50Hz, per your validated motion-window size
HEATMAP_HISTORY = 200       # columns of amplitude history kept per illuminator
EMIT_HZ = 10                # throttle UI updates regardless of packet rate

# ---------------------------------------------------------------------------
# Room geometry  (SW corner = origin, x = East, y = North, all in metres)
# Defaults: your 9ft x 16ft room. Updated live via set_room socket event.
# ---------------------------------------------------------------------------
ROOM_W = 2.743   # East-West  (9 ft)
ROOM_H = 4.877   # North-South (16 ft)

# MAC -> (x, y) for each node.  Fill in the two illuminator MACs once known;
# until then the server falls back to "no localization" gracefully.
# The receiver position is needed for bistatic geometry but doesn't produce CSI.
RX_POS = (1.524, 3.048)   # 5 ft from West, 10 ft from South

# Map illuminator MAC -> its (x,y) position.  These are pre-filled from your
# measurements; you can also update them live via the "set_layout" socket event.
NODE_POSITIONS = {
    # "B4:3A:45:3F:A2:D0": (4.369, 0.800),   # TX1 — fill your real MACs here
    # "FC:01:2C:2E:95:18": (0.610, 0.000),   # TX2
}

# CoV-excess -> path-elongation (metres) scale factor. This links a
# dimensionless motion statistic to physical RF path perturbation, which
# depends on propagation physics (wavelength, reflection strength) and can't
# be derived from node geometry alone -- it must be measured (see
# "calibrate_scale" socket event) by standing at a known point and comparing
# the true geometric elongation to the observed CoV excess. Persisted to
# calibration.json; this is only the fallback before any measurement exists.
ELONGATION_SCALE = 5.0

import math

def _dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def _ellipse_points(tx, rx, n=120):
    """
    Return (x,y) points on the bistatic ellipse whose foci are tx and rx
    and whose sum-of-distances equals the baseline path length d0.
    We derive d0 from the direct TX->RX distance (line-of-sight).
    A person perturbing the path elongates it by delta; we use delta
    proportional to the CoV excess above baseline, scaled by a tunable
    factor.  For visualisation we draw the base ellipse (delta=0) as a
    thin guide and the perturbed ellipse as the presence indicator.
    """
    d0 = _dist(tx, rx)       # direct path = 2a minimum (person on the line)
    cx = (tx[0]+rx[0])/2
    cy = (tx[1]+rx[1])/2
    angle = math.atan2(rx[1]-tx[1], rx[0]-tx[0])
    c = d0/2                  # half focal distance
    pts = []
    for i in range(n+1):
        t = 2*math.pi*i/n
        # parametric ellipse in local frame, a=c (degenerate line at delta=0)
        # caller passes a = c + delta/2
        pts.append((math.cos(t), math.sin(t)))
    return pts, cx, cy, angle, c

def bistatic_ellipse(tx, rx, a):
    """
    Return list of (x,y) world-space points for a bistatic ellipse with
    foci at tx, rx and semi-major axis a.
    a must be >= dist(tx,rx)/2 = c.
    """
    cx = (tx[0]+rx[0])/2
    cy = (tx[1]+rx[1])/2
    angle = math.atan2(rx[1]-tx[1], rx[0]-tx[0])
    c = _dist(tx, rx)/2
    if a < c:
        a = c + 0.01
    b = math.sqrt(max(0, a**2 - c**2))
    pts = []
    n = 120
    for i in range(n+1):
        t = 2*math.pi*i/n
        lx = a * math.cos(t)
        ly = b * math.sin(t)
        # rotate by angle
        wx = cx + lx*math.cos(angle) - ly*math.sin(angle)
        wy = cy + lx*math.sin(angle) + ly*math.cos(angle)
        pts.append([round(wx,4), round(wy,4)])
    return pts

def ellipse_grid_score(tx, rx, a, grid_pts, sigma):
    """
    For each grid point return how close it is to the ellipse surface.
    Score = exp(-dist_to_ellipse^2 / sigma^2). sigma is the arc thickness in
    metres -- pass a room-scaled value (see compute_localization) so this
    isn't tuned to one fixed room size.
    """
    sigma2 = sigma * sigma
    scores = []
    c = _dist(tx, rx)/2
    if a < c:
        a = c + 0.01
    for (gx, gy) in grid_pts:
        d_sum = _dist((gx,gy), tx) + _dist((gx,gy), rx)
        diff = abs(d_sum - 2*a)
        scores.append(math.exp(-diff**2/sigma2))
    return scores

def refine_intersection(p0, links, rx, room_w, room_h, iters=150):
    """
    The grid heatmap's arg-max is a coarse (one grid cell wide) score peak,
    not the actual ellipse intersection -- it never solves the
    "two links, one point" geometry the localization is supposed to be
    doing. This refines p0 via gradient descent on the sum of squared
    ellipse-equation residuals (dist(tx,p)+dist(p,rx) - 2a) across all
    active links, giving a genuine joint intersection estimate. With >=3
    links this also resolves the two-ellipse mirror ambiguity, since only
    one point can satisfy every link's equation at once.
    Returns (x, y), rms_residual_m.
    """
    def cost(p):
        total = 0.0
        for tx, a in links:
            r = (_dist(p, tx) + _dist(p, rx)) - 2 * a
            total += r * r
        return total

    x, y = p0
    h = 1e-4
    step = 0.2  # metres, shrinks on stall
    c0 = cost((x, y))
    for _ in range(iters):
        gx = (cost((x + h, y)) - c0) / h
        gy = (cost((x, y + h)) - c0) / h
        glen = math.hypot(gx, gy)
        if glen < 1e-9:
            break
        nx = min(max(x - step * gx / glen, 0.0), room_w)
        ny = min(max(y - step * gy / glen, 0.0), room_h)
        c1 = cost((nx, ny))
        if c1 < c0:
            x, y, c0 = nx, ny, c1
        else:
            step *= 0.5
            if step < 1e-4:
                break
    return (x, y), math.sqrt(c0 / max(1, len(links)))


def compute_localization(mac_covs, mac_baselines):
    """
    Given per-MAC {mac: cur_cov} and {mac: baseline_dict},
    compute a presence heatmap over the room grid and candidate blob position.

    Returns dict with keys:
      grid_n_x, grid_n_y -- grid dimensions (room-size-adaptive, see below)
      scores             -- flat list of floats [0,1], row-major (grid_n_x per row)
      blob               -- {x,y,confidence} or None
      ellipses           -- list of {tx, rx, pts} for drawing
      nodes              -- list of {label, x, y, type}
    """
    # Grid resolution scales with room size instead of a fixed 30x30 point
    # count -- a fixed count makes cells coarser as the room grows (and
    # wastes resolution on a small room). TARGET_CELL_M reproduces the
    # ~9.5cm cells the localization was validated with in the original
    # 2.743x4.877m room; MIN/MAX_GRID_N bound both ends (a tiny room still
    # gets a usable grid, a huge room can't blow up per-tick compute at
    # EMIT_HZ).
    TARGET_CELL_M = 0.095
    MIN_GRID_N, MAX_GRID_N = 15, 60
    grid_n_x = min(MAX_GRID_N, max(MIN_GRID_N, round(ROOM_W / TARGET_CELL_M) + 1))
    grid_n_y = min(MAX_GRID_N, max(MIN_GRID_N, round(ROOM_H / TARGET_CELL_M) + 1))

    # Ellipse arc thickness scales with room size instead of a value fixed
    # to the room this was first tuned in -- otherwise a much larger room
    # is undersensitive (arc too thin relative to the space) and a much
    # smaller one oversensitive (arc too thick). Fraction reproduces the
    # originally-tuned 0.3m arc at this room's 2.743m short side.
    SIGMA_FRACTION = 0.1094
    sigma = SIGMA_FRACTION * min(ROOM_W, ROOM_H)

    # build grid points (row-major: grid_n_x columns per row)
    grid_pts = []
    for gy in range(grid_n_y):
        for gx in range(grid_n_x):
            wx = ROOM_W * gx / (grid_n_x-1)
            wy = ROOM_H * gy / (grid_n_y-1)
            grid_pts.append((wx, wy))

    combined = [0.0] * (grid_n_x * grid_n_y)
    ellipses = []
    links = []          # (tx_pos, a) per active link, for exact intersection refinement
    active_links = 0

    for mac, tx_pos in NODE_POSITIONS.items():
        if mac not in mac_covs:
            continue
        base = mac_baselines.get(mac)
        if base is None:
            continue
        cur_cov = mac_covs[mac]
        threshold = base["mean"] + 3*base["std"]
        excess = max(0.0, cur_cov - threshold)

        # semi-major axis: direct path + excess elongation
        c = _dist(tx_pos, RX_POS)/2
        delta = excess * ELONGATION_SCALE
        a = c + delta + 0.01   # +0.01 so ellipse is never degenerate

        scores = ellipse_grid_score(tx_pos, RX_POS, a, grid_pts, sigma)
        for i, s in enumerate(scores):
            combined[i] += s
        active_links += 1
        links.append((tx_pos, a))

        pts = bistatic_ellipse(tx_pos, RX_POS, a)
        ellipses.append({"tx": list(tx_pos), "rx": list(RX_POS), "pts": pts,
                          "excess": round(excess, 4)})

    # normalise
    if active_links > 0:
        max_s = max(combined) or 1.0
        combined = [round(s/max_s, 4) for s in combined]

    # find blob: coarse grid peak, refined to the actual ellipse intersection
    blob = None
    if active_links >= 2 and max(combined) > 0.3:
        peak_idx = combined.index(max(combined))
        gx = peak_idx % grid_n_x
        gy = peak_idx // grid_n_x
        bx = ROOM_W * gx / (grid_n_x-1)
        by = ROOM_H * gy / (grid_n_y-1)
        (rx_x, rx_y), rms = refine_intersection((bx, by), links, RX_POS, ROOM_W, ROOM_H)
        blob = {"x": round(rx_x,3), "y": round(rx_y,3),
                "confidence": round(combined[peak_idx],3),
                "residual_m": round(rms,4)}
    elif active_links == 1 and max(combined) > 0.3:
        # single link: show ellipse arc as blob hint, no point estimate
        blob = None

    # node list for drawing
    nodes = [{"label": "RX", "x": round(RX_POS[0],3), "y": round(RX_POS[1],3), "type": "rx"}]
    for mac, pos in NODE_POSITIONS.items():
        nodes.append({"label": mac[-5:], "x": round(pos[0],3), "y": round(pos[1],3), "type": "tx"})

    return {
        "room_w": ROOM_W,
        "room_h": ROOM_H,
        "grid_n_x": grid_n_x,
        "grid_n_y": grid_n_y,
        "scores": combined,
        "blob": blob,
        "ellipses": ellipses,
        "nodes": nodes,
        "active_links": active_links,
    }

app = Flask(__name__)
app.config["SECRET_KEY"] = "csi-dashboard"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

lock = threading.Lock()

BASELINE_FILE    = os.path.join(os.path.dirname(__file__), "baseline.json")
LAYOUT_FILE      = os.path.join(os.path.dirname(__file__), "layout.json")
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")

def save_baseline():
    """Write current baseline dict to JSON. Called under lock."""
    try:
        with open(BASELINE_FILE, "w") as f:
            json.dump(baseline, f, indent=2)
        print(f"[baseline] saved to {BASELINE_FILE}", flush=True)
    except Exception as e:
        print(f"[baseline] save failed: {e}", flush=True)
    sync_active_profile()

def load_baseline():
    """Load baseline from JSON on startup. Returns dict or {}."""
    if not os.path.exists(BASELINE_FILE):
        return {}
    try:
        with open(BASELINE_FILE) as f:
            data = json.load(f)
        print(f"[baseline] loaded {len(data)} MAC(s) from {BASELINE_FILE}", flush=True)
        return data
    except Exception as e:
        print(f"[baseline] load failed: {e}", flush=True)
        return {}

def save_layout():
    """Write NODE_POSITIONS, RX_POS and room dimensions to JSON. Called under lock."""
    try:
        data = {
            "room_w": ROOM_W,
            "room_h": ROOM_H,
            "rx": list(RX_POS),
            "nodes": {mac: list(pos) for mac, pos in NODE_POSITIONS.items()},
        }
        with open(LAYOUT_FILE, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[layout] saved to {LAYOUT_FILE}", flush=True)
    except Exception as e:
        print(f"[layout] save failed: {e}", flush=True)
    sync_active_profile()

def load_layout():
    """Load node positions and room dimensions from JSON on startup."""
    global RX_POS, ROOM_W, ROOM_H
    if not os.path.exists(LAYOUT_FILE):
        return
    try:
        with open(LAYOUT_FILE) as f:
            data = json.load(f)
        if "room_w" in data:
            ROOM_W = float(data["room_w"])
        if "room_h" in data:
            ROOM_H = float(data["room_h"])
        if "rx" in data:
            RX_POS = tuple(data["rx"])
        for mac, pos in data.get("nodes", {}).items():
            NODE_POSITIONS[mac.strip().upper()] = tuple(pos)
        print(f"[layout] loaded room={ROOM_W}x{ROOM_H}m RX={RX_POS}, {len(NODE_POSITIONS)} TX(s) from {LAYOUT_FILE}", flush=True)
    except Exception as e:
        print(f"[layout] load failed: {e}", flush=True)


def save_calibration():
    """Write the measured elongation scale to JSON. Called under lock."""
    try:
        with open(CALIBRATION_FILE, "w") as f:
            json.dump({"elongation_scale": ELONGATION_SCALE}, f, indent=2)
        print(f"[calibration] saved scale={ELONGATION_SCALE:.3f} to {CALIBRATION_FILE}", flush=True)
    except Exception as e:
        print(f"[calibration] save failed: {e}", flush=True)
    sync_active_profile()


def load_calibration():
    """Load the measured elongation scale from JSON on startup."""
    global ELONGATION_SCALE
    if not os.path.exists(CALIBRATION_FILE):
        return
    try:
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        if "elongation_scale" in data:
            ELONGATION_SCALE = float(data["elongation_scale"])
        print(f"[calibration] loaded scale={ELONGATION_SCALE:.3f} from {CALIBRATION_FILE}", flush=True)
    except Exception as e:
        print(f"[calibration] load failed: {e}", flush=True)


# ---------------------------------------------------------------------------
# Room profiles. Each profile is a full snapshot of everything that is
# specific to one physical room: dimensions, node layout, the empty-room
# baseline and the measured elongation scale. Baseline and scale are
# included because both are properties of that room's propagation (walls,
# furniture, reflections) -- reusing them in a different room silently
# breaks presence detection and localization.
#
# layout.json / baseline.json / calibration.json remain the live working
# state; every save of those also writes through to the active profile, so
# edits made after switching rooms are kept with that room automatically.
# ---------------------------------------------------------------------------
PROFILES_FILE = os.path.join(os.path.dirname(__file__), "profiles.json")
PROFILE_NAME_MAX = 40

profiles = {"active": None, "profiles": {}}
busy_tasks = set()   # running "baseline"/"calibration" captures; profile switch waits for these


def current_snapshot():
    """Live room state as a profile dict. Called under lock."""
    return {
        "room_w": ROOM_W,
        "room_h": ROOM_H,
        "rx": list(RX_POS),
        "nodes": {mac: list(pos) for mac, pos in NODE_POSITIONS.items()},
        "elongation_scale": ELONGATION_SCALE,
        "baseline": dict(baseline),
    }


def save_profiles():
    """Write all profiles to JSON. Called under lock."""
    try:
        with open(PROFILES_FILE, "w") as f:
            json.dump(profiles, f, indent=2)
    except Exception as e:
        print(f"[profiles] save failed: {e}", flush=True)


def load_profiles():
    """Load profiles from JSON on startup. Called after the live state is loaded."""
    global profiles
    if os.path.exists(PROFILES_FILE):
        try:
            with open(PROFILES_FILE) as f:
                data = json.load(f)
            profiles = {"active": data.get("active"), "profiles": data.get("profiles", {})}
            if profiles["active"] not in profiles["profiles"]:
                profiles["active"] = None
            print(f"[profiles] loaded {len(profiles['profiles'])} profile(s), "
                  f"active={profiles['active']!r} from {PROFILES_FILE}", flush=True)
            return
        except Exception as e:
            print(f"[profiles] load failed: {e}", flush=True)
            return
    # First run with profiles: adopt the existing setup as "Default" so
    # nothing configured so far is lost.
    profiles = {"active": "Default", "profiles": {"Default": current_snapshot()}}
    save_profiles()
    print("[profiles] created 'Default' from the existing layout", flush=True)


def sync_active_profile():
    """Write the live state through to the active profile. Called under lock."""
    name = profiles["active"]
    if name is None:
        return
    profiles["profiles"][name] = current_snapshot()
    save_profiles()


def apply_profile(p):
    """Replace the live state with profile dict p and persist it. Called under lock."""
    global ROOM_W, ROOM_H, RX_POS, ELONGATION_SCALE
    ROOM_W = float(p["room_w"])
    ROOM_H = float(p["room_h"])
    RX_POS = tuple(p["rx"])
    NODE_POSITIONS.clear()
    for mac, pos in p.get("nodes", {}).items():
        NODE_POSITIONS[mac.strip().upper()] = tuple(pos)
    ELONGATION_SCALE = float(p.get("elongation_scale", ELONGATION_SCALE))
    baseline.clear()
    baseline.update(p.get("baseline", {}))
    save_layout()
    save_baseline()
    save_calibration()


def profiles_payload():
    return {"active": profiles["active"], "names": sorted(profiles["profiles"])}


def layout_payload():
    return {
        "room_w": ROOM_W, "room_h": ROOM_H,
        "node_positions": {k: list(v) for k, v in NODE_POSITIONS.items()},
        "rx_pos": list(RX_POS),
    }

# Per-MAC state
amp_windows = defaultdict(lambda: deque(maxlen=WINDOW_FRAMES))       # recent amplitude vectors (non-null subcarriers)
heatmap_hist = defaultdict(lambda: deque(maxlen=HEATMAP_HISTORY))    # recent amplitude vectors for heatmap
cov_history = defaultdict(lambda: deque(maxlen=HEATMAP_HISTORY))     # recent CoV scalar values
meta = {}                  # mac -> dict(rssi, noise, sig_mode, rate, mcs, cwb, channel, secondary, last_seen, pkt_count)
baseline_capture = {"active": False, "until": 0.0, "samples": defaultdict(list)}

null_mask_cache = {}       # mac -> boolean numpy-free list, True = active subcarrier
win_phy_key = {}           # mac -> (n_amps, sig_mode, rate, mcs, cwb) of current window's frames

baseline = load_baseline()  # persisted across restarts via baseline.json
load_layout()               # restores RX_POS and NODE_POSITIONS from layout.json
load_calibration()          # restores ELONGATION_SCALE from calibration.json
load_profiles()             # room profiles; must follow the three loads above


def iq_to_amplitude(values):
    """
    values: list of int8, 128 entries = 64 (imag, real) pairs.
    Returns (amplitudes, actives) where actives marks indices that are not
    null/guard subcarriers (i.e. not exactly (0,0)).
    """
    n = len(values) // 2
    amps = [0.0] * n
    actives = [False] * n
    for i in range(n):
        imag = values[2 * i]
        real = values[2 * i + 1]
        if imag == 0 and real == 0:
            actives[i] = False
            amps[i] = 0.0
        else:
            actives[i] = True
            amps[i] = (real * real + imag * imag) ** 0.5
    return amps, actives


def window_active_mask(actives_list):
    """
    Intersection of per-frame active-subcarrier masks: True only for indices
    active in every frame of the window. A subcarrier that goes null in even
    one frame is excluded from that frame's amplitude (stored as 0.0), so
    including it based on any single frame's mask would mix real amplitude
    samples with null-subcarrier zeros in the same series.
    """
    if not actives_list:
        return []
    n = min(len(a) for a in actives_list)
    return [all(a[i] for a in actives_list) for i in range(n)]


def compute_cov(amp_actives_window):
    """
    Coefficient of variation (std/mean) per active subcarrier, then averaged
    across active subcarriers -- your validated room-independent motion signal.
    amp_actives_window: list of (amps, actives) tuples, one per frame in the
    sliding window. Returns (cov, active_idx).
    """
    if not amp_actives_window:
        return 0.0, []
    mask = window_active_mask([actives for _, actives in amp_actives_window])
    active_idx = [i for i, ok in enumerate(mask) if ok]
    if not active_idx:
        return 0.0, []

    covs = []
    for i in active_idx:
        series = [amps[i] for amps, _ in amp_actives_window]
        m = sum(series) / len(series)
        if m == 0:
            continue
        var = sum((x - m) ** 2 for x in series) / len(series)
        std = var ** 0.5
        covs.append(std / m)
    if not covs:
        return 0.0, []
    return sum(covs) / len(covs), active_idx


def udp_listener(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(1.0)
    print(f"[udp] listening on {host}:{port}", flush=True)

    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue

        if len(data) < HDR or data[0:1] != b"C" or data[1] != 2:
            continue

        mac = data[2:8]
        mac_str = ":".join(f"{b:02X}" for b in mac)
        rssi = struct.unpack_from("b", data, 8)[0]
        noise = struct.unpack_from("b", data, 9)[0]
        sig_mode = data[10]
        rate = data[11]
        mcs = data[12]
        cwb = data[13]
        ch = data[14]
        sec = data[15]
        length = struct.unpack_from("<H", data, 16)[0]
        payload = data[HDR:HDR + length]
        length = len(payload)
        values = list(struct.unpack_from(f"{length}b", payload)) if length else []

        if len(values) < 4:
            continue

        amps, actives = iq_to_amplitude(values)

        with lock:
            # Reset the window on any change to length or PHY parameters --
            # amplitudes are only comparable within one constant configuration.
            win = amp_windows[mac_str]
            phy_key = (len(amps), sig_mode, rate, mcs, cwb)
            if win and win_phy_key.get(mac_str) != phy_key:
                win.clear()
                heatmap_hist[mac_str].clear()
            win_phy_key[mac_str] = phy_key

            amp_windows[mac_str].append((amps, actives))
            heatmap_hist[mac_str].append(amps)

            cov, active_idx = compute_cov(list(amp_windows[mac_str]))
            active_set = set(active_idx)
            null_mask_cache[mac_str] = [i in active_set for i in range(len(amps))]
            cov_history[mac_str].append(cov)

            if baseline_capture["active"]:
                baseline_capture["samples"][mac_str].append(cov)

            now = time.time()
            m = meta.setdefault(mac_str, {"pkt_count": 0})
            m.update(rssi=rssi, noise=noise, sig_mode=sig_mode, rate=rate,
                      mcs=mcs, cwb=cwb, channel=ch, secondary=sec,
                      last_seen=now)
            m["pkt_count"] += 1

            if rate != m.get("_last_rate", rate) or sig_mode != m.get("_last_sig_mode", sig_mode):
                print(f"[warn] {mac_str} PHY changed -- CSI not comparable across this change", flush=True)
            m["_last_rate"] = rate
            m["_last_sig_mode"] = sig_mode


def emitter_loop():
    """Push a compact snapshot to all connected clients at EMIT_HZ."""
    period = 1.0 / EMIT_HZ
    while True:
        time.sleep(period)
        with lock:
            if not meta:
                continue
            payload = {}
            mac_covs = {}
            for mac_str, m in meta.items():
                n_sub = len(null_mask_cache.get(mac_str, []))
                active_idx = [i for i, a in enumerate(null_mask_cache.get(mac_str, [])) if a]

                heat_rows = list(heatmap_hist[mac_str])
                heat_active = [[row[i] for i in active_idx] for row in heat_rows]

                cov_series = list(cov_history[mac_str])
                cur_cov = cov_series[-1] if cov_series else 0.0
                base = baseline.get(mac_str)
                motion = None
                if base is not None and base["mean"] is not None:
                    threshold = base["mean"] + 3 * base["std"]
                    motion = cur_cov > threshold

                mac_covs[mac_str] = cur_cov
                payload[mac_str] = {
                    "rssi": m.get("rssi"),
                    "noise": m.get("noise"),
                    "sig_mode": m.get("sig_mode"),
                    "rate": m.get("rate"),
                    "channel": m.get("channel"),
                    "pkt_count": m.get("pkt_count"),
                    "n_active_subcarriers": len(active_idx),
                    "n_total_subcarriers": n_sub,
                    "cov_series": cov_series,
                    "cur_cov": cur_cov,
                    "heatmap": heat_active,
                    "baseline": base,
                    "motion": motion,
                }

            loc = compute_localization(mac_covs, baseline)
            socketio.emit("csi_update", payload)
            socketio.emit("localization", loc)


@app.route("/")
def index():
    return render_template("index.html")


@socketio.on("connect")
def handle_connect():
    """Send persisted layout and baseline status to a newly connected client."""
    with lock:
        socketio.emit("layout_ack", layout_payload())
        socketio.emit("profiles", profiles_payload())
        if baseline:
            socketio.emit("baseline_status", {"phase": "done", "baseline": baseline})


@socketio.on("start_baseline")
def handle_start_baseline(data):
    duration = float(data.get("duration", 8))
    delay = float(data.get("delay", 30))

    with lock:
        busy_tasks.add("baseline")
    socketio.emit("baseline_status", {"phase": "countdown", "delay": delay, "duration": duration})

    def capture():
        # Grace period so the user can leave the room before recording starts.
        remaining = delay
        while remaining > 0:
            step = min(1.0, remaining)
            time.sleep(step)
            remaining -= step
            socketio.emit("baseline_countdown_tick", {"remaining": round(remaining, 1)})

        with lock:
            baseline_capture["active"] = True
            baseline_capture["until"] = time.time() + duration
            baseline_capture["samples"] = defaultdict(list)
        socketio.emit("baseline_status", {"phase": "capturing", "duration": duration})

        time.sleep(duration)

        with lock:
            baseline_capture["active"] = False
            for mac_str, samples in baseline_capture["samples"].items():
                if len(samples) >= 3:
                    m = sum(samples) / len(samples)
                    var = sum((x - m) ** 2 for x in samples) / len(samples)
                    std = var ** 0.5
                    baseline[mac_str] = {"mean": m, "std": std, "n": len(samples)}
            save_baseline()
        socketio.emit("baseline_status", {"phase": "done", "baseline": baseline})

    def run():
        try:
            capture()
        finally:
            with lock:
                busy_tasks.discard("baseline")

    threading.Thread(target=run, daemon=True).start()


@socketio.on("set_room")
def handle_set_room(data):
    global ROOM_W, ROOM_H
    with lock:
        ROOM_W = float(data.get("w", ROOM_W))
        ROOM_H = float(data.get("h", ROOM_H))
        save_layout()
    socketio.emit("layout_ack", layout_payload())


@socketio.on("set_layout")
def handle_set_layout(data):
    """
    data: { mac: "AA:BB:CC:DD:EE:FF", x: 1.2, y: 3.4 }
    or    { rx: true, x: 1.2, y: 3.4 }
    Lets the UI update node positions without restarting the server.
    """
    global RX_POS
    with lock:
        if data.get("rx"):
            RX_POS = (float(data["x"]), float(data["y"]))
        elif "mac" in data:
            # Normalize to match the uppercase MAC strings the UDP listener
            # builds (":".join(f"{b:02X}"...)) -- otherwise a mismatched case
            # silently never matches incoming traffic and the node never
            # participates in localization.
            NODE_POSITIONS[data["mac"].strip().upper()] = (float(data["x"]), float(data["y"]))
        save_layout()
    socketio.emit("layout_ack", layout_payload())


@socketio.on("calibrate_scale")
def handle_calibrate_scale(data):
    """
    data: { x: <metres>, y: <metres>, duration: <seconds, default 5> }
    Stand at a known (x, y) point (measured against the same tape-measured
    origin as the node layout) and capture live CoV. For each configured
    link, the true path elongation is computable from geometry alone
    (dist(tx,point) + dist(point,rx))/2 - dist(tx,rx)/2); comparing that to
    the observed CoV excess above baseline gives a measured elongation_scale
    (metres per unit CoV excess), replacing the hardcoded guess.
    """
    try:
        point = (float(data["x"]), float(data["y"]))
    except (KeyError, TypeError, ValueError):
        socketio.emit("calibration_status", {"phase": "error", "message": "x and y are required"})
        return
    duration = float(data.get("duration", 5))

    with lock:
        busy_tasks.add("calibration")
    socketio.emit("calibration_status", {"phase": "capturing", "duration": duration, "point": list(point)})

    def capture():
        global ELONGATION_SCALE
        samples = defaultdict(list)
        end = time.time() + duration
        while time.time() < end:
            time.sleep(0.1)
            with lock:
                for mac in NODE_POSITIONS:
                    series = cov_history.get(mac)
                    if series:
                        samples[mac].append(series[-1])

        with lock:
            ratios = []
            per_link = {}
            for mac, tx_pos in NODE_POSITIONS.items():
                base = baseline.get(mac)
                covs = samples.get(mac)
                if base is None or not covs:
                    continue
                avg_cov = sum(covs) / len(covs)
                threshold = base["mean"] + 3 * base["std"]
                excess = avg_cov - threshold
                c = _dist(tx_pos, RX_POS) / 2
                a_true = (_dist(tx_pos, point) + _dist(point, RX_POS)) / 2
                delta_true = max(0.0, a_true - c)
                if excess > 1e-6:
                    ratio = delta_true / excess
                    ratios.append(ratio)
                    per_link[mac] = {"excess": round(excess, 4),
                                      "delta_true_m": round(delta_true, 4),
                                      "scale": round(ratio, 3)}

            if ratios:
                ELONGATION_SCALE = sum(ratios) / len(ratios)
                save_calibration()
                socketio.emit("calibration_status", {
                    "phase": "done", "scale": round(ELONGATION_SCALE, 3), "per_link": per_link,
                })
            else:
                socketio.emit("calibration_status", {
                    "phase": "failed",
                    "message": "no configured link showed CoV excess above its baseline during "
                               "capture -- stand where motion is actually sensed and recapture",
                })

    def run():
        try:
            capture()
        finally:
            with lock:
                busy_tasks.discard("calibration")

    threading.Thread(target=run, daemon=True).start()


@socketio.on("remove_node")
def handle_remove_node(data):
    with lock:
        NODE_POSITIONS.pop(data.get("mac", "").strip().upper(), None)
        save_layout()
    socketio.emit("layout_ack", layout_payload())


@socketio.on("clear_baseline")
def handle_clear_baseline():
    with lock:
        baseline.clear()
        save_baseline()
    socketio.emit("baseline_status", {"phase": "cleared", "baseline": {}})


def _profile_error(message):
    socketio.emit("profile_status", {"phase": "error", "message": message})


@socketio.on("save_profile")
def handle_save_profile(data):
    """
    data: { name: "Bedroom" }
    Saves the current room setup (layout, baseline, scale) under name and
    makes it the active profile. Overwrites an existing profile of that name.
    """
    name = str(data.get("name", "")).strip()
    if not name or len(name) > PROFILE_NAME_MAX:
        _profile_error(f"profile name must be 1-{PROFILE_NAME_MAX} characters")
        return
    with lock:
        profiles["profiles"][name] = current_snapshot()
        profiles["active"] = name
        save_profiles()
    socketio.emit("profiles", profiles_payload())
    socketio.emit("profile_status", {"phase": "saved", "name": name})


@socketio.on("load_profile")
def handle_load_profile(data):
    """
    data: { name: "Bedroom" }
    Switches the live room setup to a saved profile.
    """
    name = str(data.get("name", "")).strip()
    with lock:
        if busy_tasks:
            # A running capture would write its result into whichever room
            # is active when it finishes -- i.e. the wrong one.
            busy = " and ".join(sorted(busy_tasks))
            _profile_error(f"wait for the {busy} capture to finish before switching rooms")
            return
        p = profiles["profiles"].get(name)
        if p is None:
            _profile_error(f"no profile named {name!r}")
            return
        profiles["active"] = name
        apply_profile(p)
        save_profiles()
        has_baseline = bool(baseline)
        scale = ELONGATION_SCALE
    socketio.emit("profiles", profiles_payload())
    socketio.emit("layout_ack", layout_payload())
    socketio.emit("baseline_status", {"phase": "loaded" if has_baseline else "cleared",
                                      "baseline": baseline})
    socketio.emit("calibration_status", {"phase": "loaded", "scale": round(scale, 3)})
    socketio.emit("profile_status", {"phase": "loaded", "name": name})


@socketio.on("delete_profile")
def handle_delete_profile(data):
    """
    data: { name: "Bedroom" }
    Deletes a saved profile. Deleting the active one leaves the current
    setup in place, just no longer saved under any profile.
    """
    name = str(data.get("name", "")).strip()
    with lock:
        if profiles["profiles"].pop(name, None) is None:
            _profile_error(f"no profile named {name!r}")
            return
        if profiles["active"] == name:
            profiles["active"] = None
        save_profiles()
    socketio.emit("profiles", profiles_payload())
    socketio.emit("profile_status", {"phase": "deleted", "name": name})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=1201, help="UDP port (CONTROLLER_PORT)")
    ap.add_argument("--web-host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=5000)
    args = ap.parse_args()

    t = threading.Thread(target=udp_listener, args=(args.host, args.port), daemon=True)
    t.start()

    e = threading.Thread(target=emitter_loop, daemon=True)
    e.start()

    print(f"[web] dashboard at http://{args.web_host}:{args.web_port}", flush=True)
    socketio.run(app, host=args.web_host, port=args.web_port, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()