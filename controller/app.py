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
# The CoV window is time-based, not frame-count-based: with a fixed frame
# count, the window only represents ~1s of real time if the per-link packet
# rate actually holds at ~50Hz. With multiple Illuminators (and the
# Guardian) sharing one channel, the real rate can drop well below that --
# a fixed 50-frame window then silently stretches to many seconds of lag.
WINDOW_SECONDS      = 1.0   # the window always covers this much real time, regardless of rate
MIN_WINDOW_FRAMES   = 5     # floor: keep at least this many samples even if they're older than WINDOW_SECONDS,
                             # so CoV doesn't become degenerate when the rate is very low
MAX_WINDOW_FRAMES   = 500   # safety cap on memory if the rate spikes
HEATMAP_HISTORY = 200       # columns of amplitude history kept per illuminator
EMIT_HZ = 10                # throttle UI updates regardless of packet rate

# Static presence: compares the *shape* of the mean subcarrier amplitude
# profile against an empty-room fingerprint, so a person who isn't moving
# (and so leaves CoV at baseline) is still detected by the attenuation and
# reflection their body adds.
PRESENCE_WINDOW_S = 3.0         # averaging window for the presence score (also the fingerprint chunk size)
PRESENCE_K = 3.0                # threshold = empty-room score mean + K * std ...
PRESENCE_MIN_RATIO = 3.0        # ... but never below RATIO * mean: a 60s capture underestimates the
                                # slow drift an empty room shows over minutes (measured: up to ~2.5x mean)
PRESENCE_HOLD_S = 5.0           # room stays "occupied" this long after the last detection
MIN_FINGERPRINT_CHUNKS = 5      # need >= 5 windows (~15s) of empty room to estimate the score spread
MIN_FRAMES_PER_WINDOW = 5

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
    any_excess = False  # True once any link's CoV is above its baseline threshold

    for mac, tx_pos in NODE_POSITIONS.items():
        if mac not in mac_covs:
            continue
        base = mac_baselines.get(mac)
        if base is None:
            continue
        cur_cov = mac_covs[mac]
        threshold = base["mean"] + 3*base["std"]
        excess = max(0.0, cur_cov - threshold)
        if excess > 0:
            any_excess = True

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

    # With no excess on any link every ellipse has collapsed onto its direct
    # TX-RX path, and those paths only meet at RX -- so the heatmap and blob
    # would always point at the receiver whether or not anyone is there.
    # Show nothing rather than that artefact.
    if not any_excess:
        combined = [0.0] * len(combined)

    # normalise
    if active_links > 0:
        max_s = max(combined) or 1.0
        combined = [round(s/max_s, 4) for s in combined]

    # find blob: coarse grid peak, refined to the actual ellipse intersection
    blob = None
    if active_links >= 2 and any_excess and max(combined) > 0.3:
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
amp_windows = defaultdict(lambda: deque(maxlen=MAX_WINDOW_FRAMES))   # (timestamp, amps, actives) -- time-pruned to WINDOW_SECONDS, see udp_listener
heatmap_hist = defaultdict(lambda: deque(maxlen=HEATMAP_HISTORY))    # recent amplitude vectors for heatmap
cov_history = defaultdict(lambda: deque(maxlen=HEATMAP_HISTORY))     # recent CoV scalar values
presence_frames = defaultdict(lambda: deque(maxlen=2000))            # (t, amps, actives) for the presence window
meta = {}                  # mac -> dict(rssi, noise, sig_mode, rate, mcs, cwb, channel, secondary, last_seen, pkt_count)
baseline_capture = {"active": False, "until": 0.0, "samples": defaultdict(list), "frames": defaultdict(list)}
last_occupied_t = 0.0      # time of the last per-link motion/presence detection, for the room-level hold

# In-dashboard CSI-fuzzing evaluation (Objective 3/4) -- records every incoming
# (timestamp, mac, amps, actives) during a capture window, the same raw data
# controller/fuzzing_eval.py works from, but captured live instead of via
# csi_server.py + a separate CSV replay step. fuzzing_eval.py is left exactly
# as-is for offline/standalone use; this is purely additive.
fuzzing_capture = {"active": False, "records": []}

# Live (continuously updating) legitimate-vs-eavesdropper comparison, as an
# alternative to the one-shot fuzzing_capture above. Two rolling windows are
# maintained in parallel over the SAME incoming stream, both pruned to
# WINDOW_SECONDS: fuzz_live_eaves_window gets every record, fuzz_live_legit
# only the ones not in a guardian-burst slot (per the live seed). Because
# both cover the same real-time span, this gives a genuinely time-aligned
# comparison -- unlike the one-shot capture's post-hoc merged series, where
# the blanked series has fewer points and drifts out of alignment.
fuzz_live = {"active": False, "seed": None, "burst_probability": 0.15}  # matches FUZZ_DEFAULT_BURST_PROBABILITY below
fuzz_live_eaves_window = deque()
fuzz_live_legit_window = deque()
fuzz_live_eaves_history = deque(maxlen=HEATMAP_HISTORY)
fuzz_live_legit_history = deque(maxlen=HEATMAP_HISTORY)

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


# ---------------------------------------------------------------------------
# In-dashboard CSI-fuzzing evaluation (Objective 3/4). slot_hash()/
# is_burst_slot() must stay byte-for-byte equivalent to GuardianBeacon.cpp's
# slotHash() and controller/fuzzing_eval.py's copy -- duplicated deliberately
# in all three places rather than shared, same reasoning as fuzzing_eval.py's
# module docstring. See that file for the full explanation of why a merged,
# undifferentiated stream (not the Illuminator's own always-clean log) is
# the comparison that actually exercises the PRNG-blanking logic.
# ---------------------------------------------------------------------------
FUZZ_SLOT_MS = 20
FUZZ_DEFAULT_BURST_PROBABILITY = 0.15


def fuzz_slot_hash(seed, slot):
    h = (seed ^ slot) & 0xFFFFFFFF
    h ^= h >> 16
    h = (h * 0x7feb352d) & 0xFFFFFFFF
    h ^= h >> 15
    h = (h * 0x846ca68b) & 0xFFFFFFFF
    h ^= h >> 16
    return h


def fuzz_is_burst_slot(seed, slot, burst_probability):
    threshold = int(burst_probability * 4294967296.0)
    return fuzz_slot_hash(seed, slot) < threshold


def fuzz_replay_cov_series(records, blank_bursts, seed, burst_probability):
    """records: [(timestamp, amps, actives), ...], already time-ordered.
    Mirrors app.py's own WINDOW_SECONDS/MIN_WINDOW_FRAMES pruning."""
    window = deque()
    series = []
    for ts, amps, actives in records:
        slot = int(ts * 1000 // FUZZ_SLOT_MS)
        if blank_bursts and fuzz_is_burst_slot(seed, slot, burst_probability):
            continue
        window.append((ts, amps, actives))
        while len(window) > MIN_WINDOW_FRAMES and ts - window[0][0] > WINDOW_SECONDS:
            window.popleft()
        cov, _ = compute_cov([(a, ac) for _, a, ac in window])
        series.append(cov)
    return series


def fuzz_summarize(series):
    series = [s for s in series if s is not None]
    if not series:
        return None
    m = sum(series) / len(series)
    var = sum((x - m) ** 2 for x in series) / len(series)
    std = var ** 0.5
    return {"n": len(series), "mean": round(m, 5), "std": round(std, 5),
            "cv": round(std / m, 3) if m else None}


def fuzz_downsample(series, max_points=400):
    """Evenly-spaced downsample for charting -- keeps the payload small
    regardless of capture duration/rate."""
    series = [round(s, 5) for s in series if s is not None]
    if len(series) <= max_points:
        return series
    step = len(series) / max_points
    return [series[int(i * step)] for i in range(max_points)]


def mean_profile(frames):
    """
    Per-subcarrier mean amplitude over frames [(amps, actives)], plus the
    mask of subcarriers active in every frame. All frames must share one PHY
    config (same vector length).
    """
    mask = window_active_mask([actives for _, actives in frames])
    n = len(mask)
    total = [0.0] * n
    for amps, _ in frames:
        for i in range(n):
            total[i] += amps[i]
    return [t / len(frames) for t in total], mask


def shape_distance(prof_a, prof_b, mask):
    """
    RMS difference between two amplitude profiles after each is divided by
    its own mean over the masked subcarriers. Dividing out the mean removes
    the ESP32's AGC gain, which rescales every subcarrier together and would
    otherwise look like presence; only the shape across subcarriers -- what
    a body in the room actually changes -- is compared.
    """
    idx = [i for i, ok in enumerate(mask) if ok]
    if len(idx) < 8:
        return None
    ma = sum(prof_a[i] for i in idx) / len(idx)
    mb = sum(prof_b[i] for i in idx) / len(idx)
    if ma <= 0 or mb <= 0:
        return None
    return (sum((prof_a[i] / ma - prof_b[i] / mb) ** 2 for i in idx) / len(idx)) ** 0.5


def build_fingerprint(frames, phy_key):
    """
    Empty-room fingerprint from frames [(t, amps, actives)] of one PHY config.

    The capture is split into PRESENCE_WINDOW_S chunks and each chunk is
    scored against the mean of all the *other* chunks (leave-one-out), so
    the spread matches what the live score -- a fresh window against a
    reference it wasn't part of -- shows when the room is empty.
    Returns the fingerprint dict, or None if the capture was too short.
    """
    if not frames:
        return None
    t0 = frames[0][0]
    chunks = defaultdict(list)
    for t, amps, actives in frames:
        chunks[int((t - t0) // PRESENCE_WINDOW_S)].append((amps, actives))
    chunks = [c for c in chunks.values() if len(c) >= MIN_FRAMES_PER_WINDOW]
    if len(chunks) < MIN_FINGERPRINT_CHUNKS:
        return None

    all_frames = [f for c in chunks for f in c]
    profile, mask = mean_profile(all_frames)
    n_all = len(all_frames)
    total = [p * n_all for p in profile]

    scores = []
    for c in chunks:
        chunk_prof, _ = mean_profile(c)
        n_rest = n_all - len(c)
        rest = [(total[i] - chunk_prof[i] * len(c)) / n_rest for i in range(len(total))]
        s = shape_distance(chunk_prof, rest, mask)
        if s is not None:
            scores.append(s)
    if len(scores) < MIN_FINGERPRINT_CHUNKS:
        return None

    m = sum(scores) / len(scores)
    std = (sum((s - m) ** 2 for s in scores) / len(scores)) ** 0.5
    return {
        "phy": list(phy_key),
        "profile": [round(p, 4) for p in profile],
        "mask": mask,
        "score_mean": m,
        "score_std": std,
        "n_chunks": len(scores),
    }


def presence_state(mac_str):
    """
    Live static-presence score for one link vs its fingerprint. Called under lock.
    Returns {score, threshold, present} or {present: None, reason} when no
    comparison is possible.
    """
    fp = (baseline.get(mac_str) or {}).get("fingerprint")
    if not fp:
        return {"present": None, "reason": "no fingerprint"}
    if list(win_phy_key.get(mac_str, ())) != fp["phy"]:
        return {"present": None, "reason": "PHY differs from fingerprint"}
    now = time.time()
    frames = [(a, m) for t, a, m in presence_frames[mac_str] if now - t <= PRESENCE_WINDOW_S]
    if len(frames) < MIN_FRAMES_PER_WINDOW:
        return {"present": None, "reason": "too few packets"}
    profile, mask = mean_profile(frames)
    if len(profile) != len(fp["profile"]):
        return {"present": None, "reason": "PHY differs from fingerprint"}
    mask = [a and b for a, b in zip(mask, fp["mask"])]
    score = shape_distance(profile, fp["profile"], mask)
    if score is None:
        return {"present": None, "reason": "too few active subcarriers"}
    threshold = max(fp["score_mean"] + PRESENCE_K * max(fp["score_std"], 1e-4),
                    fp["score_mean"] * PRESENCE_MIN_RATIO)
    return {"score": round(score, 5), "threshold": round(threshold, 5), "present": score > threshold}


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
            now = time.time()
            # Reset the window on any change to length or PHY parameters --
            # amplitudes are only comparable within one constant configuration.
            win = amp_windows[mac_str]
            phy_key = (len(amps), sig_mode, rate, mcs, cwb)
            if win and win_phy_key.get(mac_str) != phy_key:
                win.clear()
                heatmap_hist[mac_str].clear()
                presence_frames[mac_str].clear()
            win_phy_key[mac_str] = phy_key

            win.append((now, amps, actives))
            # Keep the window to WINDOW_SECONDS of real time, not a fixed frame
            # count, so CoV responsiveness doesn't silently degrade when the
            # per-link packet rate drops (more nodes sharing the channel, etc.).
            # MIN_WINDOW_FRAMES is a floor so a very low rate doesn't leave too
            # few samples for a meaningful std/mean.
            while len(win) > MIN_WINDOW_FRAMES and now - win[0][0] > WINDOW_SECONDS:
                win.popleft()
            heatmap_hist[mac_str].append(amps)
            presence_frames[mac_str].append((now, amps, actives))

            cov, active_idx = compute_cov([(a, ac) for _, a, ac in win])
            active_set = set(active_idx)
            null_mask_cache[mac_str] = [i in active_set for i in range(len(amps))]
            cov_history[mac_str].append(cov)

            if baseline_capture["active"]:
                baseline_capture["samples"][mac_str].append(cov)
                baseline_capture["frames"][mac_str].append((now, amps, actives, phy_key))

            if fuzzing_capture["active"]:
                # Records from every source that reaches here -- Illuminators
                # and, once CsiCollector.cpp forwards it, the Guardian too.
                # No filtering: that's the point, see handle_start_fuzzing_capture.
                fuzzing_capture["records"].append((now, mac_str, amps, actives))

            if fuzz_live["active"]:
                fuzz_live_eaves_window.append((now, amps, actives))
                while (len(fuzz_live_eaves_window) > MIN_WINDOW_FRAMES
                       and now - fuzz_live_eaves_window[0][0] > WINDOW_SECONDS):
                    fuzz_live_eaves_window.popleft()

                slot = int(now * 1000 // FUZZ_SLOT_MS)
                if not fuzz_is_burst_slot(fuzz_live["seed"], slot, fuzz_live["burst_probability"]):
                    fuzz_live_legit_window.append((now, amps, actives))
                    while (len(fuzz_live_legit_window) > MIN_WINDOW_FRAMES
                           and now - fuzz_live_legit_window[0][0] > WINDOW_SECONDS):
                        fuzz_live_legit_window.popleft()

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
    global last_occupied_t
    period = 1.0 / EMIT_HZ
    while True:
        time.sleep(period)
        with lock:
            if not meta:
                continue
            payload = {}
            mac_covs = {}
            detected_by = []    # (mac, "motion"|"presence") for every link firing this tick
            any_checked = False
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
                presence = presence_state(mac_str)

                if motion is not None or presence["present"] is not None:
                    any_checked = True
                if motion:
                    detected_by.append((mac_str, "motion"))
                if presence["present"]:
                    detected_by.append((mac_str, "presence"))

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
                    # CoV stats only -- the fingerprint profile isn't needed per tick
                    "baseline": {k: base.get(k) for k in ("mean", "std", "n")} if base else None,
                    "motion": motion,
                    "presence": presence,
                }

            # Room is occupied if any link sees motion or a static presence,
            # held for PRESENCE_HOLD_S so a brief dip doesn't flicker it to empty.
            now = time.time()
            if detected_by:
                last_occupied_t = now
            room = {
                "occupied": (now - last_occupied_t <= PRESENCE_HOLD_S) if any_checked else None,
                "detected_by": [{"mac": mac, "kind": kind} for mac, kind in detected_by],
            }

            loc = compute_localization(mac_covs, baseline)
            socketio.emit("csi_update", payload)
            socketio.emit("localization", loc)
            socketio.emit("room_presence", room)

            if fuzz_live["active"]:
                eaves_cov, _ = compute_cov([(a, ac) for _, a, ac in fuzz_live_eaves_window])
                legit_cov, _ = compute_cov([(a, ac) for _, a, ac in fuzz_live_legit_window])
                fuzz_live_eaves_history.append(eaves_cov)
                fuzz_live_legit_history.append(legit_cov)
                socketio.emit("fuzzing_live_update", {
                    "legit_cov": round(legit_cov, 5),
                    "eaves_cov": round(eaves_cov, 5),
                    "legit_series": list(fuzz_live_legit_history),
                    "eaves_series": list(fuzz_live_eaves_history),
                    "n_legit_window": len(fuzz_live_legit_window),
                    "n_eaves_window": len(fuzz_live_eaves_window),
                })

                # Spatial side-by-side: a second localization pass where each
                # Illuminator's own CoV is blended with whatever non-Illuminator
                # (i.e. Guardian) traffic is currently in its own window --
                # simulating an eavesdropper who, generously, has your exact
                # node geometry and baseline but can't separate Guardian noise
                # from real motion the way the MAC-scoped pipeline does. Reuses
                # compute_localization() unchanged -- same function, different
                # (contaminated) input.
                guardian_macs = [m for m in meta if m not in NODE_POSITIONS]
                guardian_frames = []
                for gm in guardian_macs:
                    guardian_frames.extend(amp_windows.get(gm, []))

                eaves_mac_covs = {}
                for illum_mac in NODE_POSITIONS:
                    illum_frames = list(amp_windows.get(illum_mac, []))
                    merged = illum_frames + guardian_frames
                    cov, _ = compute_cov([(a, ac) for _, a, ac in merged])
                    eaves_mac_covs[illum_mac] = cov

                eaves_loc = compute_localization(eaves_mac_covs, baseline)
                socketio.emit("localization_eavesdropper", eaves_loc)


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
    duration = float(data.get("duration", 60))
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
            baseline_capture["frames"] = defaultdict(list)
        socketio.emit("baseline_status", {"phase": "capturing", "duration": duration})

        time.sleep(duration)

        no_fingerprint = []
        with lock:
            baseline_capture["active"] = False
            for mac_str, samples in baseline_capture["samples"].items():
                if len(samples) >= 3:
                    m = sum(samples) / len(samples)
                    var = sum((x - m) ** 2 for x in samples) / len(samples)
                    std = var ** 0.5
                    entry = {"mean": m, "std": std, "n": len(samples)}

                    # Fingerprint from the dominant PHY config only -- amplitudes
                    # from different configs aren't comparable.
                    frames = baseline_capture["frames"][mac_str]
                    phy_counts = defaultdict(int)
                    for f in frames:
                        phy_counts[f[3]] += 1
                    fp = None
                    if phy_counts:
                        phy = max(phy_counts, key=phy_counts.get)
                        fp = build_fingerprint([f[:3] for f in frames if f[3] == phy], phy)
                    if fp:
                        entry["fingerprint"] = fp
                    else:
                        no_fingerprint.append(mac_str)
                        print(f"[baseline] {mac_str}: capture too short for a presence fingerprint "
                              f"(need >= {MIN_FINGERPRINT_CHUNKS * PRESENCE_WINDOW_S:.0f}s)", flush=True)
                    baseline[mac_str] = entry
            baseline_capture["frames"] = defaultdict(list)
            save_baseline()
        socketio.emit("baseline_status", {"phase": "done", "baseline": baseline,
                                          "no_fingerprint": no_fingerprint})

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


@socketio.on("start_fuzzing_capture")
def handle_start_fuzzing_capture(data):
    """
    data: { duration: <seconds, default 20>, seed: <int, required>,
            burst_probability: <float, default 0.15> }

    Records every incoming CSI record (Illuminators and, once
    CsiCollector.cpp forwards it, the Guardian) for `duration` seconds, then
    replays that one capture three ways -- see fuzzing_eval.py's module
    docstring for why a single capture is enough and why the merged,
    undifferentiated view (not the Illuminator's own always-clean log) is
    the comparison that actually means something:
      1) legitimate, MAC-scoped  -- today's real pipeline (Illuminator
         records only; never had Guardian noise mixed in to begin with)
      2) legitimate, merged + blanked -- all records, burst-slot ones
         excluded using the known schedule
      3) eavesdropper, merged, unblanked -- all records, nothing removed
    (2) vs (3) is the actual protection/degradation evidence.
    """
    try:
        seed = int(data["seed"])
    except (KeyError, TypeError, ValueError):
        socketio.emit("fuzzing_eval_status", {"phase": "error", "message": "seed is required"})
        return
    duration = float(data.get("duration", 20))
    burst_probability = float(data.get("burst_probability", FUZZ_DEFAULT_BURST_PROBABILITY))

    with lock:
        busy_tasks.add("fuzzing_capture")
    socketio.emit("fuzzing_eval_status", {"phase": "capturing", "duration": duration})

    def capture():
        with lock:
            fuzzing_capture["active"] = True
            fuzzing_capture["records"] = []

        time.sleep(duration)

        with lock:
            fuzzing_capture["active"] = False
            records = list(fuzzing_capture["records"])

        illum_records = [(ts, a, ac) for ts, mac, a, ac in records if mac in NODE_POSITIONS]
        all_records = sorted(((ts, a, ac) for ts, mac, a, ac in records), key=lambda r: r[0])

        if not records:
            socketio.emit("fuzzing_eval_status", {
                "phase": "failed",
                "message": "no CSI records captured during the window -- check the boards are transmitting",
            })
            return

        legit_scoped_series = fuzz_replay_cov_series(
            illum_records, blank_bursts=False, seed=seed, burst_probability=burst_probability)
        legit_merged_series = fuzz_replay_cov_series(
            all_records, blank_bursts=True, seed=seed, burst_probability=burst_probability)
        eaves_merged_series = fuzz_replay_cov_series(
            all_records, blank_bursts=False, seed=seed, burst_probability=burst_probability)

        legit_scoped = fuzz_summarize(legit_scoped_series)
        legit_merged = fuzz_summarize(legit_merged_series)
        eaves_merged = fuzz_summarize(eaves_merged_series)

        burst_count = sum(
            1 for ts, _, _ in all_records
            if fuzz_is_burst_slot(seed, int(ts * 1000 // FUZZ_SLOT_MS), burst_probability)
        )

        socketio.emit("fuzzing_eval_status", {
            "phase": "done",
            "n_illuminator_records": len(illum_records),
            "n_total_records": len(all_records),
            "n_burst_records": burst_count,
            "legitimate_scoped": legit_scoped,
            "legitimate_merged_blanked": legit_merged,
            "eavesdropper_merged": eaves_merged,
            # Downsampled series for the side-by-side chart -- (2) vs (3) is
            # the actual protection/degradation comparison; (1) included too
            # since it's cheap and useful context.
            "series_legitimate_scoped": fuzz_downsample(legit_scoped_series),
            "series_legitimate_merged_blanked": fuzz_downsample(legit_merged_series),
            "series_eavesdropper_merged": fuzz_downsample(eaves_merged_series),
        })

    def run():
        try:
            capture()
        finally:
            with lock:
                busy_tasks.discard("fuzzing_capture")

    threading.Thread(target=run, daemon=True).start()


@socketio.on("start_fuzzing_live")
def handle_start_fuzzing_live(data):
    """
    data: { seed: <int, required>, burst_probability: <float, default 0.15> }
    Continuously-updating alternative to start_fuzzing_capture: two rolling
    windows (legitimate/eavesdropper) are maintained in udp_listener over
    the same live stream, both genuinely time-aligned since they cover the
    same WINDOW_SECONDS span -- emitter_loop pushes both every tick via
    "fuzzing_live_update" until stop_fuzzing_live is called.
    """
    try:
        seed = int(data["seed"])
    except (KeyError, TypeError, ValueError):
        socketio.emit("fuzzing_eval_status", {"phase": "error", "message": "seed is required"})
        return
    burst_probability = float(data.get("burst_probability", 0.15))

    with lock:
        fuzz_live["active"] = True
        fuzz_live["seed"] = seed
        fuzz_live["burst_probability"] = burst_probability
        fuzz_live_eaves_window.clear()
        fuzz_live_legit_window.clear()
        fuzz_live_eaves_history.clear()
        fuzz_live_legit_history.clear()
    socketio.emit("fuzzing_eval_status", {"phase": "live_started"})


@socketio.on("stop_fuzzing_live")
def handle_stop_fuzzing_live():
    with lock:
        fuzz_live["active"] = False
    socketio.emit("fuzzing_eval_status", {"phase": "live_stopped"})


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