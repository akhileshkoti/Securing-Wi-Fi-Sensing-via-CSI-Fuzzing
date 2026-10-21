#!/usr/bin/env python3
"""
Standalone evaluation harness for the CSI-fuzzing defense (Objective 3).

Deliberately NOT wired into app.py / csi_server.py -- kept separate so the
already-tested sensing pipeline is never at risk while this is developed and
measured. Run it on its own, against a CSV log already captured by
csi_server.py (unchanged; this just reads its output).

It replays one per-MAC CSV log and classifies each record's timestamp
against the same PRNG-derived burst schedule the Guardian firmware
(lib/GuardianBeacon) uses, via slot_hash() below -- it must stay
byte-for-byte equivalent to GuardianBeacon.cpp's slotHash(). Neither side
needs to exchange anything over the network beyond agreeing on the seed
once; the schedule is computed independently from (seed, wall-clock time).

For the same log it computes two CoV series:
  - "legitimate": records whose timestamp falls in a guardian-burst slot
    are excluded before the sliding-window CoV is computed -- this is what
    a receiver that knows the seed does.
  - "eavesdropper": every record is used -- this is what a passive listener
    without the seed is stuck with.
Comparing the two is the protection/degradation measurement Objective 4
calls for.

IMPORTANT: this only shows a real effect on a log captured while an actual
Guardian board was transmitting in the room. Run against an old log from
before the Guardian existed (e.g. the sample CSVs already in csi_logs/) and
"legitimate" vs "eavesdropper" will look almost the same -- there was no
guardian noise in the air to blank in the first place. Use this script to
validate the schedule/harness now; the real comparison needs a fresh
capture made with the Guardian actively running.

Usage:
    python fuzzing_eval.py --csv ../csi_logs/csi_XXXX.csv --seed 1234567
"""
import argparse
import csv
import math
from collections import deque

SLOT_MS = 20                 # must match GuardianSettings::SLOT_MS
DEFAULT_BURST_PROBABILITY = 0.15   # must match GuardianSettings::BURST_PROBABILITY
WINDOW_FRAMES = 50            # matches controller/app.py's sliding window


def slot_hash(seed: int, slot: int) -> int:
    """Must match GuardianBeacon.cpp's slotHash() exactly (splitmix32-style mix)."""
    h = (seed ^ slot) & 0xFFFFFFFF
    h ^= h >> 16
    h = (h * 0x7feb352d) & 0xFFFFFFFF
    h ^= h >> 15
    h = (h * 0x846ca68b) & 0xFFFFFFFF
    h ^= h >> 16
    return h


def is_burst_slot(seed: int, slot: int, burst_probability: float) -> bool:
    threshold = int(burst_probability * 4294967296.0)
    return slot_hash(seed, slot) < threshold


def iq_to_amplitude(values):
    """Same convention as controller/app.py's iq_to_amplitude() -- duplicated
    deliberately, see module docstring on why this file doesn't import app.py."""
    n = len(values) // 2
    amps = [0.0] * n
    actives = [False] * n
    for i in range(n):
        imag, real = values[2 * i], values[2 * i + 1]
        if imag == 0 and real == 0:
            continue
        actives[i] = True
        amps[i] = math.hypot(real, imag)
    return amps, actives


def window_active_mask(actives_list):
    if not actives_list:
        return []
    n = min(len(a) for a in actives_list)
    return [all(a[i] for a in actives_list) for i in range(n)]


def compute_cov(window):
    """window: list of (amps, actives). Same logic as controller/app.py's
    compute_cov() -- intersection of per-frame active masks, duplicated
    deliberately rather than imported (see module docstring)."""
    if not window:
        return None
    mask = window_active_mask([a for _, a in window])
    idx = [i for i, ok in enumerate(mask) if ok]
    if not idx:
        return None
    covs = []
    for i in idx:
        series = [amps[i] for amps, _ in window]
        m = sum(series) / len(series)
        if m == 0:
            continue
        var = sum((x - m) ** 2 for x in series) / len(series)
        covs.append((var ** 0.5) / m)
    return sum(covs) / len(covs) if covs else None


def load_records(csv_path):
    """Returns [(timestamp, amps, actives), ...] from a csi_server.py CSV log."""
    records = []
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        n_csi = sum(1 for h in header if h.startswith("csi_"))
        for row in reader:
            ts = float(row[0])
            values = [int(v) for v in row[-n_csi:]] if n_csi else []
            if len(values) < 4:
                continue
            amps, actives = iq_to_amplitude(values)
            records.append((ts, amps, actives))
    return records


def rolling_cov_series(records, blank_bursts, seed, burst_probability):
    window = deque(maxlen=WINDOW_FRAMES)
    series = []
    for ts, amps, actives in records:
        slot = int(ts * 1000 // SLOT_MS)
        if blank_bursts and is_burst_slot(seed, slot, burst_probability):
            continue
        window.append((amps, actives))
        cov = compute_cov(list(window))
        if cov is not None:
            series.append(cov)
    return series


def summarize(label, series):
    if not series:
        print(f"  {label}: no samples")
        return
    m = sum(series) / len(series)
    var = sum((x - m) ** 2 for x in series) / len(series)
    std = var ** 0.5
    cv = std / m if m else float("nan")
    print(f"  {label}: n={len(series)}  mean={m:.5f}  std={std:.5f}  cv={cv:.3f}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True, help="per-MAC CSV log from csi_server.py")
    ap.add_argument("--seed", type=int, required=True,
                     help="shared PRNG seed (must match GuardianSettings::PRNG_SEED)")
    ap.add_argument("--burst-probability", type=float, default=DEFAULT_BURST_PROBABILITY,
                     help="must match GuardianSettings::BURST_PROBABILITY (default 0.15)")
    args = ap.parse_args()

    records = load_records(args.csv)
    print(f"loaded {len(records)} records from {args.csv}")
    if not records:
        return

    burst_count = sum(
        1 for ts, _, _ in records
        if is_burst_slot(args.seed, int(ts * 1000 // SLOT_MS), args.burst_probability)
    )
    print(f"{burst_count}/{len(records)} records ({burst_count / len(records) * 100:.1f}%) "
          f"fall in a guardian-burst slot\n")

    legit = rolling_cov_series(records, blank_bursts=True,
                                seed=args.seed, burst_probability=args.burst_probability)
    eaves = rolling_cov_series(records, blank_bursts=False,
                                seed=args.seed, burst_probability=args.burst_probability)

    print("CoV series comparison (eavesdropper's higher spread == the attacker's")
    print("motion score degrading toward noise; this needs a capture where a real")
    print("Guardian board was actually transmitting to show a non-trivial gap):")
    summarize("legitimate   (seed-aware, bursts blanked)", legit)
    summarize("eavesdropper (no seed, bursts included)  ", eaves)


if __name__ == "__main__":
    main()
