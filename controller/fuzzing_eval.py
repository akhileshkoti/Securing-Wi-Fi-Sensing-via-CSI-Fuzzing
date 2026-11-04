#!/usr/bin/env python3
"""
Standalone evaluation harness for the CSI-fuzzing defense (Objective 3).

Deliberately NOT wired into app.py / csi_server.py -- kept separate so the
already-tested sensing pipeline is never at risk while this is developed and
measured. Run it on its own, against per-MAC CSV logs already captured by
csi_server.py (unchanged; this just reads its output).

WHY NO SECOND RECEIVER IS NEEDED
---------------------------------
Physically, a passive eavesdropper in range receives the same radio signal
your Receiver does. The only difference between "legitimate" and
"eavesdropper" is whether you know the PRNG seed to clean the data up
afterward -- not the listening hardware. So one capture, processed two
ways, stands in for both.

THE SUBTLETY THAT MATTERS
---------------------------------
The deployed Receiver (CsiCollector.cpp) already scopes CSI capture to known
MACs. That means it has always ignored the Guardian's traffic *by MAC*,
with no need for PRNG-based blanking at all -- so comparing "the
Illuminator's own clean log" against itself proves nothing; there was never
any Guardian noise mixed into that file to blank in the first place.

The meaningful comparison needs a stream where source separation ISN'T
assumed -- modelling a passive listener who can't or doesn't filter by MAC
(a realistic worst case for a cheap/naive eavesdropper). This script builds
that by merging the Illuminator's and the Guardian's own per-MAC CSVs into
one time-ordered, undifferentiated stream, the same way an eavesdropper's
radio would receive both without being told which is which. On that merged
stream it then computes:

  - "eavesdropper": every record used, nothing removed.
  - "legitimate (merged, PRNG-blanked)": burst-slot records excluded using
    the known schedule -- i.e. what the defense actually buys you even
    under the harder assumption that you can't rely on MAC separation.

If only an Illuminator CSV is given (no --guardian-csv), it falls back to
single-stream mode against one file -- useful for sanity-checking the
schedule logic itself, but as noted above, won't show a real effect unless
that one file already contains genuine Guardian-caused interference.

The schedule function (slot_hash) must stay byte-for-byte equivalent to
GuardianBeacon.cpp's slotHash() -- both sides compute it independently from
(seed, wall-clock time), no network exchange needed beyond sharing the seed.

Usage:
    # Full demonstration (recommended): merges both captures.
    python fuzzing_eval.py --illuminator-csv ../csi_logs/csi_<ILLUM_MAC>.csv \\
                            --guardian-csv    ../csi_logs/csi_<GUARDIAN_MAC>.csv \\
                            --seed 1234567

    # Harness/schedule sanity check only (single file, no real comparison):
    python fuzzing_eval.py --illuminator-csv ../csi_logs/csi_<ILLUM_MAC>.csv --seed 1234567
"""
import argparse
import csv
import math
from collections import deque

SLOT_MS = 20                 # must match GuardianSettings::SLOT_MS
DEFAULT_BURST_PROBABILITY = 0.15   # must match GuardianSettings::BURST_PROBABILITY
WINDOW_SECONDS = 1.0          # must match controller/app.py's WINDOW_SECONDS
MIN_WINDOW_FRAMES = 5          # must match controller/app.py's MIN_WINDOW_FRAMES


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
    deliberately rather than imported (see module docstring).

    NOTE on merged streams: when window mixes frames from two different
    physical links (Illuminator + Guardian), subcarrier index i from one
    frame isn't physically the same subcarrier as index i from the other --
    window_active_mask() just takes the shorter length and intersects
    blindly. That's intentional here, not an oversight: it models a naive
    eavesdropper applying one blind formula to an undifferentiated stream
    it doesn't know how to properly separate, not a rigorous per-link
    measurement."""
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
        # .strip(): tolerate a header/row with padded spaces (e.g. a CSV
        # re-saved from a spreadsheet app) -- without it this silently loads
        # zero records instead of erroring, which is worse than a crash.
        n_csi = sum(1 for h in header if h.strip().startswith("csi_"))
        for row in reader:
            ts = float(row[0])
            values = [int(v.strip()) for v in row[-n_csi:]] if n_csi else []
            if len(values) < 4:
                continue
            amps, actives = iq_to_amplitude(values)
            records.append((ts, amps, actives))
    return records


def load_merged_records(csv_paths):
    """Loads several per-MAC CSVs and merges them into one time-ordered
    stream -- what an eavesdropper who doesn't separate by source MAC would
    see, regardless of which physical node each record actually came from."""
    merged = []
    for path in csv_paths:
        merged.extend(load_records(path))
    merged.sort(key=lambda r: r[0])
    return merged


def rolling_cov_series(records, blank_bursts, seed, burst_probability):
    """Time-windowed CoV series -- mirrors controller/app.py's WINDOW_SECONDS/
    MIN_WINDOW_FRAMES pruning (see udp_listener), not a fixed frame count."""
    window = deque()
    series = []
    for ts, amps, actives in records:
        slot = int(ts * 1000 // SLOT_MS)
        if blank_bursts and is_burst_slot(seed, slot, burst_probability):
            continue
        window.append((ts, amps, actives))
        while len(window) > MIN_WINDOW_FRAMES and ts - window[0][0] > WINDOW_SECONDS:
            window.popleft()
        cov = compute_cov([(a, ac) for _, a, ac in window])
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
    ap.add_argument("--illuminator-csv", action="append", required=True,
                     help="per-MAC CSV log from csi_server.py for a real Illuminator; repeatable")
    ap.add_argument("--guardian-csv", default=None,
                     help="per-MAC CSV log for the Guardian's own traffic; omit for single-file mode")
    ap.add_argument("--seed", type=int, required=True,
                     help="shared PRNG seed (must match GuardianSettings::PRNG_SEED)")
    ap.add_argument("--burst-probability", type=float, default=DEFAULT_BURST_PROBABILITY,
                     help="must match GuardianSettings::BURST_PROBABILITY (default 0.15)")
    args = ap.parse_args()

    illum_paths = args.illuminator_csv
    illum_records = load_merged_records(illum_paths)
    print(f"loaded {len(illum_records)} Illuminator record(s) from {', '.join(illum_paths)}")

    if not args.guardian_csv:
        print("\nNo --guardian-csv given -- single-file mode. This sanity-checks the\n"
              "schedule/harness logic only; it will NOT show a real protection effect\n"
              "(see module docstring for why).\n")
        legit = rolling_cov_series(illum_records, blank_bursts=True,
                                    seed=args.seed, burst_probability=args.burst_probability)
        eaves = rolling_cov_series(illum_records, blank_bursts=False,
                                    seed=args.seed, burst_probability=args.burst_probability)
        summarize("blanked  ", legit)
        summarize("unblanked", eaves)
        return

    guardian_records = load_records(args.guardian_csv)
    print(f"loaded {len(guardian_records)} Guardian record(s) from {args.guardian_csv}")

    merged = illum_records + guardian_records
    merged.sort(key=lambda r: r[0])

    burst_count = sum(
        1 for ts, _, _ in merged
        if is_burst_slot(args.seed, int(ts * 1000 // SLOT_MS), args.burst_probability)
    )
    print(f"{burst_count}/{len(merged)} merged records ({burst_count / max(1, len(merged)) * 100:.1f}%) "
          f"fall in a guardian-burst slot\n")

    legit_scoped = rolling_cov_series(illum_records, blank_bursts=False,
                                       seed=args.seed, burst_probability=args.burst_probability)
    legit_merged_blanked = rolling_cov_series(merged, blank_bursts=True,
                                               seed=args.seed, burst_probability=args.burst_probability)
    eaves_merged = rolling_cov_series(merged, blank_bursts=False,
                                       seed=args.seed, burst_probability=args.burst_probability)

    print("Three views of the same capture session:")
    print("  1) what the deployed Receiver actually sees today (MAC-scoped --")
    print("     Guardian traffic was never mixed in, so nothing to blank)")
    print("  2) the harder, undifferentiated-stream case: merged traffic with")
    print("     PRNG-blanking applied -- what the defense recovers without relying")
    print("     on MAC separation")
    print("  3) the eavesdropper's view: merged traffic, nothing removed\n")
    summarize("1) legitimate, MAC-scoped (today's real pipeline)", legit_scoped)
    summarize("2) legitimate, merged + PRNG-blanked              ", legit_merged_blanked)
    summarize("3) eavesdropper, merged, nothing blanked          ", eaves_merged)


if __name__ == "__main__":
    main()
