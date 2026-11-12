#!/usr/bin/env python3
"""
List every recorded session with simple quality stats, and optionally delete bad ones.

  python3 check_data.py                    show the table
  python3 check_data.py --delete knock3 knock5   remove those sessions (backup is kept)
"""
import argparse
import shutil
from pathlib import Path

import pandas as pd

CSV = Path(__file__).parent / "data.csv"
ap = argparse.ArgumentParser()
ap.add_argument("--delete", nargs="+", metavar="SESSION")
args = ap.parse_args()

d = pd.read_csv(CSV)
if args.delete:
    shutil.copy(CSV, CSV.with_name("data_backup.csv"))
    d = d[~d["session"].isin(args.delete)]
    d.to_csv(CSV, index=False)
    print(f"Deleted {args.delete}. Backup saved as data_backup.csv\n")

def status(g):
    lab, n = g["label"].iloc[0], len(g)
    if lab == "knock":
        ok = ((g.mic_peak > 80) | (g.mic_peaks >= 1)).sum() >= 0.3 * n
    elif lab == "shake":
        ok = (g.acc_max > 0.3).sum() >= 0.3 * n
    elif lab == "carried":
        ok = ((g.acc_max > 0.15) | (g.gyro_max > 30)).sum() >= 0.3 * n
    else:
        ok = (g.acc_max < 0.3).sum() >= 0.8 * n          # idle should be calm
    return "ok" if ok else "SUSPICIOUS - redo?"

t = d.groupby("session").apply(lambda g: pd.Series({
    "label": g["label"].iloc[0], "rows": len(g),
    "mic_peak_max": g.mic_peak.max(), "mic_peaks_mean": round(g.mic_peaks.mean(), 2),
    "acc_max_max": round(g.acc_max.max(), 2), "acc_max_mean": round(g.acc_max.mean(), 3),
    "gyro_max_mean": round(g.gyro_max.mean(), 1), "gyro_net_mean": round(g.gyro_net.mean(), 1),
    "status": status(g)}), include_groups=False)
print(t.sort_values(["label", "session"]).to_string())
print("\nRows per class:\n", d["label"].value_counts().to_string())
