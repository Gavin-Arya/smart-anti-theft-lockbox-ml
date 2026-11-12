#!/usr/bin/env python3
import argparse
import csv
import shutil
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

CSV_PATH = Path(__file__).parent / "data.csv"
WINDOWS_PER_SESSION = 60
SKIP_FIRST = 2          
N_FEATURES = 15
FEATURES = ["acc_mean", "acc_std", "acc_max", "acc_peaks",
            "mic_peak", "mic_mean", "mic_peaks",
            "jerk_mean", "ax_std", "ay_std", "az_std",
            "gyro_mean", "gyro_max", "gyro_net", "acc_cross"]
HEADER = FEATURES + ["label", "session"]

PLAN = {
    "idle": [
        "Box still, room quiet.",
        "Box still, people talking nearby.",
        "Box still, music or a video playing.",
        "Box still, fan on or typing on the same table.",
        "Box still, you walk around it (do not touch it).",
        "Box still, TYPE HARD on a keyboard on the same table, right next to the box.",
        "Box still, tap / drum your fingers on the table beside the box (do not touch the box).",
        "Box still, bump the table lightly, move your chair, put a cup or phone down on the table.",
    ],
    "knock": [
        "Soft knocks on the lid, any rhythm, with pauses.",
        "Normal knocks on the lid (like your secret knock).",
        "Hard knocks on the lid.",
        "Knock on the sides and front of the box.",
        "Mix: knock-knock ... pause ... knock at different spots and strengths.",
        "Fast knocking, many knocks in a row on the lid.",
        "Knock with a knuckle, then with a pen or coin, different spots.",
    ],
    "shake": [
        "SHAKE: gentle back-and-forth shaking, box stays on the table (it must not travel far).",
        "SHAKE: hard back-and-forth shaking, hold the box and keep it in the SAME place (no walking).",
        "Hit / slap the box and bang the table.",
        "Try to pry or pull the lid open (box stays locked).",
        "SHAKE: irregular shaking in all directions, with short stops in between, stay in place.",
        "SHAKE: rapid small vibrations / rattling, hold it in the air but do not walk or turn.",
    ],
    "carried": [
        "CARRY: pick it up and walk slowly, turn corners.",
        "CARRY: tilt and rotate it slowly in your hands, then walk a few steps.",
        "CARRY: carry it fast / jog, in a smooth motion (no shaking on purpose).",
        "CARRY: lift, walk a few steps, put down, lift again (repeat).",
        "CARRY: carry it in different orientations (upside down, on its side), walking around the room.",
        "CARRY: carry it like a bag or under your arm, walking up/down stairs if you can.",
    ],
}


def find_port(given=None):
    if given:
        return given
    skip = ("bluetooth", "debug-console", "wlan", "airpods")
    keys = ("usbserial", "usbmodem", "slab", "cp210", "ch340", "wch", "ttyusb", "ttyacm")
    cands = []
    for p in list_ports.comports():
        text = (p.device + " " + (p.description or "")).lower()
        if any(k in text for k in skip):
            continue
        if any(k in text for k in keys) or p.device.upper().startswith("COM"):
            cands.append(p.device)
    if len(cands) == 1:
        return cands[0]
    print("Serial ports found:", [p.device for p in list_ports.comports()] or "none")
    return input("Type the port (Arduino IDE > Tools > Port): ").strip()


def parse(line):
    if line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) != N_FEATURES:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def read_window(ser, timeout=10):
    """Wait for the next valid window line; None if nothing arrives in time."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        vals = parse(line) if line else None
        if vals:
            return vals
    return None


def record(ser, n):
    """Returns the windows recorded (maybe fewer than n if the stream stops)."""
    rows = []
    try:
        ser.reset_input_buffer()
        for _ in range(SKIP_FIRST):           
            if read_window(ser) is None:
                print("  No data from the ESP32. Is logger.ino running?")
                return rows
        while len(rows) < n:
            vals = read_window(ser)
            if vals is None:
                print("  Lost the data stream.")
                break
            rows.append(vals)
            print(f"  {len(rows):2d}/{n}  acc_max={vals[2]:.2f}  gyro_max={vals[12]:.0f}  mic_peak={vals[4]:.0f}")
    except (serial.SerialException, OSError):
        print("  Serial connection lost (cable unplugged?).")
    except KeyboardInterrupt:
        print("  Interrupted.")
    return rows


def check_session(label, rows):
    """Quick sanity check that the activity really happened. Returns (ok, message)."""
    n = len(rows)
    if label == "knock":
        k = sum(1 for r in rows if r[4] > 80 or r[6] >= 1)
        return k >= 0.3 * n, f"{k}/{n} windows contain a knock sound"
    if label == "shake":
        k = sum(1 for r in rows if r[2] > 0.3)
        return k >= 0.3 * n, f"{k}/{n} windows show strong shaking"
    if label == "carried":
        k = sum(1 for r in rows if r[2] > 0.15 or r[12] > 30)
        return k >= 0.3 * n, f"{k}/{n} windows show carrying movement"
    return True, ""


def existing_sessions():
    done = {}
    if CSV_PATH.exists():
        with open(CSV_PATH, newline="") as f:
            first = f.readline().strip().split(",")
        if first != HEADER:
            sys.exit("\ndata.csv has the OLD format (7 features, 50 Hz logger). It cannot be mixed with the new data.\n"
                     "Run:  python3 collect.py --reset   (your old file is kept as data_old_backup.csv)\n")
        with open(CSV_PATH, newline="") as f:
            for r in csv.DictReader(f):
                done[r["session"]] = done.get(r["session"], 0) + 1
    return done


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/cu.usbserial-0001")
    ap.add_argument("--reset", action="store_true", help="delete data.csv (backup kept) and start over")
    args = ap.parse_args()

    print(f"Script : {Path(__file__).resolve()}")
    print(f"Data   : {CSV_PATH.resolve()}")
    if args.reset:
        if CSV_PATH.exists():
            shutil.copy(CSV_PATH, CSV_PATH.with_name("data_old_backup.csv"))
            CSV_PATH.unlink()
            print("Reset: old data deleted (backup saved as data_old_backup.csv).")
        else:
            print("Reset: there was no data.csv to delete.")
    done = existing_sessions()
    finished = [k for k, v in done.items() if v >= WINDOWS_PER_SESSION]
    n_all = sum(len(v) for v in PLAN.values())
    print(f"Sessions already complete: {len(finished)}/{n_all}")
    if len(finished) == n_all:
        print("\nEverything is already recorded, so there is nothing left to do.")
        print("To record again from scratch run:  python3 collect.py --reset")
        return
    if finished:
        print("(these are skipped:", ", ".join(sorted(finished)) + ")")
        print("(to start from scratch run: python3 collect.py --reset)\n")

    port = find_port(args.port)
    ser = serial.Serial()                 
    ser.port = port                       
    ser.baudrate = 115200
    ser.timeout = 5
    ser.dtr = False
    ser.rts = False
    ser.open()
    print("Checking the ESP32 is sending data (up to 15 s)...")
    if read_window(ser, timeout=15) is None:
        print(f"\nNO DATA on {port}. Check, in this order:")
        print("  1. Is this the right port? Compare with Arduino IDE > Tools > Port.")
        print("  2. Is logger.ino (not lockbox.ino) running? Open the Serial Monitor once to see lines")
        print("     every 1 s, then CLOSE it. If it is silent, press the EN button on the ESP32.")
        print("  3. Press the EN (reset) button now and run this script again.")
        return
    print("ESP32 is sending data.\n")
    done = existing_sessions()
    new_file = not CSV_PATH.exists()

    total = sum(len(v) for v in PLAN.values())
    print(f"\nPort: {port}   Output: {CSV_PATH}\n")

    try:
        for label, notes in PLAN.items():
            for s, note in enumerate(notes, start=1):
                name = f"{label}{s}"
                have = done.get(name, 0)
                if have >= WINDOWS_PER_SESSION:
                    continue
                need = WINDOWS_PER_SESSION - have
                while True:                    
                    print("=" * 60)
                    print(f"CLASS: {label.upper()}   session {s}/{len(notes)}   ({name})")
                    print(f"DO THIS: {note}")
                    if have:
                        print(f"({have}/{WINDOWS_PER_SESSION} already saved, recording the remaining {need})")
                    ans = input("Press Enter ONCE to start (s = skip, q = quit): ").strip().lower()
                    if ans == "q":
                        raise KeyboardInterrupt
                    if ans == "s":
                        break
                    for c in (3, 2, 1):
                        print(f"  starting in {c}...")
                        time.sleep(1)
                    print("  RECORDING - keep doing it until it says done")
                    rows = record(ser, need)
                    if len(rows) == need:
                        ok, msg = check_session(label, rows)
                        if not ok:
                            print(f"  WARNING: only {msg}. It looks like the activity did not happen.")
                            if input("  Keep it anyway? (y = keep, Enter = discard and redo): ").strip().lower() != "y":
                                print("  Discarded.\n")
                                continue
                    if rows:                   
                        with open(CSV_PATH, "a", newline="") as f:
                            w = csv.writer(f)
                            if new_file:
                                w.writerow(HEADER)
                                new_file = False
                            for vals in rows:
                                w.writerow(vals + [label, name])
                        done[name] = have + len(rows)
                    if len(rows) < need:
                        print(f"  Saved {len(rows)} windows (session {done.get(name, 0)}/{WINDOWS_PER_SESSION}).")
                        print("  Fix the connection, then run the script again to finish this session.")
                        raise KeyboardInterrupt
                    print(f"  DONE - saved {len(rows)} windows.\n")
                    break
    except KeyboardInterrupt:
        print("\nStopped. Run the script again to continue where you left off.")

    done = existing_sessions()
    print("\nRecords per class:")
    for label in PLAN:
        print(f"  {label:8s} {sum(v for k, v in done.items() if k.startswith(label))}")
    print(f"  TOTAL    {sum(done.values())}  (target {total * WINDOWS_PER_SESSION})")


if __name__ == "__main__":
    main()
