#!/usr/bin/env python3
"""
Wireless version of collect.py: receives the windows over MQTT.
Same plan (27 sessions x 60 windows = 1620 records, 15 features), same data.csv.

  pip install paho-mqtt pandas scikit-learn numpy pyserial
  python collect_mqtt.py

Upload logger_wifi/logger_wifi.ino first and power the ESP32 from the power bank.
Needs collect.py in the same folder (it reuses its plan).
"""
import csv
import queue
import time

import paho.mqtt.client as mqtt

from collect import (CSV_PATH, HEADER, PLAN, WINDOWS_PER_SESSION, SKIP_FIRST,
                     existing_sessions, parse, check_session)

BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC = "lockbox_gavin_7f3a_rec/window"     

windows = queue.Queue()


def on_connect(client, userdata, *args):
    client.subscribe(TOPIC)                  


def on_message(client, userdata, msg):
    windows.put(msg.payload.decode(errors="ignore").strip())


def make_client():
    try:                                     
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:                   
        c = mqtt.Client()
    c.on_connect = on_connect
    c.on_message = on_message
    return c


def read_window(timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            line = windows.get(timeout=1)
        except queue.Empty:
            continue
        vals = parse(line)
        if vals:
            return vals
    return None


def record(n):
    """Returns the windows recorded (maybe fewer than n if the stream stops)."""
    rows = []
    try:
        while not windows.empty():           
            windows.get_nowait()
        for _ in range(SKIP_FIRST):          
            if read_window(30) is None:
                print("  No data received. Is the ESP32 on and connected to Wi-Fi?")
                return rows
        while len(rows) < n:
            vals = read_window(15)
            if vals is None:
                print("  Lost the data stream (Wi-Fi or broker problem).")
                break
            rows.append(vals)
            print(f"  {len(rows):2d}/{n}  acc_max={vals[2]:.2f}  gyro_max={vals[12]:.0f}  mic_peak={vals[4]:.0f}")
    except KeyboardInterrupt:
        print("  Interrupted.")
    return rows


def main():
    client = make_client()
    client.connect(BROKER, PORT, 60)
    client.loop_start()
    print(f"\nBroker: {BROKER}   Topic: {TOPIC}   Output: {CSV_PATH}")
    print("Waiting for the ESP32 (up to 30 s)...")
    if read_window(30) is None:
        print("Nothing received yet. Check the ESP32 Serial Monitor for '# logger ready'. Continuing anyway.")
    else:
        print("ESP32 is sending data.\n")

    done = existing_sessions()
    new_file = not CSV_PATH.exists()
    total = sum(len(v) for v in PLAN.values()) * WINDOWS_PER_SESSION

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
                    rows = record(need)
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
    finally:
        client.loop_stop()

    done = existing_sessions()
    print("\nRecords per class:")
    for label in PLAN:
        print(f"  {label:8s} {sum(v for k, v in done.items() if k.startswith(label))}")
    print(f"  TOTAL    {sum(done.values())}  (target {total})")


if __name__ == "__main__":
    main()
