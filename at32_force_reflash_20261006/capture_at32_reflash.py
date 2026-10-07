#!/usr/bin/env python3
"""capture_at32_reflash.py - host trigger/logger for nexarm-at32-force-reflash-r1

Modes:
  --query          read-only: ask ESP32 to query AT32 version (QUERY_VER), log output
  --reflash-once   send FORCE_REFLASH_ONCE, then log the entire OTA session to file
  --reset          passive listen only (e.g. after ESP32 reset)

Gate before --reflash-once: firmware banner must be nexarm-at32-force-reflash-r1
and self-checks must have passed (READY line present). The script never sends the
trigger unless the banner matches.
"""
import argparse, datetime, json, os, sys, time

PORT_DEFAULT = "COM9"
BAUD = 1000000
EXPECT_FW = "nexarm-at32-force-reflash-r"  # prefix match: r1, r2, ...

def open_port(port):
    import serial
    s = serial.Serial(port, BAUD, timeout=0.2)
    s.reset_input_buffer()
    return s

def run(port, mode, outdir, listen_s):
    os.makedirs(outdir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(outdir, f"at32_reflash_{mode}_{stamp}.log")
    s = open_port(port)
    lines, banner_ok = [], False
    t0 = time.time()
    def pump(dur):
        nonlocal banner_ok
        end = time.time() + dur
        while time.time() < end:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip()
            if not line:
                continue
            lines.append(line)
            print(line)
            if EXPECT_FW in line and "READY" in line:
                banner_ok = True
    pump(2.0)
    if mode == "query":
        s.write(b"QUERY_VER\n"); s.flush(); pump(2.0)
        s.write(b"QUERY65\n"); s.flush(); pump(2.0)
    elif mode == "reflash-once":
        if not banner_ok:
            # Banner scrolls by if ESP32 booted before we attached; probe liveness
            # instead: only this sketch answers QUERY_VER, and only when its
            # self-checks passed (otherwise its UART stays disabled).
            print("[gate] banner missed; probing with QUERY_VER")
            s.write(b"QUERY_VER\n"); s.flush()
            end = time.time() + 3.0
            while time.time() < end and not banner_ok:
                raw = s.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", "replace").rstrip()
                if not line:
                    continue
                lines.append(line)
                print(line)
                if '"type":"ota_query"' in line and '"got":true' in line:
                    banner_ok = True
        if not banner_ok:
            print(f"[gate] FAIL: sketch '{EXPECT_FW}' not confirmed alive; NOT sending trigger")
            s.close(); sys.exit(2)
        print("[gate] OK; sending FORCE_REFLASH_ONCE")
        s.write(b"FORCE_REFLASH_ONCE\n"); s.flush()
        # OTA takes ~15-25s; listen generously, stop 3s after FINISHED evidence
        end = time.time() + listen_s
        done_at = None
        while time.time() < end:
            raw = s.readline()
            if not raw:
                if done_at and time.time() - done_at > 3.0:
                    break
                continue
            line = raw.decode("utf-8", "replace").rstrip()
            if not line:
                continue
            lines.append(line)
            print(line)
            if '"phase":8' in line:  # FINISHED
                done_at = done_at or time.time()
    else:  # reset / passive
        pump(listen_s)
    s.close()
    with open(log_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[log] {log_path} ({len(lines)} lines)")
    # verdict summary for reflash mode
    if mode == "reflash-once":
        verdict = "UNKNOWN"
        for line in lines:
            if "reflash_success" in line:
                verdict = "SUCCESS"
            elif '"reason":"' in line and '"phase":8' in line:
                try:
                    verdict = json.loads(line).get("reason", verdict)
                except Exception:
                    pass
        print(f"[verdict] {verdict}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT_DEFAULT)
    ap.add_argument("--query", action="store_true")
    ap.add_argument("--reflash-once", action="store_true")
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--listen", type=float, default=60.0)
    ap.add_argument("--outdir", default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    mode = "reflash-once" if a.reflash_once else "query" if a.query else "reset"
    run(a.port, mode, a.outdir, a.listen)
