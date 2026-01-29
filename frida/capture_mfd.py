"""
Capture MFD text output from Orbiter using Frida.
Groups text by HDC (drawing surface) to identify MFD panels.

Usage:
    python capture_mfd.py
"""

import frida
import sys
import time
import os
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Collect text per HDC, flush periodically
surfaces = defaultdict(list)  # hdc -> [(x, y, text), ...]
last_flush = time.time()
FLUSH_INTERVAL = 2.0  # seconds between dumps

def find_orbiter_process():
    device = frida.get_local_device()
    for proc in device.enumerate_processes():
        name = proc.name.lower()
        if 'orbiter' in name or 'openorbiter' in name:
            return proc.pid, proc.name
    return None, None

def flush_surfaces():
    global surfaces, last_flush
    if not surfaces:
        return
    print(f"\n{'='*60}")
    print(f"Frame capture at {time.strftime('%H:%M:%S')}")
    print(f"{'='*60}")
    for hdc, texts in sorted(surfaces.items()):
        # Sort by y then x for reading order
        texts.sort(key=lambda t: (t[1], t[0]))
        print(f"\n--- HDC 0x{hdc:08X} ({len(texts)} strings) ---")
        for x, y, text in texts:
            print(f"  ({x:4d},{y:4d})  {text}")
    surfaces.clear()
    last_flush = time.time()

def on_message(message, data):
    global last_flush
    if message['type'] == 'send':
        p = message['payload']
        surfaces[p['hdc']].append((p['x'], p['y'], p['text']))
        if time.time() - last_flush > FLUSH_INTERVAL:
            flush_surfaces()
    elif message['type'] == 'error':
        print(f"[ERROR] {message}", file=sys.stderr)
    elif message['type'] == 'log':
        print(f"[LOG] {message['payload']}")

def main():
    pid, name = find_orbiter_process()
    if pid is None:
        print("Orbiter process not found. Is it running?")
        device = frida.get_local_device()
        for proc in device.enumerate_processes():
            if 'orbit' in proc.name.lower():
                print(f"  PID {proc.pid}: {proc.name}")
        return

    print(f"Attaching to {name} (PID {pid})...")
    session = frida.attach(pid)

    with open(os.path.join(SCRIPT_DIR, "hook_mfd_text.js")) as f:
        script_code = f.read()

    script = session.create_script(script_code)
    script.on('message', on_message)
    script.load()

    print(f"Capturing... (dumping every {FLUSH_INTERVAL}s, Ctrl+C to stop)")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        flush_surfaces()
        print("\nDetaching...")
        session.detach()

if __name__ == '__main__':
    main()
