"""
Interactive MFD reader for Orbiter using Frida.

Continuously captures MFD text in the background and provides
a command loop for querying MFD contents and button labels.

Commands:
    list              Show active MFDs (left/right) and their titles
    read <l|r>        Show current content of left or right MFD
    watch <l|r>       Continuously print MFD updates (Ctrl+C to stop)
    buttons <l|r>     Show button labels for left/right MFD
    press <l|r> <#>   Press a button by number
    help              Show commands
    quit              Exit

Usage:
    python mfd_reader.py
"""

import frida
import sys
import time
import os
import threading
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Orbiter MFD identifiers
MFD_LEFT = 0
MFD_RIGHT = 1

# HDC is considered inactive after this many seconds without text
ACTIVITY_TIMEOUT = 2.0


class MFDReader:
    def __init__(self):
        # HDC -> list of (x, y, text) for current frame
        self._pending = defaultdict(list)
        # HDC -> (x, y) position of the frame-start text (MFD title)
        self._hdc_frame_pos = {}
        # HDC -> title string (identified from frame-start text)
        self._hdc_titles = {}
        # HDC -> monotonic timestamp of last text received
        self._hdc_last_seen = {}
        # HDCs in order of first appearance (left MFD renders first)
        self._hdc_order = []
        self._lock = threading.Lock()
        self._session = None
        self._script = None
        # RPC response tracking
        self._rpc_event = threading.Event()
        self._rpc_result = None
        self._has_buttons = False

    def attach(self):
        device = frida.get_local_device()
        pid = None
        for proc in device.enumerate_processes():
            if 'orbiter' in proc.name.lower():
                pid = proc.pid
                name = proc.name
                break
        if pid is None:
            print("Orbiter not found. Is it running?")
            return False

        print(f"Attaching to {name} (PID {pid})...")
        self._session = frida.attach(pid)

        with open(os.path.join(SCRIPT_DIR, "hook_mfd_text.js")) as f:
            script_code = f.read()

        self._script = self._session.create_script(script_code)
        self._script.on('message', self._on_message)
        self._script.load()
        print("Hooked. Capturing MFD text.\n")
        return True

    def _on_message(self, message, data):
        if message['type'] == 'send':
            p = message['payload']
            msg_type = p.get('type', 'text')

            if msg_type == 'text':
                hdc = p['hdc']
                x, y, text = p['x'], p['y'], p['text']

                with self._lock:
                    self._hdc_last_seen[hdc] = time.monotonic()

                    # Track HDC discovery order (left MFD renders first)
                    if hdc not in self._hdc_frame_pos and y <= 1 and x < 20:
                        self._hdc_order.append(hdc)

                    entries = self._pending[hdc]

                    # Detect new frame: only flush when we see text at the
                    # same (x,y) position as this HDC's title. This avoids
                    # mid-frame flushes from MFDs like TransX that draw
                    # multiple texts at y=0 (title + "Stage 1:1").
                    frame_pos = self._hdc_frame_pos.get(hdc)
                    if frame_pos and (x, y) == frame_pos and len(entries) > 0:
                        self._pending[hdc] = []
                        entries = self._pending[hdc]
                        # Update title text if it changed
                        self._hdc_titles[hdc] = text

                    entries.append((x, y, text))

                    # Identify frame-start position from first text near top-left
                    if y <= 1 and x < 20 and hdc not in self._hdc_frame_pos:
                        self._hdc_frame_pos[hdc] = (x, y)
                        self._hdc_titles[hdc] = text

            elif msg_type == 'rpc_result':
                self._rpc_result = p.get('result', {})
                self._rpc_event.set()

        elif message['type'] == 'log':
            msg = message.get('payload', '')
            # Track whether button API was found
            if 'Found button label' in msg:
                self._has_buttons = True
                print(f"  [API] {msg}")
            elif 'Found send key' in msg:
                print(f"  [API] {msg}")
            elif 'Found process button' in msg:
                print(f"  [API] {msg}")
            elif 'Could not find' in msg or 'MFD-related exports' in msg:
                print(f"  [API] {msg}")
            elif msg.startswith('  '):
                # Indented export listing
                print(f"  [API] {msg}")

    def _rpc_call(self, cmd, *args):
        """Send an RPC call to the Frida script and wait for result."""
        self._rpc_event.clear()
        self._rpc_result = None
        self._script.post({'type': 'rpc', 'cmd': cmd, 'args': list(args)})
        if self._rpc_event.wait(timeout=3.0):
            return self._rpc_result
        return {'error': 'Timeout waiting for response'}

    def _get_active_hdcs(self):
        """Return active HDCs in discovery order (left MFD first).

        An HDC is active if it received text recently and has enough
        entries to be a real MFD (not noise).
        """
        now = time.monotonic()
        return [
            hdc for hdc in self._hdc_order
            if now - self._hdc_last_seen.get(hdc, 0) < ACTIVITY_TIMEOUT
            and len(self._pending.get(hdc, [])) > 3
        ]

    def _get_mfd_hdc(self, mfd_id):
        """Get the HDC for left (0) or right (1) MFD, or None."""
        active = self._get_active_hdcs()
        if mfd_id < len(active):
            return active[mfd_id]
        return None

    def _get_mfd_entries(self, mfd_id):
        """Get (title, sorted_entries) for left or right MFD, or None."""
        hdc = self._get_mfd_hdc(mfd_id)
        if hdc is None:
            return None
        title = self._hdc_titles.get(hdc, "Unknown")
        entries = sorted(self._pending.get(hdc, []), key=lambda t: (t[1], t[0]))
        return title, list(entries)

    def list_mfds(self):
        """List currently active MFDs."""
        with self._lock:
            active = self._get_active_hdcs()
        if not active:
            print("No MFDs active. Wait a moment and try again.")
            return
        sides = ["Left", "Right"]
        for i, hdc in enumerate(active):
            side = sides[i] if i < len(sides) else f"MFD {i}"
            title = self._hdc_titles.get(hdc, "Unknown")
            count = len(self._pending.get(hdc, []))
            print(f"  {side}: {title}  ({count} fields)")

    def read_mfd(self, arg):
        """Read the current content of the left or right MFD."""
        mfd_id, mfd_name, _ = self._parse_mfd_side(arg)
        if mfd_id is None:
            print("Usage: read <left|right>  (or l/r)")
            return

        with self._lock:
            result = self._get_mfd_entries(mfd_id)
        if result is None:
            print(f"No {mfd_name.lower()} active.")
            return
        title, entries = result
        print(f"\n=== {title} ({mfd_name}) ===")
        self._print_entries(entries)

    def watch_mfd(self, arg):
        """Continuously print updates for the left or right MFD."""
        mfd_id, mfd_name, _ = self._parse_mfd_side(arg)
        if mfd_id is None:
            print("Usage: watch <left|right>  (or l/r)")
            return

        print(f"Watching: {mfd_name}  (Ctrl+C to stop)\n")
        last_text = None
        try:
            while True:
                time.sleep(0.5)
                with self._lock:
                    result = self._get_mfd_entries(mfd_id)
                if result is None:
                    continue
                title, entries = result
                current_text = [(x, y, t) for x, y, t in entries]
                if current_text != last_text:
                    os.system('cls' if os.name == 'nt' else 'clear')
                    print(f"=== {title} ({mfd_name}) === (Ctrl+C to stop)\n")
                    self._print_entries(entries)
                    last_text = current_text
        except KeyboardInterrupt:
            print("\nStopped watching.")

    def _parse_mfd_side(self, arg):
        """Parse 'left'/'right' or 'l'/'r' into (mfd_id, mfd_name) or None."""
        parts = arg.strip().lower().split()
        side = parts[0] if parts else ''
        if side in ('l', 'left', '0'):
            return MFD_LEFT, "Left MFD", parts[1:]
        elif side in ('r', 'right', '1'):
            return MFD_RIGHT, "Right MFD", parts[1:]
        return None, None, None

    def press_button(self, arg):
        """Press a button by number on left or right MFD."""
        mfd_id, mfd_name, rest = self._parse_mfd_side(arg)
        if mfd_id is None:
            print("Usage: press <left|right> <button#>")
            return

        if not rest:
            print("Usage: press <left|right> <button#>")
            return

        try:
            bt = int(rest[0])
        except ValueError:
            print(f"Invalid button number: {rest[0]}")
            return

        result = self._rpc_call('press', mfd_id, bt)
        if 'error' in result:
            print(f"Error: {result['error']}")
        else:
            # Show updated buttons after press
            self.show_buttons(arg.split()[0])

    def show_buttons(self, side):
        """Show button labels for left or right MFD."""
        mfd_id, mfd_name, _ = self._parse_mfd_side(side)
        if mfd_id is None:
            print("Usage: buttons <left|right>  (or l/r)")
            return

        result = self._rpc_call('buttons', mfd_id)
        if 'error' in result:
            print(f"Error: {result['error']}")
            return

        buttons = result.get('buttons', [])
        if not buttons:
            print(f"{mfd_name}: No buttons active")
            return

        print(f"\n{mfd_name} buttons:")
        left_btns = [b for b in buttons if b['side'] == 'L']
        right_btns = [b for b in buttons if b['side'] == 'R']

        if left_btns:
            print("  Left column:")
            for b in left_btns:
                print(f"    [{b['index']}] {b['label']}")
        if right_btns:
            print("  Right column:")
            for b in right_btns:
                print(f"    [{b['index']}] {b['label']}")

    def _print_entries(self, entries):
        """Print MFD text entries in a readable format."""
        lines = []
        current_line = []
        last_y = None
        Y_TOLERANCE = 8

        for x, y, text in entries:
            if last_y is not None and abs(y - last_y) > Y_TOLERANCE:
                if current_line:
                    lines.append(current_line)
                current_line = []
            current_line.append((x, y, text))
            last_y = y

        if current_line:
            lines.append(current_line)

        for line_entries in lines:
            line_entries.sort(key=lambda e: e[0])
            parts = [text for x, y, text in line_entries]
            print("  ".join(parts))

    def detach(self):
        if self._session:
            self._session.detach()


def main():
    reader = MFDReader()
    if not reader.attach():
        return

    print("Commands: list, read <l|r>, watch <l|r>, buttons <l|r>, press <l|r> <#>, help, quit\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break

            if not line:
                continue

            parts = line.split(None, 1)
            cmd = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else ""

            if cmd in ('quit', 'exit', 'q'):
                break
            elif cmd in ('list', 'ls'):
                reader.list_mfds()
            elif cmd in ('read', 'r'):
                if not arg:
                    print("Usage: read <left|right>  (or l/r)")
                else:
                    reader.read_mfd(arg)
            elif cmd in ('watch', 'w'):
                if not arg:
                    print("Usage: watch <left|right>  (or l/r)")
                else:
                    reader.watch_mfd(arg)
            elif cmd in ('buttons', 'btn', 'b'):
                if not arg:
                    print("Usage: buttons <left|right>  (or l/r)")
                else:
                    reader.show_buttons(arg)
            elif cmd in ('press', 'p'):
                if not arg:
                    print("Usage: press <left|right> <button#>")
                else:
                    reader.press_button(arg)
            elif cmd in ('help', '?'):
                print("Commands:")
                print("  list              Show active MFDs and their titles")
                print("  read <l|r>        Show current content of left/right MFD")
                print("  watch <l|r>       Live updates (Ctrl+C to stop)")
                print("  buttons <l|r>     Show button labels for left/right MFD")
                print("  press <l|r> <#>   Press button by number")
                print("  quit              Exit")
            else:
                print(f"Unknown command: {cmd}. Type 'help' for commands.")

    except KeyboardInterrupt:
        pass

    print("\nDetaching...")
    reader.detach()


if __name__ == '__main__':
    main()
