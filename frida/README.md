# MFD Text Capture via Frida

## Overview

Orbiter's MFDs render text through `oapi::Sketchpad::Text()`, which in the
built-in GDI graphics engine calls Win32 `TextOutA` from `gdi32.dll`. By
hooking this function with Frida, we can capture every piece of text drawn
to any MFD without reimplementing the underlying orbital mechanics.

This eliminates a class of bugs where our own math diverges from Orbiter's
calculations (sign conventions, reference frame mismatches, node direction
conventions, etc.).

## How It Works

### Hook Target

```
gdi32.dll!TextOutA(HDC hdc, int x, int y, LPCSTR str, int len)
```

The GDI client's `GDIPad::Text` is a thin wrapper around this. Source:
`OVP/GDIClient/GDIClient.cpp:282`.

### Frida Script

```javascript
var gdi32 = Process.getModuleByName('gdi32.dll');
var pTextOutA = gdi32.getExportByName('TextOutA');

Interceptor.attach(pTextOutA, {
    onEnter: function(args) {
        var hdc = args[0].toUInt32();
        var x = args[1].toInt32();
        var y = args[2].toInt32();
        var len = args[4].toInt32();
        // readLatin1 needed for degree symbol (0xBA) and superscript 2 (0xB2)
        var str = readLatin1(args[3], len);
        send({hdc: hdc, x: x, y: y, text: str});
    }
});
```

Strings use Windows-1252 encoding (not UTF-8). The degree symbol `0xBA` and
superscript-2 `0xB2` require a byte-level reader:

```javascript
function readLatin1(ptr, len) {
    var buf = ptr.readByteArray(len);
    var bytes = new Uint8Array(buf);
    var str = '';
    for (var i = 0; i < bytes.length; i++) {
        str += String.fromCharCode(bytes[i]);
    }
    return str;
}
```

### Identifying MFD Panels

Each MFD draws to its own HDC (GDI device context). HDC values change
between sessions but are stable within a session. Identify MFDs by the
first text string drawn (at y=0):

| First string       | MFD                |
|---------------------|--------------------|
| `Align plane`       | Align Planes MFD   |
| `Orbit: <body>`     | Orbit MFD          |
| `Surface: <body>`   | Surface MFD        |
| `Map: <body>`       | Map MFD            |

Each HDC appears twice per frame (double-buffering). The content is nearly
identical between the two; use either.

Glass cockpit instruments (attitude indicator, HSI, ADI) also render via
TextOutA with their own HDCs, but contain only scale numbers without a
title string.

### Example Captured Output: Align Planes MFD

```
(  5,   0)  Align plane
(148,   1)  Ref
(187,   1)  Earth
(  5,  20)  Auto
( 55,  20)  Orbit
(148,  20)  Tgt
(187,  20)  ISS
(  5,  48)  Current     Target      Relative
(  5,  69)  Inc  52.31°  Inc  74.51°  RInc  24.24°
(  5,  88)  LAN 180.05°  LAN 169.03°  R  +0.000°/s
(  5, 126)  Node encounter
( 38, 145)  dA[°] TtN[s]
(  5, 166)  DN 146.4 2.32k
(  5, 185)  AN 326.4 5.00k
(  5, 343)  Thrust [320.0kN]
( 49, 362)  Dir   dV[m/s]  BT[s]   TtB[s]
(  5, 383)  DN  NML+  3.189k
(  5, 402)  AN  NML-  3.326k
```

### MFD Source Code Reference

The Align Planes MFD source is at `Src/Orbiter/MfdAlign.cpp`. Key details:

- `crossp(nm1, nm2)` = ascending node direction (line 211)
- AN always gets `NML-`, DN always gets `NML+` (lines 321, 338)
- Coordinates are relative to the MFD surface, not the screen
- `cw` and `ch` are character width/height in pixels

The text layout is deterministic — field positions are computed from `cw`
and `ch` multiplied by fixed column/row constants. Parsing can use y
coordinate ranges rather than regex.

## Running

```
python capture_mfd.py
```

Requires:
- Frida (`pip install frida`) — tested with 17.5.2
- Orbiter running with the built-in GDI graphics engine

The script auto-detects the Orbiter process, attaches, and dumps captured
text grouped by HDC every 2 seconds.

## Architecture Direction

### Current: Console Plugin (AccessibleMFD.dll)

- In-process Orbiter plugin
- User types commands, gets text responses
- Reimplements orbital mechanics calculations
- Handles both data queries and control commands

### Proposed: Split Read/Write

**Read (flight data):** Frida-based MFD reader with a web UI. Push-based,
always updating. Screen readers can consume ARIA live regions. No orbital
math to reimplement — Orbiter's own MFDs are the source of truth.

**Write (controls):** Keep the in-process plugin for commands that need the
Orbiter API (autopilot, throttle, warp, target selection). Could expose
these via a local HTTP endpoint so the web UI can send commands too.

### Why

The console's query commands (`orbit`, `align`, `fuel`, `surface`) largely
reimplement what Orbiter's MFDs already compute. This led to bugs where our
math diverged from Orbiter's conventions (e.g., ascending/descending node
identification). Reading MFD output directly eliminates this entire class
of errors.

The console remains valuable for control inputs that require the Orbiter
plugin API, which Frida cannot easily replace.

## Files

| File              | Purpose                                      |
|-------------------|----------------------------------------------|
| `hook_mfd_text.js`| Frida script — hooks TextOutA, sends text    |
| `capture_mfd.py`  | Python runner — attaches to Orbiter, prints   |
| `README.md`       | This file                                    |
