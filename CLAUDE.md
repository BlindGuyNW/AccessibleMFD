# AccessibleMFD

Screen reader accessible console interface for Orbiter Space Simulator.

## Build

Requires Orbiter SDK. Build from Orbiter root:
```bash
cmake -B build -G "Visual Studio 17 2022" -A Win32
cmake --build build --target AccessibleMFD --config Release
```

Output: `Modules/Plugin/AccessibleMFD.dll`

## Architecture

- Console-based UI via `AllocConsole()` - no graphics to parse
- On-demand queries - user types command, gets text response
- Accessed via Ctrl+F4 "Custom Functions" menu in Orbiter

## Commands

| Command | Description |
|---------|-------------|
| `v`, `vessel` | Vessel name, class, reference body |
| `o`, `orbit` | Altitude, apoapsis, periapsis, inclination, eccentricity, period |
| `f`, `flight` | Velocity, heading, pitch, bank |
| `m`, `mfd` | Left/right MFD modes |
| `d`, `dock` | NAV target, distance, closure rate, relative velocities |
| `fuel` | Fuel mass, capacity, percentage |
| `a`, `all` | All data queries combined |
| `na [mode]` | Autopilot: pro/retro/nml/anml/kill/level/halt/off |
| `th [n]` | Throttle 0-100, or `th main/retro/hover n` |
| `warp [n]` | Time warp (0.1 = slow motion, 1 = normal, max 100000) |
| `?`, `help` | Show help |
| `q`, `quit` | Close console |

## Key APIs

- `oapiGetFocusInterface()` - get current vessel
- `VESSEL::GetElements()` - orbital parameters
- `VESSEL::GetNavmodeState/ToggleNavmode()` - autopilot control
- `VESSEL::GetEngineLevel/SetEngineLevel()` - engine throttle
- `oapiGetTimeAcceleration/oapiSetTimeAcceleration()` - time warp

## Future Ideas

- RCS control (`rcs rot/lin/off`)
- Targeting (`target <name>`, `target list`)
- HUD/MFD mode switching
- Atmospheric data (Mach, dynamic pressure, lift/drag)
