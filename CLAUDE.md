# AccessibleMFD

Screen reader accessible console interface for Orbiter Space Simulator.

## Build

Standalone build (can be located anywhere):
```bash
cmake -B build -G "Visual Studio 17 2022" -A Win32 -DORBITER_DIR=C:/YourOrbiterPath
cmake --build build --config Release
```

Or set `ORBITER_DIR` environment variable, or edit the default in CMakeLists.txt.

Output: `${ORBITER_DIR}/Modules/Plugin/AccessibleMFD.dll`

## Orbiter SDK Reference

Orbiter installation: `C:/Orbit`

Key locations for reference:
- `C:/Orbit/Orbitersdk/include/` - API headers
- `C:/Orbit/Orbitersdk/samples/` - Sample modules
- `C:/Orbit/Orbitersdk/doc/` - API documentation
- `C:/Orbit/Config/` - Vessel and scenario configs

## Architecture

- Console-based UI via `AllocConsole()` - no graphics to parse
- On-demand queries - user types command, gets text response
- Accessed via Ctrl+F4 "Custom Functions" menu in Orbiter

### Source Files

| File | Purpose |
|------|---------|
| `AccessibleMFD.cpp` | Module entry points (InitModule/ExitModule) |
| `Console.cpp/.h` | Console window, thread, input loop |
| `Commands.cpp/.h` | Command registry and dispatch table |
| `Formatting.cpp/.h` | Distance, time, lat/lon formatting helpers |
| `OrbitalCalc.cpp/.h` | Hohmann, phase angle, plane change calculations |
| `Queries.cpp/.h` | Data queries (vessel, orbit, flight, surface, fuel, map) |
| `Controls.cpp/.h` | Control commands (autopilot, throttle, warp) |
| `Transfer.cpp/.h` | Transfer planner and target management |

## Commands

### Data Queries
| Command | Description |
|---------|-------------|
| `v`, `vessel` | Vessel name, class, reference body |
| `o`, `orbit` | Altitude, apoapsis, periapsis, inclination, eccentricity, period |
| `f`, `flight` | Velocity, heading, pitch, bank |
| `sf`, `surface` | Altitude, speed, attitude, atmosphere, aerodynamics |
| `sf alt` | Detailed altitude, speed, and attitude |
| `sf atm` | Atmospheric conditions (Mach, temperature, pressure) |
| `sf forces` | Aerodynamic forces (lift, drag, L/D ratio) |
| `m`, `mfd` | Left/right MFD modes |
| `d`, `dock` | NAV target, distance, closure rate, relative velocities |
| `fuel` | Fuel mass, capacity, percentage |
| `map` | Position, altitude, ground track |
| `map bases` | List nearby bases with distance/bearing |
| `a`, `all` | All data queries combined |

### Transfer Planner
| Command | Description |
|---------|-------------|
| `tgt` | Show current target |
| `tgt <name>` | Set target (body or vessel) |
| `tgt list` | List celestial bodies and vessels |
| `tgt clear` | Clear target |
| `tr` | Transfer summary (Hohmann, phase, plane change) |
| `tr hohmann` | Hohmann transfer delta-v and time |
| `tr phase` | Phase angle to transfer window |
| `tr plane` | Plane change requirements |
| `tr ren` | Rendezvous data (vessel targets) |

### Control
| Command | Description |
|---------|-------------|
| `na [mode]` | Autopilot: pro/retro/nml/anml/kill/level/halt/off |
| `th [n]` | Throttle 0-100, or `th main/retro/hover n` |
| `warp [n]` | Time warp (0.1 = slow motion, 1 = normal, max 100000) |

### System
| Command | Description |
|---------|-------------|
| `?`, `help` | Show help |
| `q`, `quit` | Close console |

## Key APIs

- `oapiGetFocusInterface()` - get current vessel
- `VESSEL::GetElements()` - orbital parameters
- `VESSEL::GetNavmodeState/ToggleNavmode()` - autopilot control
- `VESSEL::GetEngineLevel/SetEngineLevel()` - engine throttle
- `oapiGetTimeAcceleration/oapiSetTimeAcceleration()` - time warp
- `oapiGetGbodyByName/oapiGetVesselByName()` - target lookup
- `oapiGetGlobalPos/oapiGetGlobalVel()` - position/velocity for calculations
- `VESSEL::GetAtmPressure/GetAtmTemperature/GetAtmDensity()` - atmospheric data
- `VESSEL::GetMachNumber/GetDynPressure()` - aerodynamic state
- `VESSEL::GetLift/GetDrag/GetAOA/GetSlipAngle()` - aerodynamic forces

## Future Ideas

- RCS control (`rcs rot/lin/off`)
- HUD/MFD mode switching
