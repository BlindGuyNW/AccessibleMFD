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
