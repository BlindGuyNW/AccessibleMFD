# Orbiter MFD Internals — Accessing MFD Data from an In-Process DLL

## Overview

Orbiter's public SDK exposes limited MFD access: you can query the current mode
(`oapiGetMFDMode`), send keys (`oapiSendMFDKey`), and read button labels
(`oapiMFDButtonLabel`). But you cannot read the computed data an MFD displays.

Since AccessibleMFD is a DLL loaded into Orbiter's process, it shares the full
address space. The Orbiter 2024 source reveals that MFD instances store rich
computed data in member variables — orbital elements, flight parameters,
distances, targets — all accessible if you know the internal class layout.

## Entry Points

### Global Pointers

Defined in `Src/Orbiter/Orbiter.cpp:92-139`, these globals are extern-declared
across 80+ source files and resolve automatically for any in-process DLL:

```cpp
extern Orbiter *g_pOrbiter;   // Main application object
extern Pane    *g_pane;       // Panel/MFD/HUD display manager
extern Vessel  *g_focusobj;   // Current focus vessel (internal class)
```

### The Pane → MFD Path

The `Pane` class (defined in `Src/Orbiter/Pane.h`) manages all cockpit displays
including MFDs. It holds an array of 12 MFD slots:

```cpp
// Pane.h, lines 33-40
struct MFDspec {
    Instrument *instr;      // Pointer to the live MFD instance
    int lastmode;           // Last active mode
    bool exist;             // MFD defined for current panel?
    bool active;            // MFD switched on?
    double upDTscale;       // Refresh interval multiplier
    EXTMFDSPEC prm;         // MFD geometry parameters
};

// Pane.h, line 296
MFDspec mfd[MAXMFD];        // MAXMFD = 12
```

Public accessor (`Pane.h:211`, implemented in `Pane.cpp:1094-1104`):

```cpp
Instrument *Pane::MFD(int which);  // Returns mfd[which].instr
```

### MFD Display Identifiers

```cpp
#define MFD_LEFT    0
#define MFD_RIGHT   1
#define MFD_USER1   2   // through MFD_USER10 = 11
#define MAXMFD     12
```

## The Instrument Base Class

Defined in `Src/Orbiter/Mfd.h`. All built-in MFD modes inherit from this.

### Key Members

```cpp
class Instrument {
protected:
    INT_PTR id;             // Instrument identifier
    Vessel *vessel;         // Associated vessel (internal class)
    int IW, IH;             // Display dimensions (pixels)
    SURFHANDLE surf;        // Rendering surface
    double instrDT;         // Update interval
    double updT;            // Next update time

    // Button state
    int nbtl, nbtr;         // Button count (left/right side)
    int nbt;                // Total buttons per page
    int btnpage;            // Current button page

public:
    Vessel *GetVessel();
    SURFHANDLE Surface();
    int Type();             // Returns MFD mode ID (MFD_ORBIT, etc.)
    const char *BtnLabel(int bt);
    bool Update(double upDTscale);
};
```

### Static Mode Registry

The `Instrument` class maintains a global registry of all MFD modes:

```cpp
static DWORD nGlobalModes;      // Total registered modes
static MFDMODE *GlobalMode;     // Array of mode definitions

struct MFDMODE {
    MFDMODESPECEX *spec;    // Mode specification (name, msgproc, etc.)
    MFDMODESPEC *oldspec;   // Legacy specification
    int id;                 // Mode identifier
};
```

## Built-In MFD Mode Data

Each built-in mode is a derived class storing its computed values as member
variables. The display text is generated on-the-fly in `UpdateDraw()` via
`sprintf` directly to a `Sketchpad` — the formatted strings are not stored, but
the underlying numerical data persists in member fields between frames.

### Standard Mode IDs

```cpp
#define MFD_ORBIT       1
#define MFD_SURFACE     2
#define MFD_MAP         3
#define MFD_HSI         4
#define MFD_LANDING     5
#define MFD_DOCKING     6
#define MFD_OPLANEALIGN 7
#define MFD_OSYNC       8
#define MFD_TRANSFER    9
#define MFD_COMMS      10
#define MFD_USERTYPE   64   // Custom/addon modes start here
```

### Instrument_Orbit (MFD_ORBIT)

Source: `Src/Orbiter/MfdOrbit.h`, `MfdOrbit.cpp`

```cpp
class Instrument_Orbit : public Instrument {
    Elements *shpel;            // Ship orbital elements
    Elements *tgtel;            // Target orbital elements (if target set)
    const CelestialBody *elref; // Reference body
    const Body *tgt;            // Target object (vessel or body)
    double scale;               // Display scale factor

    // Display mode settings
    ProjectionMode projmode;    // Projection type
    FrameMode frmmode;          // Reference frame
    DistMode dstmode;           // Distance display mode
    DisplayMode dispmode;       // What to show
};
```

The `Elements` structure contains the full Keplerian elements: semi-major axis
(`a`), eccentricity (`e`), inclination (`i`), LAN (`theta`), argument of
periapsis (`omegab`), mean longitude (`L`), plus derived values accessible via
methods like `SMi()` (semi-minor axis), `ApDist()`, `PeDist()`, etc.

### Instrument_Surface (MFD_SURFACE)

Source: `Src/Orbiter/MfdSurface.h`, `MfdSurface.cpp`

```cpp
class Instrument_Surface : public Instrument {
    double alt;         // Altitude
    double spd;         // Airspeed
    double acc;         // Acceleration
    double vspd;        // Vertical speed
    double vacc;        // Vertical acceleration
    double dir;         // Heading
    double pitch;       // Pitch angle
    double bank;        // Bank angle
    double aoa;         // Angle of attack
    double psimt;       // Sideslip angle
    double lng, lat;    // Geographic position
    double vlng, vlat;  // Velocity in geographic coordinates
    double atm_p;       // Atmospheric pressure
    double atm_rho;     // Atmospheric density
    double atm_T;       // Atmospheric temperature
    double atm_M;       // Mach number
    int spdmode;        // Speed indicator mode
};
```

### Instrument_Docking (MFD_DOCKING)

Source: `Src/Orbiter/MfdDocking.h`, `MfdDocking.cpp`

```cpp
class Instrument_Docking : public Instrument {
    enum SensorMode { UNDEF, NAV, VISUAL };
    SensorMode smode;           // Data acquisition method
    DWORD refdock;              // Reference docking port index
    const Body *Legacy_ref;     // Target station/vessel
    double dst;                 // Distance to target
    Vector adpos;               // Target position in approach frame
};
```

### Instrument_Transfer (MFD_TRANSFER)

Source: `Src/Orbiter/MfdTransfer.h`, `MfdTransfer.cpp`

```cpp
class Instrument_Transfer : public Instrument {
    Elements *shpel;            // Ship orbital elements
    Elements *shpel2;           // Ship elements in target frame
    Elements *tgtel;            // Target orbital elements
    const CelestialBody *elref; // Reference body
    double l_eject;             // Longitude of ejection point
    double deltav;              // Ejection delta-v
    double hto_a;               // Transfer orbit semi-major axis
    Vector *path;               // Numerical trajectory points
    oapi::IVECTOR2 *pathp;      // Screen-projected trajectory
};
```

### Instrument_Map (MFD_MAP)

Source: `Src/Orbiter/MfdMap.h`, `MfdMap.cpp`

```cpp
class Instrument_Map : public Instrument {
    VectorMap *map;             // Map rendering object
    const Planet *refplanet;    // Reference planet
    int zoom;                   // Zoom level
    bool track;                 // Auto-track vessel?
    DWORD dispflag;             // Display flags (grid, labels, etc.)
};
```

### Instrument_Landing (MFD_LANDING)

Source: search for `MfdLanding` or similar in `Src/Orbiter/`

Stores altitude, vertical speed, descent rate, and landing site information
relative to the target base.

### Instrument_OPlaneAlign (MFD_OPLANEALIGN)

Stores relative inclination, LAN difference, and time-to-node for orbital
plane alignment maneuvers.

### Instrument_OSync (MFD_OSYNC)

Stores synchronization orbit data: phase angles, timing for orbit matching,
and transfer window parameters.

### Instrument_HSI (MFD_HSI)

Horizontal Situation Indicator — stores NAV radio tuning, course deviation,
glideslope, bearing, and distance to selected navaid.

## Access Pattern

### Reading MFD Data

```cpp
// Declare the globals (resolved at load time — same process)
extern Pane *g_pane;

// Get the left MFD instance
Instrument *instr = g_pane->MFD(MFD_LEFT);
if (!instr) return;  // MFD not active

// Check what mode it's in
int mode = instr->Type();

// Cast to the specific derived class to read data
switch (mode) {
case MFD_ORBIT: {
    auto *mfd = static_cast<Instrument_Orbit*>(instr);
    // mfd->shpel->a   — semi-major axis
    // mfd->shpel->e   — eccentricity
    // mfd->elref       — reference body
    // mfd->tgt         — target (may be null)
    break;
}
case MFD_SURFACE: {
    auto *mfd = static_cast<Instrument_Surface*>(instr);
    // mfd->alt         — altitude
    // mfd->spd         — airspeed
    // mfd->pitch       — pitch angle
    // mfd->atm_M       — Mach number
    break;
}
// ... other modes
}
```

### The VESSEL → Vessel Bridge

The SDK's `VESSEL` class wraps the internal `Vessel`:

```cpp
// VesselAPI.h:5395
class VESSEL {
protected:
    Vessel *vessel;     // Internal vessel object
    short flightmodel;
    short version;
};
```

`oapiGetFocusInterface()` returns the `VESSEL` wrapper. The internal `Vessel*`
is the first protected member. The internal `Vessel` class provides access to
thrusters, control surfaces, animations, docking ports, nav radios, mesh lists,
and MFD mode lists — all beyond what the SDK exposes.

## Practical Considerations

### What This Enables for Accessibility

1. **Read what the user's MFDs are showing** — report the same orbital elements,
   surface data, or docking info that appears on the graphical display
2. **Track MFD mode changes** — know when the user switches modes
3. **Mirror addon MFD data** — custom MFD modes from third-party addons store
   data in their own member variables; if their class layout is known, the same
   approach works
4. **Access panel area registries** — enumerate what interactive areas exist on
   the current panel, even though semantic meaning isn't discoverable

### Risks and Mitigations

**ABI coupling**: This approach depends on the exact memory layout of Orbiter's
internal classes. If a future Orbiter version changes member ordering, offsets
break. Mitigations:
- The Orbiter source is available — verify layout against the target version
- Orbiter 2024 appears to be a stable release; layout changes are infrequent
- Runtime sanity checks (validate known fields) can detect layout mismatches

**Access specifiers**: Many members are `protected` or `private`. In C++ this is
a compile-time constraint, not a runtime one. Options:
- `#define private public` before including internal headers (crude but works)
- Reproduce the class layout as a struct with public members
- Use offset-based access with `reinterpret_cast`

**Thread safety**: MFD `UpdateDraw()` runs on the render thread. Reading member
variables from the console thread could race. Orbiter's simulation loop is
largely single-threaded, but care is needed if the console input thread reads
MFD data asynchronously.

### Build Integration

To use internal headers, add the Orbiter source include path:

```cmake
target_include_directories(AccessibleMFD PRIVATE
    "${ORBITER_SOURCE_DIR}/Src/Orbiter"
)
```

Or selectively copy the needed headers (`Pane.h`, `Mfd.h`, `MfdOrbit.h`, etc.)
into the project and maintain them as version-locked references.

## Source File Reference

| File | Contents |
|------|----------|
| `Src/Orbiter/Orbiter.cpp:92-139` | Global pointer definitions |
| `Src/Orbiter/Pane.h` | Pane class — MFD slot array, panel/VC/HUD management |
| `Src/Orbiter/Pane.cpp:1094-1104` | `Pane::MFD()` accessor implementation |
| `Src/Orbiter/Mfd.h` | Instrument base class, MFDMODE registry |
| `Src/Orbiter/MfdOrbit.h/.cpp` | Orbit MFD — orbital elements, target, display modes |
| `Src/Orbiter/MfdSurface.h/.cpp` | Surface MFD — flight parameters, atmosphere |
| `Src/Orbiter/MfdDocking.h/.cpp` | Docking MFD — target distance, approach frame |
| `Src/Orbiter/MfdTransfer.h/.cpp` | Transfer MFD — delta-v, ejection, trajectory |
| `Src/Orbiter/MfdMap.h/.cpp` | Map MFD — planet, zoom, display flags |
| `Src/Orbiter/Panel.h` | Legacy 2D panel — area registry |
| `Src/Orbiter/Panel2D.h` | Modern 2D panel — area registry, MFD geometry |
| `Src/Orbiter/VCockpit.h` | Virtual cockpit — 3D click areas |
| `Src/Orbiter/Vessel.h` | Internal Vessel class (full simulation state) |
| `Orbitersdk/include/VesselAPI.h:5395` | VESSEL wrapper with protected `Vessel*` member |
