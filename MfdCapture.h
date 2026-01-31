// MfdCapture.h - In-process MFD text capture via Sketchpad::Text hook

#ifndef MFD_CAPTURE_H
#define MFD_CAPTURE_H

#include <windows.h>

// A single text fragment captured from Sketchpad::Text
struct MfdTextEntry {
    int x;
    int y;
    char text[128];
};

// All captured text for one MFD slot
struct MfdSlotData {
    int mfdIndex;       // Orbiter MFD index (0-11)
    int modeId;         // oapiGetMFDMode() result
    char title[128];    // DisplayTitle text (drawn at y==0)
    int entryCount;
    MfdTextEntry entries[64];
};

// Snapshot of MFD text data
struct MfdFrameData {
    int slotCount;
    MfdSlotData slots[12];
    DWORD frameNumber;

    // Diagnostic counters
    int activeMfdCount;     // Active MFD slots (from oapiGetMFDMode)
    int trackedSurfCount;   // Total tracked surfaces (MFD and non-MFD)
    int mfdSurfCount;       // Surfaces identified as MFDs (received y==0 text)
};

// Install Sketchpad::Text hook via MinHook (call from InitModule)
bool MfdCaptureInstall();

// Remove hook (call from ExitModule)
void MfdCaptureRemove();

// Query active MFDs and snapshot MFD surface data into display buffer.
// Call from clbkPreStep (main thread).
void MfdCaptureFrameStart();

// Thread-safe copy of last completed snapshot (call from console thread)
void MfdCaptureGetFrame(MfdFrameData* out);

// Milliseconds since last snapshot update
DWORD MfdCaptureGetFrameAge();

// Print diagnostic info about capture state
void MfdCaptureDiag();

#endif // MFD_CAPTURE_H
