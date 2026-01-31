// MfdCapture.cpp - In-process MFD text capture via Sketchpad::Text hook
//
// Hooks oapi::Sketchpad::Text() via vtable discovery. Each surface that
// receives text is tracked independently. Surfaces are identified as MFDs
// by the presence of y==0 text, which corresponds to Instrument::DisplayTitle()
// (always draws the MFD title at y=0). Non-MFD surfaces (glass cockpit
// instruments, etc.) never call DisplayTitle and have no y==0 text.
//
// Surface-to-MFD-slot mapping uses two strategies:
//  1. Same-frame ordering: When all active MFDs render in one Pane::Update()
//     pass, titleSeq order matches MFD index order. This establishes a
//     persistent SURFHANDLE→mfdIndex mapping.
//  2. Mode-change tracking: When oapiGetMFDMode() changes for a slot, the
//     old surface's mapping is invalidated. If exactly one slot and one
//     surface are unmatched, they're paired automatically.

#include "MfdCapture.h"
#include "orbitersdk.h"
#include "MinHook.h"
#include <string.h>
#include <stdio.h>

// --------------------------------------------------------------------------
// Sketchpad::Text vtable index (counted from DrawAPI.h virtual layout)
//   [0] ~Sketchpad  [1] SetFont  [2] SetPen  [3] SetBrush
//   [4] SetTextAlign [5] SetTextColor [6] SetBackgroundColor
//   [7] SetBackgroundMode [8] GetCharSize [9] GetTextWidth
//   [10] SetOrigin [11] GetOrigin [12] Text
// --------------------------------------------------------------------------
static const int SKP_TEXT_VTABLE_INDEX = 12;

// --------------------------------------------------------------------------
// Original function pointer (__thiscall: this in ECX, params on stack)
// --------------------------------------------------------------------------
typedef bool (__thiscall *SkpText_t)(oapi::Sketchpad*, int, int, const char*, int);
static SkpText_t g_origSkpText = nullptr;
static void*     g_hookedAddr = nullptr;

// --------------------------------------------------------------------------
// Per-surface accumulator (persistent across frames, main thread only)
// --------------------------------------------------------------------------
struct SurfCapture {
    SURFHANDLE surf;           // 0 = unused slot
    bool       isMfd;          // has received y==0 text (DisplayTitle)
    bool       hasTitlePos;    // whether titleX has been established
    int        titleX;         // x-coordinate of the frame-start text
    char       title[128];     // title from most recent frame-start text
    int        entryCount;
    MfdTextEntry entries[64];
    DWORD      titleSeq;       // monotonic counter for render-order sorting
    DWORD      lastTitleTick;  // GetTickCount() when last frame-start seen
    int        assignedMfd;    // MFD index (0-11) or -1 if unassigned
};

static const int MAX_SURFS = 16;
static SurfCapture g_surfs[MAX_SURFS];
static int         g_surfCount = 0;
static DWORD       g_titleSeqCounter = 0;

// Active MFD slots (from oapiGetMFDMode, updated in MfdCaptureFrameStart)
static int  g_activeMfdCount = 0;
static int  g_activeMfdIndices[12];
static int  g_activeMfdModes[12];

// Previous frame's MFD modes for change detection
static int  g_prevMfdModes[12];   // oapiGetMFDMode(i) for i=0..11
static bool g_prevModesValid = false;

// Frame sequencing for same-frame detection
static DWORD g_lastFrameStartSeq = 0;

// --------------------------------------------------------------------------
// Display buffer (read by console thread under lock)
// --------------------------------------------------------------------------
static CRITICAL_SECTION g_displayLock;
static MfdFrameData     g_displayFrame;
static DWORD            g_displayTimestamp = 0;
static DWORD            g_frameNumber = 0;

// --------------------------------------------------------------------------
// Hook state
// --------------------------------------------------------------------------
static bool g_mhInitialized = false;
static bool g_hookInstalled = false;

// --------------------------------------------------------------------------
// Hooked Sketchpad::Text
// Uses __fastcall trick: ECX = this, EDX = unused, then stack params.
// Runs on main thread during rendering -- no lock needed for g_surfs.
// --------------------------------------------------------------------------
static bool __fastcall Hooked_SkpText(oapi::Sketchpad* self, void* /*edx*/,
                                       int x, int y, const char* str, int len) {
    bool result = g_origSkpText(self, x, y, str, len);

    if (len <= 0 || !str)
        return result;

    SURFHANDLE surf = self->GetSurface();
    if (!surf) return result;

    // Find or create surface slot
    int slot = -1;
    for (int i = 0; i < g_surfCount; i++) {
        if (g_surfs[i].surf == surf) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        if (g_surfCount >= MAX_SURFS)
            return result;
        slot = g_surfCount++;
        memset(&g_surfs[slot], 0, sizeof(SurfCapture));
        g_surfs[slot].surf = surf;
        g_surfs[slot].assignedMfd = -1;
    }

    SurfCapture& cap = g_surfs[slot];

    // Detect MFD surfaces and frame boundaries via y==0 text.
    // Instrument::DisplayTitle() always draws at y=0. Custom MFDs may
    // draw multiple texts at y==0 (e.g., "Ascent P1/4", "ACT", "MET:...").
    // To avoid clearing mid-frame, we only treat the FIRST y==0 position
    // (by x-coordinate) as the frame-start signal, matching how the Frida
    // mfd_reader detects new frames by title position.
    if (y == 0) {
        if (!cap.hasTitlePos) {
            // First y==0 text on this surface: establish title position
            cap.titleX = x;
            cap.hasTitlePos = true;
            cap.isMfd = true;
            cap.entryCount = 0;
            int n = (len < 127) ? len : 127;
            memcpy(cap.title, str, n);
            cap.title[n] = '\0';
            cap.titleSeq = ++g_titleSeqCounter;
            cap.lastTitleTick = GetTickCount();
        }
        else if (x == cap.titleX) {
            // Title position seen again → new render frame for this surface.
            // Check for title text change (mode change / surface reuse).
            int n = (len < 127) ? len : 127;
            if (strncmp(cap.title, str, n) != 0 || cap.title[n] != '\0') {
                cap.assignedMfd = -1;
            }
            cap.entryCount = 0;
            memcpy(cap.title, str, n);
            cap.title[n] = '\0';
            cap.titleSeq = ++g_titleSeqCounter;
            cap.lastTitleTick = GetTickCount();
        }
        // else: y==0 at different x → additional title-line text, just append
    }

    // Append entry
    if (cap.entryCount >= 64)
        return result;

    MfdTextEntry& e = cap.entries[cap.entryCount++];
    e.x = x;
    e.y = y;
    int n = (len < 127) ? len : 127;
    memcpy(e.text, str, n);
    e.text[n] = '\0';

    return result;
}

// --------------------------------------------------------------------------
// Discover Sketchpad::Text address via vtable and hook it
// Called lazily on first simulation frame (graphics client is ready)
// --------------------------------------------------------------------------
static bool InstallSkpTextHook() {
    SURFHANDLE tmpSurf = oapiCreateSurface(256, 256);
    if (!tmpSurf) return false;

    oapi::Sketchpad* skp = oapiGetSketchpad(tmpSurf);
    if (!skp) {
        oapiDestroySurface(tmpSurf);
        return false;
    }

    void** vtable = *(void***)skp;
    g_hookedAddr = vtable[SKP_TEXT_VTABLE_INDEX];

    oapiReleaseSketchpad(skp);
    oapiDestroySurface(tmpSurf);

    if (!g_hookedAddr) return false;

    if (MH_CreateHook(g_hookedAddr, (LPVOID)Hooked_SkpText,
                       (LPVOID*)&g_origSkpText) != MH_OK)
        return false;

    if (MH_EnableHook(g_hookedAddr) != MH_OK) {
        MH_RemoveHook(g_hookedAddr);
        return false;
    }

    return true;
}

// --------------------------------------------------------------------------
// MfdCaptureInstall  (call from InitModule)
// --------------------------------------------------------------------------
bool MfdCaptureInstall() {
    if (g_mhInitialized)
        return true;

    InitializeCriticalSection(&g_displayLock);
    memset(&g_displayFrame, 0, sizeof(g_displayFrame));
    memset(g_surfs, 0, sizeof(g_surfs));
    for (int i = 0; i < MAX_SURFS; i++)
        g_surfs[i].assignedMfd = -1;
    g_surfCount = 0;
    g_activeMfdCount = 0;
    g_titleSeqCounter = 0;
    g_lastFrameStartSeq = 0;
    g_prevModesValid = false;
    memset(g_prevMfdModes, 0, sizeof(g_prevMfdModes));

    if (MH_Initialize() != MH_OK)
        return false;

    g_mhInitialized = true;
    return true;
}

// --------------------------------------------------------------------------
// MfdCaptureRemove  (call from ExitModule)
// --------------------------------------------------------------------------
void MfdCaptureRemove() {
    if (g_hookInstalled && g_hookedAddr) {
        MH_DisableHook(g_hookedAddr);
        MH_RemoveHook(g_hookedAddr);
    }
    if (g_mhInitialized) {
        MH_Uninitialize();
        DeleteCriticalSection(&g_displayLock);
        g_mhInitialized = false;
    }
    g_hookInstalled = false;
    g_hookedAddr = nullptr;
    g_origSkpText = nullptr;
}

// --------------------------------------------------------------------------
// Helper: insertion sort int array by a key function
// --------------------------------------------------------------------------
static void SortSlotsBySeq(int* arr, int count) {
    for (int i = 1; i < count; i++) {
        int tmp = arr[i];
        DWORD tmpSeq = g_surfs[tmp].titleSeq;
        int j = i - 1;
        while (j >= 0 && g_surfs[arr[j]].titleSeq > tmpSeq) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = tmp;
    }
}

// --------------------------------------------------------------------------
// MfdCaptureFrameStart  (called from clbkPreStep on main thread)
//
// Does NOT clear surface data. Each surface's entries persist until that
// surface renders again (detected by y==0 in the hook). This function
// queries active MFDs, maintains the surface→MFD mapping, and snapshots
// data for the console thread.
// --------------------------------------------------------------------------
void MfdCaptureFrameStart() {
    if (!g_mhInitialized)
        return;

    if (!g_hookInstalled) {
        g_hookInstalled = InstallSkpTextHook();
        if (!g_hookInstalled)
            return;
    }

    // --- Query which MFD slots are active ---
    g_activeMfdCount = 0;
    for (int i = 0; i < 12; i++) {
        int mode = oapiGetMFDMode(i);
        if (mode != MFD_NONE) {
            g_activeMfdIndices[g_activeMfdCount] = i;
            g_activeMfdModes[g_activeMfdCount] = mode;
            g_activeMfdCount++;
        }
    }

    // --- Detect mode changes and invalidate affected mappings ---
    if (g_prevModesValid) {
        for (int i = 0; i < 12; i++) {
            if (oapiGetMFDMode(i) != g_prevMfdModes[i]) {
                // MFD i changed mode -- unassign its surface
                for (int j = 0; j < g_surfCount; j++) {
                    if (g_surfs[j].assignedMfd == i) {
                        g_surfs[j].assignedMfd = -1;
                    }
                }
            }
        }
    }
    for (int i = 0; i < 12; i++)
        g_prevMfdModes[i] = oapiGetMFDMode(i);
    g_prevModesValid = true;

    // --- Collect non-stale MFD surfaces ---
    DWORD now = GetTickCount();
    const DWORD STALE_MS = 5000;

    int mfdSlots[MAX_SURFS];
    int mfdCount = 0;

    for (int i = 0; i < g_surfCount; i++) {
        if (g_surfs[i].isMfd) {
            if ((now - g_surfs[i].lastTitleTick) < STALE_MS) {
                mfdSlots[mfdCount++] = i;
            } else {
                // Stale -- clear assignment
                g_surfs[i].assignedMfd = -1;
            }
        }
    }

    // --- Strategy 1: Same-frame ordering ---
    // If all active MFDs rendered since last MfdCaptureFrameStart,
    // titleSeq order matches MFD index order (Pane::Update iterates
    // in index order). Establish definitive mapping.
    DWORD prevSeq = g_lastFrameStartSeq;
    g_lastFrameStartSeq = g_titleSeqCounter;

    int thisFrameSlots[MAX_SURFS];
    int thisFrameCount = 0;
    for (int i = 0; i < mfdCount; i++) {
        if (g_surfs[mfdSlots[i]].titleSeq > prevSeq) {
            thisFrameSlots[thisFrameCount++] = mfdSlots[i];
        }
    }

    if (thisFrameCount >= g_activeMfdCount && g_activeMfdCount > 0) {
        // All active MFDs rendered in this frame -- assign by titleSeq order
        SortSlotsBySeq(thisFrameSlots, thisFrameCount);
        for (int i = 0; i < g_activeMfdCount && i < thisFrameCount; i++) {
            g_surfs[thisFrameSlots[i]].assignedMfd = g_activeMfdIndices[i];
        }
    }

    // --- Strategy 2: Single-change matching ---
    // Find active MFDs with no assigned surface and unassigned surfaces.
    // If exactly one of each, pair them.
    {
        int unassignedMfds[12];
        int unassignedMfdCount = 0;
        for (int i = 0; i < g_activeMfdCount; i++) {
            int mfdIdx = g_activeMfdIndices[i];
            bool found = false;
            for (int j = 0; j < mfdCount; j++) {
                if (g_surfs[mfdSlots[j]].assignedMfd == mfdIdx) {
                    found = true;
                    break;
                }
            }
            if (!found)
                unassignedMfds[unassignedMfdCount++] = mfdIdx;
        }

        int unassignedSurfs[MAX_SURFS];
        int unassignedSurfCount = 0;
        for (int i = 0; i < mfdCount; i++) {
            if (g_surfs[mfdSlots[i]].assignedMfd == -1)
                unassignedSurfs[unassignedSurfCount++] = mfdSlots[i];
        }

        if (unassignedMfdCount == 1 && unassignedSurfCount == 1) {
            // Exactly one orphan on each side -- pair them
            g_surfs[unassignedSurfs[0]].assignedMfd = unassignedMfds[0];
        }
        else if (unassignedMfdCount > 1 &&
                 unassignedMfdCount == unassignedSurfCount) {
            // Multiple unassigned -- fall back to titleSeq ordering
            // among the unassigned surfaces. Sort unassigned MFD indices
            // ascending, and unassigned surfaces by titleSeq, then pair.
            SortSlotsBySeq(unassignedSurfs, unassignedSurfCount);
            // Sort MFD indices ascending
            for (int i = 1; i < unassignedMfdCount; i++) {
                int tmp = unassignedMfds[i];
                int j = i - 1;
                while (j >= 0 && unassignedMfds[j] > tmp) {
                    unassignedMfds[j + 1] = unassignedMfds[j];
                    j--;
                }
                unassignedMfds[j + 1] = tmp;
            }
            for (int i = 0; i < unassignedMfdCount; i++)
                g_surfs[unassignedSurfs[i]].assignedMfd = unassignedMfds[i];
        }
    }

    // --- Build display frame using assignments ---
    MfdFrameData newFrame;
    memset(&newFrame, 0, sizeof(newFrame));
    newFrame.frameNumber = ++g_frameNumber;
    newFrame.activeMfdCount = g_activeMfdCount;
    newFrame.trackedSurfCount = g_surfCount;
    newFrame.mfdSurfCount = mfdCount;

    int slotIdx = 0;
    for (int i = 0; i < g_activeMfdCount; i++) {
        int mfdIdx = g_activeMfdIndices[i];
        // Find the surface assigned to this MFD
        SurfCapture* cap = nullptr;
        for (int j = 0; j < mfdCount; j++) {
            if (g_surfs[mfdSlots[j]].assignedMfd == mfdIdx) {
                cap = &g_surfs[mfdSlots[j]];
                break;
            }
        }
        if (!cap) continue;

        MfdSlotData& slot = newFrame.slots[slotIdx];
        slot.mfdIndex = mfdIdx;
        slot.modeId = g_activeMfdModes[i];
        strncpy(slot.title, cap->title, 127);
        slot.title[127] = '\0';
        slot.entryCount = cap->entryCount;
        if (slot.entryCount > 64) slot.entryCount = 64;
        memcpy(slot.entries, cap->entries,
               slot.entryCount * sizeof(MfdTextEntry));
        slotIdx++;
    }
    newFrame.slotCount = slotIdx;

    if (slotIdx > 0) {
        EnterCriticalSection(&g_displayLock);
        memcpy(&g_displayFrame, &newFrame, sizeof(MfdFrameData));
        g_displayTimestamp = now;
        LeaveCriticalSection(&g_displayLock);
    }
}

// --------------------------------------------------------------------------
// MfdCaptureGetFrame  (called from console thread)
// --------------------------------------------------------------------------
void MfdCaptureGetFrame(MfdFrameData* out) {
    if (!g_hookInstalled || !out) {
        if (out) memset(out, 0, sizeof(MfdFrameData));
        return;
    }

    EnterCriticalSection(&g_displayLock);
    memcpy(out, &g_displayFrame, sizeof(MfdFrameData));
    LeaveCriticalSection(&g_displayLock);
}

// --------------------------------------------------------------------------
// MfdCaptureGetFrameAge
// --------------------------------------------------------------------------
DWORD MfdCaptureGetFrameAge() {
    if (!g_hookInstalled || g_displayTimestamp == 0)
        return MAXDWORD;

    return GetTickCount() - g_displayTimestamp;
}

// --------------------------------------------------------------------------
// MfdCaptureDiag  (called from console thread)
// Note: reads g_surfs without lock -- diagnostic only, may show torn data.
// --------------------------------------------------------------------------
void MfdCaptureDiag() {
    printf("Hook installed: %s\n", g_hookInstalled ? "yes" : "no");
    printf("Hooked addr: %p\n", g_hookedAddr);

    if (!g_hookInstalled) return;

    MfdFrameData frame;
    EnterCriticalSection(&g_displayLock);
    memcpy(&frame, &g_displayFrame, sizeof(MfdFrameData));
    LeaveCriticalSection(&g_displayLock);

    DWORD age = MfdCaptureGetFrameAge();
    printf("Frame #%lu, age %lu ms\n", frame.frameNumber, age);
    printf("Active MFDs: %d\n", frame.activeMfdCount);
    printf("Tracked surfaces: %d\n", frame.trackedSurfCount);
    printf("MFD surfaces (y==0): %d\n", frame.mfdSurfCount);
    printf("Mapped slots: %d\n\n", frame.slotCount);

    // Raw surface state (diagnostic, unlocked read)
    DWORD now = GetTickCount();
    printf("--- All tracked surfaces ---\n");
    for (int i = 0; i < g_surfCount && i < MAX_SURFS; i++) {
        SurfCapture& s = g_surfs[i];
        DWORD surfAge = s.lastTitleTick ? (now - s.lastTitleTick) : 0;
        printf("  [%d] surf=%p  mfd=%s  assigned=%d  entries=%d  seq=%lu  age=%lums",
               i, (void*)s.surf, s.isMfd ? "yes" : "no",
               s.assignedMfd, s.entryCount, s.titleSeq, surfAge);
        if (s.isMfd)
            printf("  \"%s\"", s.title);
        printf("\n");
    }

    printf("\n--- Mapped MFD slots ---\n");
    for (int i = 0; i < frame.slotCount; i++) {
        MfdSlotData& s = frame.slots[i];
        printf("  MFD %d (mode %d) \"%s\": %d entries\n",
               s.mfdIndex, s.modeId, s.title, s.entryCount);
        int show = s.entryCount < 5 ? s.entryCount : 5;
        for (int j = 0; j < show; j++) {
            printf("    [%d,%d] \"%s\"\n",
                   s.entries[j].x, s.entries[j].y, s.entries[j].text);
        }
        if (s.entryCount > 5) printf("    ... +%d more\n", s.entryCount - 5);
    }
}
