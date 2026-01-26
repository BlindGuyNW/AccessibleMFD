# Licensed under the MIT License

# =============================================================================
# AccessibleMFD - Console-based accessible flight data
# Standalone build configuration for use outside Orbiter source tree
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(AccessibleMFD LANGUAGES CXX)

# =============================================================================
# Orbiter SDK Configuration
# =============================================================================

# Allow setting via command line: cmake -DORBITER_DIR=C:/Orbiter
# Or environment variable, or edit the default here
if(DEFINED ENV{ORBITER_DIR} AND NOT ORBITER_DIR)
    set(ORBITER_DIR "$ENV{ORBITER_DIR}")
endif()

set(ORBITER_DIR "C:/orbit" CACHE PATH "Orbiter installation directory")

# Validate Orbiter directory
if(NOT EXISTS "${ORBITER_DIR}/Orbitersdk/include/OrbiterAPI.h")
    message(FATAL_ERROR
        "Orbiter SDK not found at: ${ORBITER_DIR}\n"
        "Set ORBITER_DIR to your Orbiter installation path:\n"
        "  cmake -DORBITER_DIR=C:/YourOrbiterPath .."
    )
endif()

# SDK paths
set(ORBITER_SDK_INCLUDE "${ORBITER_DIR}/Orbitersdk/include")
set(ORBITER_SDK_LIB "${ORBITER_DIR}/Orbitersdk/lib")
set(ORBITER_PLUGIN_DIR "${ORBITER_DIR}/Modules/Plugin")

message(STATUS "Orbiter SDK: ${ORBITER_DIR}")
message(STATUS "Output dir:  ${ORBITER_PLUGIN_DIR}")

# =============================================================================
# Platform checks (Orbiter requires 32-bit Windows)
# =============================================================================

if(NOT WIN32)
    message(FATAL_ERROR "Orbiter plugins require Windows")
endif()

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(WARNING
        "Building 64-bit but Orbiter requires 32-bit.\n"
        "Configure with: cmake -A Win32 .."
    )
endif()

# =============================================================================
# Import Orbiter SDK libraries
# =============================================================================

add_library(Orbiter STATIC IMPORTED)
set_target_properties(Orbiter PROPERTIES
    IMPORTED_LOCATION "${ORBITER_SDK_LIB}/Orbiter.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${ORBITER_SDK_INCLUDE}"
)

add_library(Orbitersdk STATIC IMPORTED)
set_target_properties(Orbitersdk PROPERTIES
    IMPORTED_LOCATION "${ORBITER_SDK_LIB}/Orbitersdk.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${ORBITER_SDK_INCLUDE}"
)

# =============================================================================
# AccessibleMFD Plugin
# =============================================================================

set(PRJ AccessibleMFD)

set(Sources
    AccessibleMFD.cpp
)

# Collect SDK headers for IDE convenience
file(GLOB APIHeaders "${ORBITER_SDK_INCLUDE}/*.h")
source_group("API Headers" FILES ${APIHeaders})
source_group("Source Files" FILES ${Sources})

add_library(${PRJ} SHARED
    ${Sources}
    ${APIHeaders}
)

target_link_libraries(${PRJ} PRIVATE
    Orbiter
    Orbitersdk
)

# MSVC settings
if(MSVC)
    target_compile_definitions(${PRJ} PRIVATE
        _CRT_SECURE_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
    )
    # Match Orbiter SDK runtime library (dynamic)
    set_property(TARGET ${PRJ} PROPERTY MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Output directly to Orbiter's plugin directory
set_target_properties(${PRJ} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${ORBITER_PLUGIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${ORBITER_PLUGIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${ORBITER_PLUGIN_DIR}"
    # Clean output name without config suffix
    OUTPUT_NAME "${PRJ}"
    DEBUG_POSTFIX ""
)
