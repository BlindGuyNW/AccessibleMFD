# Licensed under the MIT License

# =============================================================================
# AccessibleMFD - Console-based accessible flight data
# =============================================================================

set(PRJ AccessibleMFD)

set(Sources
	AccessibleMFD.cpp
)

source_group(APIHeaders FILES ${APIHeaders})
source_group(Sources FILES ${Sources})

add_library(${PRJ} SHARED
	${Sources}
	${APIHeaders}
)

target_link_libraries(${PRJ} Orbiter Orbitersdk)

set_target_properties(${PRJ}
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PLUGIN_DIR}
	RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PLUGIN_DIR}
	FOLDER Accessibility
)
