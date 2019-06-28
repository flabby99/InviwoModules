#--------------------------------------------------------------------
# Inviwo looking-glass Module
ivw_module(lookingglass)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/lfentryexitpoints.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/lfswizzleprocessor.hpp
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/lfentryexitpoints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/lfswizzleprocessor.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})


#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfentryexitpoints.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfentryexitpoints.vert

    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfswizzle.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfswizzle.vert

    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfcapnearclipping.frag
)
ivw_group("Shader Files" ${SHADER_FILES})


#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
	#${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/lookingglass-test.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)