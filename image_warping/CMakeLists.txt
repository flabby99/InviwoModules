#--------------------------------------------------------------------
# Inviwo image_warping Module
ivw_module(image_warping)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/image_warpingprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/shaderwarp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/depth_to_disparity.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/lfentryexitpoints.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/image_warpingprocessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/shaderwarp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/depth_to_disparity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/lfentryexitpoints.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})


#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/glsl/image_warpingprocessor.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/depth_to_disparity.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/backwardwarping.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfcapnearclipping_w.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfentryexitpoints_w.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/lfentryexitpoints_w.vert
)
ivw_group("Shader Files" ${SHADER_FILES})


#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
	#${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/image_warping-test.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)