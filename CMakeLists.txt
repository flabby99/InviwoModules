#--------------------------------------------------------------------
# Inviwo layereddepth Module
ivw_module(layereddepth)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/layereddepthprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/vertexarray.h
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/vertexbuffer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/clippingrenderprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/compositor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/fixedimagesize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/multiplaneraycaster.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/multipleplaneprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/vectorimageselector.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/vectorraycasting.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/layereddepthprocessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/vertexarray.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers/vertexbuffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/clippingrenderprocessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/compositor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/fixedimagesize.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/multiplaneraycaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/multipleplaneprocessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/vectorimageselector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/vectorraycasting.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})


#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/glsl/layereddepthprocessor.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/clippingrenderprocessor.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/clippingrenderprocessor.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/multipleplaneprocessor.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/multipleplaneprocessor.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/layeredraycasting.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/layeredcomposite.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/facerender.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/facerender.vert
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/loopraycasting.frag
    ${CMAKE_CURRENT_SOURCE_DIR}/glsl/firstpasslayeredraycasting.frag
)
ivw_group("Shader Files" ${SHADER_FILES})


#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
	#${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/layereddepth-test.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
# ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)