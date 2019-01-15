/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <math.h>

#include <modules/image_warping/processors/shaderwarp.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>

#define PI_VALUE 3.1415927

namespace inviwo {

const ProcessorInfo ShaderWarp::processorInfo_{
    "org.inviwo.ShaderWarp",  // Class identifier
    "Shader Based Backward Warp",            // Display name
    "Image Processing",            // Category
    CodeState::Experimental,       // Code state
    "Warp", "Image"          // Tags
};

const ProcessorInfo ShaderWarp::getProcessorInfo() const { return processorInfo_; }

ShaderWarp::ShaderWarp()
    : Processor()
    , entryPort_("entry")
    , outport_("outport")
    , disparity_()
    , cameraBaseline_("cameraBaseline", "Camera Baseline", 0.05, 0, 1, 0.001)
    , disparityScale_x_("disparityScale_x", "Disparity Scale x", 0.0, -10, 10, 0.01)
    , disparityScale_y_("disparityScale_y", "Disparity Scale y", 0.0, -10, 10, 0.01)
    , camera_("camera", "Camera")
    , shader_("backwardwarping.frag")
    , depthShader_("depth_to_disparity.frag") {

    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addPort(entryPort_, "ImagePortGroup1");
    addPort(outport_, "ImagePortGroup1");

    addProperty(cameraBaseline_);
    addProperty(camera_);
    addProperty(disparityScale_x_);
    addProperty(disparityScale_y_);
    disparityScale_x_.setReadOnly(true);
    disparityScale_y_.setReadOnly(true);

    disparity_ = Image(outport_.getDimensions(), outport_.getDataFormat());
}

void ShaderWarp::initializeResources() {
    // Add any defines here.

    depthShader_.build();

    shader_.build();
}

float ShaderWarp::getSensorSize() {
    float focal_length = camera_.projectionMatrix()[0][0];
    float fov_degrees = ((PerspectiveCamera*) (&camera_.get()))->getFovy();
    float fov_radians = (fov_degrees / 2) * PI_VALUE / 180.0f;
    float sensor_size = 2.0f * focal_length * tan(fov_radians);
    return sensor_size;
}

// TODO loop this
void ShaderWarp::process() {
    if (entryPort_.isReady()){    
        //TODO only do this if the dimensions have changed
        disparity_ = Image(outport_.getDimensions(), outport_.getDataFormat());

        // Use shader to convert depth to disparity
        utilgl::activateAndClearTarget(disparity_);
        depthShader_.activate();
        
        TextureUnitContainer units;   
        utilgl::bindAndSetUniforms(
            depthShader_, units, entryPort_, ImageType::ColorDepth);
        utilgl::setUniforms(depthShader_, camera_, cameraBaseline_);
        utilgl::setShaderUniforms(depthShader_, disparity_, "disparityParameters");
        utilgl::singleDrawImagePlaneRect();
        
        depthShader_.deactivate();
        utilgl::deactivateCurrentTarget();

        // Set up distances
        float distance_x = -4.0f;
        float distance_y = -4.0f;
        float sensor_size = getSensorSize();
        disparityScale_x_ = (distance_x / sensor_size);
        disparityScale_y_ = (distance_y / sensor_size);

        // Do the backward warping
        utilgl::activateAndClearTarget(outport_);
        shader_.activate();
        
        utilgl::bindAndSetUniforms(
            shader_, units, disparity_, "disparity", ImageType::ColorDepth);
        utilgl::setUniforms(shader_, outport_, disparityScale_x_, disparityScale_y_);

        utilgl::singleDrawImagePlaneRect();
        
        shader_.deactivate();
        utilgl::deactivateCurrentTarget();
    }
}

void ShaderWarp::deserialize(Deserializer& d) {
    util::renamePort(d, {{&entryPort_, "entry-points"}, });
    Processor::deserialize(d);
}

}
