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

#include <modules/image_warping/processors/depth_to_disparity.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>

#define PI_VALUE 3.1415927

namespace inviwo {

const ProcessorInfo DepthToDisparity::processorInfo_{
    "org.inviwo.DepthToDisparity",  // Class identifier
    "Depth To Disparity",            // Display name
    "Image Processing",            // Category
    CodeState::Experimental,       // Code state
    Tags::GL          // Tags
};

const ProcessorInfo DepthToDisparity::getProcessorInfo() const { return processorInfo_; }

DepthToDisparity::DepthToDisparity()
    : Processor()
    , entryPort_("entry")
    , disparity_("disparity")
    , cameraBaseline_("cameraBaseline", "Camera Baseline", 0.05, 0, 1, 0.001)
    , camera_("camera", "Camera")
    , depthShader_("depth_to_disparity.frag") {

    depthShader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addPort(entryPort_, "ImagePortGroup1");
    addPort(disparity_, "ImagePortGroup1");

    addProperty(cameraBaseline_);
    addProperty(camera_);

    disparity_size_ = size2_t(512, 512);

    disparity_.setDimensions(disparity_size_);
    disparity_.setHandleResizeEvents(false);
    (&entryPort_)->setOutportDeterminesSize(true);
}

void DepthToDisparity::initializeResources() {
    // Add any defines here.

    depthShader_.build();
}

void DepthToDisparity::process() {
    if (entryPort_.isReady()){
        auto start = std::chrono::system_clock::now();    
        // Use shader to convert depth to disparity
        utilgl::activateAndClearTarget(disparity_);
        depthShader_.activate();
        
        TextureUnitContainer units;   
        utilgl::bindAndSetUniforms(
            depthShader_, units, entryPort_, ImageType::ColorDepth);
        utilgl::setUniforms(depthShader_, camera_, cameraBaseline_);
        utilgl::setShaderUniforms(depthShader_, disparity_);
        
        utilgl::singleDrawImagePlaneRect();

        std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
        LogInfo("Disparity conversion took " << diff.count() << "s");
        depthShader_.deactivate();
        utilgl::deactivateCurrentTarget();
    }
}

void DepthToDisparity::deserialize(Deserializer& d) {
    util::renamePort(d, {{&entryPort_, "entry-points"}, });
    Processor::deserialize(d);
}

}
