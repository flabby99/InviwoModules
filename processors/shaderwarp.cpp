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
    Tags::GL          // Tags
};

const ProcessorInfo ShaderWarp::getProcessorInfo() const { return processorInfo_; }

ShaderWarp::ShaderWarp()
    : Processor()
    , entryPort_("disparity")
    , outport_("outport")
    , disparityScale_x_("disparityScale_x", "Disparity Scale x", 0.0, -10, 10, 0.01)
    , disparityScale_y_("disparityScale_y", "Disparity Scale y", 0.0, -10, 10, 0.01)
    , regionSizeProperty_("size", "Size", 5.0f, 0.0f, 10.0f)
    , verticalAngleProperty_("vertical_angle", "Vertical Angle", 0.0f, -60.0f, 60.0f, 0.1f)
    , viewConeProperty_("view_cone", "View cone", 40.0f, 0.0f, 90.0f, 0.1f)
    , shift_("shift", "Shift between cameras", 0.0f, -100.0f, 100.0f, 0.01f)
    , camera_("camera", "Camera")
    , shader_("backwardwarping.frag") {
    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addPort(entryPort_, "ImagePortGroup1");
    addPort(outport_, "ImagePortGroup1");

    addProperty(camera_);
    addProperty(disparityScale_x_);
    addProperty(disparityScale_y_);
    addProperty(regionSizeProperty_);
    addProperty(viewConeProperty_);
    addProperty(verticalAngleProperty_);
    addProperty(shift_);
    disparityScale_x_.setReadOnly(true);
    disparityScale_y_.setReadOnly(true);
    shift_.setReadOnly(true);

    disparity_size_ = size2_t(819, 455);

    outport_.setDimensions(size2_t(4096, 4096));
}

void ShaderWarp::initializeResources() {
    // Add any defines here.

    shader_.build();
}

float ShaderWarp::getSensorSizeY() {
    float focal_length = camera_.projectionMatrix()[0][0];
    float fov_degrees = ((PerspectiveCamera*) (&camera_.get()))->getFovy();
    float fov_radians = fov_degrees * PI_VALUE / 180.0f;
    float sensor_size = 2.0f * focal_length * tan(fov_radians / 2.0f);
    return sensor_size;
}

float ShaderWarp::getSensorSizeX() {
    float focal_length = camera_.projectionMatrix()[0][0];
    float fov_degrees = ((PerspectiveCamera*) (&camera_.get()))->getFovy();
    float aspect_ratio = ((PerspectiveCamera*) (&camera_.get()))->getAspectRatio();
    float fov_radians = fov_degrees * PI_VALUE / 180.0f;
    fov_radians = 2 * atan((fov_radians / 2.0f) * aspect_ratio);
    float sensor_size = 2.0f * focal_length * tan(fov_radians);
    return sensor_size;
}

void ShaderWarp::process() {
    if (entryPort_.isReady()){    
        // Do the backward warping
        TextureUnitContainer units;
        utilgl::activateAndClearTarget(outport_);
        shader_.activate();

        utilgl::bindAndSetUniforms(
                shader_, units, entryPort_, ImageType::ColorDepth);
        utilgl::setUniforms(shader_, outport_);

        drawLGViews();

        shader_.deactivate();
        utilgl::deactivateCurrentTarget();
    }
}

void ShaderWarp::drawLGViews() {
    float sensor_size_x = getSensorSizeX();
    float sensor_size_y = getSensorSizeY();

    // Draw the views
    int view = 0;
    size2_t tileSize = disparity_size_;
    float viewCone = viewConeProperty_.get();
    PerspectiveCamera* cam = (PerspectiveCamera*)&camera_.get();
    float size = regionSizeProperty_.get();
    float verticalAngle = verticalAngleProperty_.get();
    float adjustedSize = size / tanf(glm::radians(cam->getFovy() * 0.5f));
    float offsetX = 0;
    float offsetY = 0;
    for(int y = 0; y < 9; ++y)
    {
        for(int x = 0; x < 5; ++x)
        { 
        float angleAtView = -viewCone * 0.5f + (float)view / (45.0f - 1.0f) * viewCone;
        offsetX = adjustedSize * tanf(glm::radians(angleAtView));
        offsetY = adjustedSize * tanf(glm::radians(verticalAngle));

        // TODO this may not be needed (the division)
        disparityScale_x_ = (offsetX / size * cam->getAspectRatio());
        disparityScale_y_ = (offsetY / size);
        shift_ = offsetX / (size * cam->getAspectRatio());
        
        utilgl::setUniforms(shader_, disparityScale_x_, disparityScale_y_, shift_);

        size2_t start(x * tileSize.x, y * tileSize.y);
        glViewport(start.x, start.y, tileSize.x, tileSize.y);

        inviwo::vec4 viewport = vec4(start.x, start.y, tileSize.x, tileSize.y);
        shader_.setUniform("viewport", viewport);
        
        utilgl::singleDrawImagePlaneRect();
        ++view;
        }
    }
    glViewport(0, 0, 4096, 4096);
}

void ShaderWarp::deserialize(Deserializer& d) {
    util::renamePort(d, {{&entryPort_, "entry-points"}, });
    Processor::deserialize(d);
}

}
