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

#include <modules/layereddepth/processors/clippingrenderprocessor.h>

#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/rendering/meshdrawergl.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo clippingRenderProcessor::processorInfo_{
    "org.inviwo.clippingRenderProcessor",      // Class identifier
    "Clipping Render Processor",                // Display name
    "Rendering",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo clippingRenderProcessor::getProcessorInfo() const { return processorInfo_; }

clippingRenderProcessor::clippingRenderProcessor()
    : Processor()
    , inport_("inport")
    , entryPort_("entry", DataVec4UInt16::get())
    , exitPort_("exit", DataVec4UInt16::get())
    , camera_("camera", "Camera", vec3(0.0f, 0.0f, -2.0f), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), &inport_)
    , trackball_(&camera_)
    , planeNormal_("normal", "Plane normal", vec3(0.0f), vec3(-100.0f), vec3(100.0f))
    , planeDistance_("distance", "Plane distance along normal", 0.0f, -10.0f, 10.0f)
    , planeReverseDistance_("reverse_distance", "Reverse plane distance along normal", 0.0f, -10.0f, 10.0f)
    , useCameraNormalAsPlane_("camNormal", "Use camera normal as plane normal", true)
    , shader_("clippingrenderprocessor.vert", "clippingrenderprocessor.frag")
    {
    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addPort(inport_);
    addPort(entryPort_, "ImagePortGroup1");
    addPort(exitPort_, "ImagePortGroup1");
    addProperty(planeDistance_);
    addProperty(planeReverseDistance_);
    addProperty(planeNormal_);
    addProperty(useCameraNormalAsPlane_);
    addProperty(camera_);
    addProperty(trackball_);

    planeNormal_.setReadOnly(useCameraNormalAsPlane_.get());
    entryPort_.addResizeEventListener(&camera_);
    useCameraNormalAsPlane_.onChange(
        [this]() { onAlignPlaneNormalToCameraNormalToggled(); });
}

void clippingRenderProcessor::onAlignPlaneNormalToCameraNormalToggled() {
    planeNormal_.setReadOnly(useCameraNormalAsPlane_.get());
    if (useCameraNormalAsPlane_.get()) {           
        planeNormal_.set(glm::normalize(camera_.getLookTo() - camera_.getLookFrom()));
    }
}

void clippingRenderProcessor::initializeResources() {
    shader_.build();
}

void clippingRenderProcessor::process() {
    if (useCameraNormalAsPlane_.get()) {           
        planeNormal_.set(glm::normalize(camera_.getLookTo() - camera_.getLookFrom()));
    }
    utilgl::DepthFuncState depthfunc(GL_ALWAYS);
    utilgl::PointSizeState pointsize(1.0f);

    mat4 projectionMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getViewToClipMatrix();
    mat4 viewMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getWorldToViewMatrix();
    mat4 worldMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getDataToWorldMatrix();
    mat4 mvpMatrix = projectionMatrix * viewMatrix * worldMatrix;

    // Turn on clipping plane distances
    glEnable(GL_CLIP_DISTANCE0);
    auto planeNormal = planeNormal_.get();
    vec4 planeEquation = vec4(planeNormal[0], planeNormal[1], planeNormal[2], planeDistance_);

    glEnable(GL_CLIP_DISTANCE1);
    vec4 reversePlaneEquation = vec4(-planeNormal[0], -planeNormal[1], -planeNormal[2], planeReverseDistance_);

    shader_.activate();
    shader_.setUniform("clipPlane", planeEquation);
    shader_.setUniform("reversePlane", reversePlaneEquation);
    shader_.setUniform("dataToClip", mvpMatrix);
    shader_.setUniform("worldMatrix", worldMatrix);
    auto drawer = MeshDrawerGL::getDrawObject(inport_.getData().get());

    // Draw the front faces
    {
        utilgl::activateAndClearTarget(entryPort_);
        //utilgl::CullFaceState cull(GL_BACK);
        drawer.draw();
        utilgl::deactivateCurrentTarget();
    }

    // Draw the back faces
    {
        utilgl::activateAndClearTarget(exitPort_);
        //utilgl::CullFaceState cull(GL_FRONT);
        drawer.draw();
    }

    shader_.deactivate();
    utilgl::deactivateCurrentTarget();
}

}  // namespace inviwo
