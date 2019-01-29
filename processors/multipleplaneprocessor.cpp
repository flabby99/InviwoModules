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

#include <modules/layereddepth/processors/multipleplaneprocessor.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo multipleplaneProcessor::processorInfo_{
    "org.inviwo.multipleplaneProcessor",      // Class identifier
    "Multipleplane Processor",                // Display name
    "Rendering",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo multipleplaneProcessor::getProcessorInfo() const { return processorInfo_; }

multipleplaneProcessor::multipleplaneProcessor()
    : Processor()
    , firstImage_("first")
    , secondImage_("second")
    , outport_("outport")
    , camera_("camera", "Camera", vec3(0.0f, 0.0f, -2.0f), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), &firstImage_)
    , trackball_(&camera_)
    , shouldShear_("shouldShear", "Should Use Shear Projection", true)
    , regionSizeProperty_("size", "Size", 5.0f, 0.0f, 10.0f)
    , verticalAngleProperty_("vertical_angle", "Vertical Angle", 0.0f, -60.0f, 60.0f, 0.1f)
    , viewConeProperty_("view_cone", "View cone", 40.0f, 0.0f, 90.0f, 0.1f)
    , shader_("multipleplaneProcessor.vert", "multipleplaneProcessor.frag")
    , gridPosition_("position", "Grid Position", 0, 0, 44, 1)
    , grid_(NULL)
    , outportXDim_(819)
    , outportYDim_(455)
    , va_()
    {
    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addPort(outport_);
    addPort(firstImage_);
    addPort(secondImage_);
    addProperty(camera_);
    addProperty(trackball_);
    addProperty(gridPosition_);
    addProperty(regionSizeProperty_);
    addProperty(viewConeProperty_);
    addProperty(verticalAngleProperty_);
    addProperty(shouldShear_);

    // Use this to define a fixed size vertex grid
    outport_.setDimensions(size2_t(outportXDim_, outportYDim_));
    width_ = outportXDim_;
    height_ = outportYDim_;
    outport_.setHandleResizeEvents(false);
    (&firstImage_)->setOutportDeterminesSize(true);
    (&secondImage_)->setOutportDeterminesSize(true);
}

/*
void multipleplaneProcessor::~multipleplaneProcessor() {
    
}
*/

void multipleplaneProcessor::initializeResources() {
    createVertexGrid(grid_, width_, height_);
    vb_ = std::make_shared<VertexBuffer>(
        grid_, (unsigned int)sizeof(float) * 2 * width_ * height_
    );
    delete[] grid_;
    va_.Addbuffer_2f(vb_, 0);

    shader_.build();
}

// Create a set of vertices in the pixel positions
// These will be in UV co-ordinates in 0, 1
void multipleplaneProcessor::createVertexGrid(float* grid, const unsigned int width, const unsigned int height) {
    float width_increment = 1.0f / width;
    float height_increment = 1.0f / width;
    float start_width = width_increment;
    float start_height = height_increment;
    grid = new float[width * height * 2];
    for (unsigned int i = 0; i < width; ++i) {
      for (unsigned int j = 0; j < height; ++j) {
        grid[2 * (i * height + j)] = (float)(start_width + width_increment * i);
        grid[2 * (i * height + j) + 1] = (float)(start_height + height_increment * j);
      }
    }
}

void multipleplaneProcessor::process() {
    ImageInport* ports_pointer_list[2] = {&firstImage_, &secondImage_};
    
    // Set up correct states
    {
        glEnable(GL_BLEND);

        // glBlendFunc(Src_blend_factor, dest_blend_factor)
        // dest is the value already in the framebuffer
        glBlendFunc(GL_ONE_MINUS_DST_ALPHA, GL_ONE);

        // Don't smooth points
        glDisable(GL_POINT_SMOOTH);
        glHint(GL_POINT_SMOOTH_HINT, GL_FASTEST);

        // Allow the size of a point to be specified in the shader
        glEnable(GL_PROGRAM_POINT_SIZE);
    }

    // Initialize shaders, textures, targets and uniforms
    {
        shader_.activate();
        utilgl::activateAndClearTarget(outport_);
    }

    // Do preliminary calculations
    {
        mat4 projectionMatrix = camera_.get().getProjectionMatrix();
        mat4 viewMatrix = camera_.get().getViewMatrix();

        size2_t tileSize(819, 455);
        float viewCone = viewConeProperty_.get();
        PerspectiveCamera* cam = (PerspectiveCamera*)&camera_.get();
        float size = regionSizeProperty_.get();
        float verticalAngle = verticalAngleProperty_.get();
        float adjustedSize = size / tanf(glm::radians(cam->getFovy() * 0.5f));
        float offsetX = 0;
        float offsetY = 0;
        
        /*
        for(int y = 0; y < 9; ++y)
        {
            for(int x = 0; x < 5; ++x)
            {
        */
        int view = gridPosition_;
        float angleAtView = -viewCone * 0.5f + (float)view / (45.0f - 1.0f) * viewCone;
        offsetX = adjustedSize * tanf(glm::radians(angleAtView));
        offsetY = adjustedSize * tanf(glm::radians(verticalAngle));

        mat4 currentViewMatrix = viewMatrix;
        currentViewMatrix[3][0] -= offsetX;
        currentViewMatrix[3][1] -= offsetY;
        

        mat4 currentProjectionMatrix = projectionMatrix;
        if (shouldShear_.get()) {
            currentProjectionMatrix[2][0] -= offsetX / (size * cam->getAspectRatio());
            currentProjectionMatrix[2][1] -= offsetY / size;
        }            
        mat4 vpMatrix = currentProjectionMatrix * currentViewMatrix;
        mat4 vpMatrixInverse = (
            camera_.get().getInverseViewMatrix() *
            camera_.get().getInverseProjectionMatrix()
        );
        mat4 transformMatrix = vpMatrix * vpMatrixInverse;
        shader_.setUniform("transformMatrix", transformMatrix);
        /*
        size2_t start(x * tileSize.x, y * tileSize.y);
        glViewport(start.x, start.y, tileSize.x, tileSize.y);
        
        ++view;
        
            }
        }
        glViewport(0, 0, 4096, 4096);
        */

    }

    for(auto inport : ports_pointer_list) {
        // Set up the source
        {
            TextureUnitContainer units;
            utilgl::bindAndSetUniforms(shader_, units, *inport->getData(), "tex0",
                                    ImageType::ColorDepth);
        }
        
        // Draw the set of vertices
        {
            int num_vertices = width_ * height_;
            va_.Bind();
            glDrawArrays(GL_POINTS, 0, num_vertices);
        }
    }

    // Clean up
    {   
        utilgl::deactivateCurrentTarget();
        shader_.deactivate();
        
        glDisable(GL_BLEND);
        glEnable(GL_POINT_SMOOTH);
    }
}

}  // namespace inviwo
