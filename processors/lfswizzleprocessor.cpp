#include "lfswizzleprocessor.hpp"

#include <inviwo/core/datastructures/camera.h>
#include <inviwo/core/datastructures/coordinatetransformer.h>
#include <inviwo/core/datastructures/image/image.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/rendering/meshdrawergl.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo lfswizzleprocessor::processorInfo_{
    "org.inviwo.lfswizzleprocessor",      // Class identifier
    "Looking Glass LF Swizzle",                // Display name
    "Looking Glass",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo lfswizzleprocessor::getProcessorInfo() const { return processorInfo_; }

lfswizzleprocessor::lfswizzleprocessor()
    : Processor()
    , tileImagePort_("tiledata", DataVec4UInt16::get())
    , outputPort_("output", DataVec4UInt16::get())
    , swizzleShader_("lfswizzle.vert", "lfswizzle.frag") 
{
    addPort(tileImagePort_);
    addPort(outputPort_);
}

void lfswizzleprocessor::process() {
    // outport_.setData(myImage);
    //tileImagePort_.getEditableData().get()->setDimensions(size2_t(4096, 4096));
    outputPort_.getEditableData().get()->setDimensions(size2_t(2560, 1600));

    utilgl::DepthFuncState depthfunc(GL_ALWAYS);
    utilgl::PointSizeState pointsize(1.0f);

    
    utilgl::activateAndClearTarget(*outputPort_.getEditableData().get(), ImageType::ColorDepth);

    swizzleShader_.activate();

    TextureUnit imgUnit;
    utilgl::bindColorTexture(tileImagePort_, imgUnit);
    swizzleShader_.setUniform("tileTexture", imgUnit);
    
    
    utilgl::singleDrawImagePlaneRect();
    utilgl::deactivateCurrentTarget();

    swizzleShader_.deactivate();
}

}  // namespace inviwo
