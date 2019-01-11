#include <modules/lookingglass/processors/lfentryexitpoints.h>

#include <inviwo/core/datastructures/camera.h>
#include <inviwo/core/datastructures/coordinatetransformer.h>
#include <inviwo/core/datastructures/image/image.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/openglutils.h>
#include <modules/opengl/rendering/meshdrawergl.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo lfentryexitpoints::processorInfo_{
    "org.inviwo.lfentryexitpoints",      // Class identifier
    "Looking Glass Entry Exit Points",                // Display name
    "Looking Glass",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo lfentryexitpoints::getProcessorInfo() const { return processorInfo_; }

lfentryexitpoints::lfentryexitpoints()
    : Processor()
    , inport_("geometry")
    , entryPort_("entry", DataVec4UInt16::get())
    , exitPort_("exit", DataVec4UInt16::get())
    , camera_("camera", "Camera", vec3(0.0f, 0.0f, -2.0f), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), &inport_)
    , capNearClipping_("capNearClipping", "Cap near plane clipping", true)
    , trackball_(&camera_)
    , regionSizeProperty_("size", "Size", 5.0f, 0.0f, 10.0f)
    , entryExitShader_("lfentryexitpoints.vert", "lfentryexitpoints.frag") {

    addPort(inport_);
    addPort(entryPort_, "ImagePortGroup1");
    addPort(exitPort_, "ImagePortGroup1");
    addProperty(regionSizeProperty_);
    addProperty(capNearClipping_);
    addProperty(camera_);
    addProperty(trackball_);
    entryPort_.addResizeEventListener(&camera_);
}

void lfentryexitpoints::process() {
    // outport_.setData(myImage);
    entryPort_.getEditableData().get()->setDimensions(size2_t(4096, 4096));
    exitPort_.getEditableData().get()->setDimensions(size2_t(4096, 4096));
    {
        utilgl::DepthFuncState depthfunc(GL_ALWAYS);
        utilgl::PointSizeState pointsize(1.0f);

        entryExitShader_.activate();
        mat4 modelMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getDataToClipMatrix();
        entryExitShader_.setUniform("dataToClip", modelMatrix);

        
        {
            // generate exit points
            utilgl::activateAndClearTarget(
                *exitPort_.getEditableData().get(), 
                ImageType::ColorDepth
            );
            utilgl::CullFaceState cull(GL_FRONT);
            drawViews();
            utilgl::deactivateCurrentTarget();
        }

        {
            // generate entry points
            utilgl::activateAndClearTarget(
                *entryPort_.getEditableData().get(), 
                ImageType::ColorDepth
            );

            utilgl::CullFaceState cull(GL_BACK);
            drawViews();
            entryExitShader_.deactivate();
            utilgl::deactivateCurrentTarget();
        }
    }
}

void lfentryexitpoints::drawViews()
{
    auto drawer = MeshDrawerGL::getDrawObject(inport_.getData().get());

    mat4 projectionMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getViewToClipMatrix();
    mat4 viewMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getWorldToViewMatrix();
    mat4 worldMatrix = inport_.getData().get()->getCoordinateTransformer(camera_.get()).getDataToWorldMatrix();

    int view = 0;
    size2_t tileSize(819, 455);
    float spacing = 0.1f;
    float width = spacing * 45.0f;
    float offset = (width * 0.5f);
    float viewCone = 40.0f;
    PerspectiveCamera* cam = (PerspectiveCamera*)&camera_.get();
    float size = regionSizeProperty_.get();
    float adjustedSize = size / tanf(glm::radians(cam->getFovy() * 0.5f));
    
    for(int y = 0; y < 9; ++y)
    {
        for(int x = 0; x < 5; ++x)
        {
            float angleAtView = -viewCone * 0.5f + (float)view / (45.0f - 1.0f) * viewCone;
            offset = adjustedSize * tanf(glm::radians(angleAtView));

            mat4 currentViewMatrix = viewMatrix;
            currentViewMatrix[3][0] -= offset;

            mat4 currentProjectionMatrix = projectionMatrix;
            currentProjectionMatrix[2][0] -= offset / (size * cam->getAspectRatio());
            
            mat4 mvpMatrix = currentProjectionMatrix * currentViewMatrix * worldMatrix;
            entryExitShader_.setUniform("dataToClip", mvpMatrix);
            size2_t start(x * tileSize.x, y * tileSize.y);
            glViewport(start.x, start.y, tileSize.x, tileSize.y);
            drawer.draw();
            ++view;
        }
    }
    glViewport(0, 0, 4096, 4096);
    
}

}  // namespace inviwo
