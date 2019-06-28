/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2012-2018 Inviwo Foundation
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

#include <modules/layereddepth/processors/multiplaneraycaster.h>
#include <inviwo/core/io/serialization/serialization.h>
#include <inviwo/core/io/serialization/versionconverter.h>
#include <inviwo/core/interaction/events/keyboardevent.h>
#include <modules/opengl/volume/volumegl.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/volume/volumeutils.h>
#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/util/rendercontext.h>

namespace inviwo {

const ProcessorInfo LayeredRaycaster::processorInfo_{
    "org.inviwo.LayeredVolumeRaycaster",  // Class identifier
    "Layered Volume Raycaster",            // Display name
    "Volume Rendering",            // Category
    CodeState::Stable,             // Code state
    "GL, DVR, Raycasting"          // Tags
};

LayeredRaycaster::LayeredRaycaster()
    : Processor()
    , firstShader_("firstpasslayeredraycasting.frag", false)
    , shader_("layeredraycasting.frag", false)
    , volumePort_("volume")
    , entryPort_("entry")
    , exitPort_("exit")
    , backgroundPort_("bg")
    , outport_("outport")
    , channel_("channel", "Render Channel")
    , raycasting_("raycaster", "Raycasting")
    , isotfComposite_("isotfComposite", "TF & Isovalues", &volumePort_,
                      InvalidationLevel::InvalidResources)
    , camera_("camera", "Camera")
    , lighting_("lighting", "Lighting", &camera_)
    , positionIndicator_("positionindicator", "Position Indicator")
    , toggleShading_("toggleShading", "Toggle Shading", [this](Event* e) { toggleShading(e); },
                     IvwKey::L)
    , rayLengthBlock_("rayLengthBlock", "Ray Length Block", 0.0f, 0.0f, 2.0f, 0.01f)
    , numClips_("num_planes", "Number of clips", 2, 1, 16, 1)
    , xDim_("xdim", "Image width", 819, 256, 1024, 1)
    , yDim_("ydim", "Image height", 455, 256, 512, 1)
    {

    shader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });
    firstShader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    addPort(volumePort_, "VolumePortGroup");
    addPort(entryPort_, "ImagePortGroup1");
    addPort(exitPort_, "ImagePortGroup1");
    addPort(outport_);
    addPort(backgroundPort_, "ImagePortGroup1");

    backgroundPort_.setOptional(true);

    channel_.addOption("Channel 1", "Channel 1", 0);
    channel_.setSerializationMode(PropertySerializationMode::All);
    channel_.setCurrentStateAsDefault();

    volumePort_.onChange([this]() {
        if (volumePort_.hasData()) {
            size_t channels = volumePort_.getData()->getDataFormat()->getComponents();

            if (channels == channel_.size()) return;

            std::vector<OptionPropertyIntOption> channelOptions;
            for (size_t i = 0; i < channels; i++) {
                channelOptions.emplace_back("Channel " + toString(i + 1),
                                            "Channel " + toString(i + 1), static_cast<int>(i));
            }
            channel_.replaceOptions(channelOptions);
            channel_.setCurrentStateAsDefault();
        }
    });
    backgroundPort_.onConnect([&]() { this->invalidate(InvalidationLevel::InvalidResources); });
    backgroundPort_.onDisconnect([&]() { this->invalidate(InvalidationLevel::InvalidResources); });

    // change the currently selected channel when a pre-computed gradient is selected
    raycasting_.gradientComputation_.onChange([this]() {
        if (channel_.size() == 4) {
            if (raycasting_.gradientComputation_.get() ==
                RaycastingProperty::GradientComputation::PrecomputedXYZ) {
                channel_.set(3);
            } else if (raycasting_.gradientComputation_.get() ==
                       RaycastingProperty::GradientComputation::PrecomputedYZW) {
                channel_.set(0);
            }
        }
    });

    addProperty(channel_);
    addProperty(raycasting_);
    addProperty(isotfComposite_);

    addProperty(camera_);
    addProperty(lighting_);
    addProperty(positionIndicator_);
    addProperty(toggleShading_);

    addProperty(numClips_);
    addProperty(xDim_);
    addProperty(yDim_);
    addProperty(rayLengthBlock_);
    
    numClips_.onChange(
        [this]() {initialiseImageData(); });
    xDim_.onChange(
        [this]() {initialiseImageData(); });
    yDim_.onChange(
        [this]() {initialiseImageData(); });

    outImages_ = std::make_shared<std::vector<std::shared_ptr<Image>>>();
    initialiseImageData();
}

const ProcessorInfo LayeredRaycaster::getProcessorInfo() const { return processorInfo_; }

void LayeredRaycaster::initializeResources() {
    utilgl::addDefines(firstShader_, raycasting_, isotfComposite_, camera_, lighting_,
                       positionIndicator_);
    utilgl::addShaderDefinesBGPort(firstShader_, backgroundPort_);
    firstShader_.build();
    utilgl::addDefines(shader_, raycasting_, isotfComposite_, camera_, lighting_,
                       positionIndicator_);
    utilgl::addShaderDefinesBGPort(shader_, backgroundPort_);
    shader_.build();
}

void LayeredRaycaster::initialiseImageData() {
    size2_t dim = size2_t(xDim_, yDim_);
    outImages_->clear();
    outImages_->reserve(numClips_.get());
    auto type = DataVec4UInt16::get();
    for(int i = 0; i < numClips_; ++i){
        auto outImage = std::make_shared<Image>(dim, type);
        outImages_->push_back(outImage);
    }
}

void LayeredRaycaster::process() {
    if (volumePort_.isChanged()) {
        auto newVolume = volumePort_.getData();

        if (newVolume->hasRepresentation<VolumeGL>()) {
            loadedVolume_ = newVolume;
        } else {
            dispatchPool([this, newVolume]() {
                RenderContext::getPtr()->activateLocalRenderContext();
                newVolume->getRep<kind::GL>();
                glFinish();
                dispatchFront([this, newVolume]() {
                    loadedVolume_ = newVolume;
                    invalidate(InvalidationLevel::InvalidOutput);
                });
            });
        }
    }

    if (!loadedVolume_) return;
    if (!loadedVolume_->hasRepresentation<VolumeGL>()) {
        LogWarn("No GL rep !!!");
        return;
    }

    // Render the first view not using the picking layer
    {
        firstShader_.activate();
        TextureUnitContainer units;
        utilgl::bindAndSetUniforms(firstShader_, units, *loadedVolume_, "volume");
        utilgl::bindAndSetUniforms(firstShader_, units, isotfComposite_);
        utilgl::bindAndSetUniforms(firstShader_, units, exitPort_, ImageType::ColorDepth);
        utilgl::setUniforms(firstShader_, camera_, lighting_, raycasting_, positionIndicator_,
                            channel_, rayLengthBlock_);
        if (backgroundPort_.hasData()) {
            utilgl::bindAndSetUniforms(firstShader_, units, backgroundPort_, ImageType::ColorDepthPicking);
        }

        utilgl::bindAndSetUniforms(firstShader_, units, entryPort_, ImageType::ColorDepthPicking);
        firstShader_.setUniform("rayLengthScale", (float)numClips_.get());
        
        utilgl::activateAndClearTarget(*outImages_->at(0), ImageType::ColorDepthPicking);

        utilgl::singleDrawImagePlaneRect();
        firstShader_.deactivate();
    }

    //Render the other views
    {
        shader_.activate();
        TextureUnitContainer units;
        utilgl::bindAndSetUniforms(shader_, units, *loadedVolume_, "volume");
        utilgl::bindAndSetUniforms(shader_, units, isotfComposite_);
        utilgl::bindAndSetUniforms(shader_, units, exitPort_, ImageType::ColorDepth);
        utilgl::bindAndSetUniforms(shader_, units, entryPort_, ImageType::ColorDepth);
        utilgl::setUniforms(shader_, camera_, lighting_, raycasting_, positionIndicator_,
                            channel_, rayLengthBlock_);
        if (backgroundPort_.hasData()) {
            utilgl::bindAndSetUniforms(shader_, units, backgroundPort_, ImageType::ColorDepthPicking);
        }

        for (int i = 1; i < numClips_.get(); ++i) {
            TextureUnitContainer newUnits;
            utilgl::bindAndSetUniforms(shader_, newUnits, *outImages_->at(i - 1), "position", ImageType::ColorPicking);
            shader_.setUniform("rayLengthScale", (float)(numClips_.get() - i));
            
            utilgl::activateAndClearTarget(*outImages_->at(i), ImageType::ColorDepthPicking);

            utilgl::singleDrawImagePlaneRect();
        }
        shader_.deactivate();
    }
    outport_.setData(outImages_);
    utilgl::deactivateCurrentTarget();
}

void LayeredRaycaster::toggleShading(Event*) {
    if (lighting_.shadingMode_.get() == ShadingMode::None) {
        lighting_.shadingMode_.set(ShadingMode::Phong);
    } else {
        lighting_.shadingMode_.set(ShadingMode::None);
    }
}

// override to do member renaming.
void LayeredRaycaster::deserialize(Deserializer& d) {
    util::renamePort(d, {{&entryPort_, "entry-points"}, {&exitPort_, "exit-points"}});
    Processor::deserialize(d);
}

}  // namespace inviwo
