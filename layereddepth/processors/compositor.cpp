/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2018 Inviwo Foundation
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

#include <modules/layereddepth/processors/compositor.h>
#include <modules/opengl/image/imagegl.h>
#include <modules/opengl/texture/textureutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo LayeredCompositeProcessor::processorInfo_{
    "org.inviwo.ImageCompositeLayeredProcessorGL",  // Class identifier
    "Layered Image Composite",                       // Display name
    "Image Operation",                       // Category
    CodeState::Experimental,                 // Code state
    Tags::GL,                                // Tags
};
const ProcessorInfo LayeredCompositeProcessor::getProcessorInfo() const { return processorInfo_; }

LayeredCompositeProcessor::LayeredCompositeProcessor()
    : Processor()
    , imageInport1_("front")
    , imageInport2_("back")
    , outport_("outport")
    , compositor_("layeredcomposite.frag") {
    addPort(imageInport1_);
    addPort(imageInport2_);
    addPort(outport_);
}

void LayeredCompositeProcessor::process() {
    utilgl::activateTargetAndCopySource(outport_, imageInport2_);
    utilgl::deactivateCurrentTarget();

    compositor_.composite(imageInport1_, outport_, ImageType::ColorDepth);
}

}  // namespace inviwo
