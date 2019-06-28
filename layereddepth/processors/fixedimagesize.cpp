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

#include <modules/layereddepth/processors/fixedimagesize.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo fixedimagesize::processorInfo_{
    "org.inviwo.fixedimagesize",      // Class identifier
    "Fixed Image Size",                // Display name
    "Utility",              // Category
    CodeState::Stable,  // Code state
    Tags::CPU,               // Tags
};
const ProcessorInfo fixedimagesize::getProcessorInfo() const { return processorInfo_; }

fixedimagesize::fixedimagesize()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    {

    addPort(outport_);
    addPort(inport_);

    outport_.setHandleResizeEvents(false);
    inport_.setOutportDeterminesSize(false);
}

void fixedimagesize::process() {
    outport_.setData(inport_.getData());
}

void fixedimagesize::propagateEvent(Event* event, Outport* target) {
    if (event->hash() == ResizeEvent::chash()) {
        event->markAsUsed();
    }
    else { 
        Processor::propagateEvent(event, target);
    }
}

}  // namespace inviwo
