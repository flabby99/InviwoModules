#include <modules/layereddepth/processors/vectorimageselector.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo imageVectorSelector::processorInfo_{
    "org.inviwo.vectorImageSelector",      // Class identifier
    "Vector Image Selector",                // Display name
    "Image Processing",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo imageVectorSelector::getProcessorInfo() const { return processorInfo_; }

imageVectorSelector::imageVectorSelector()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    , imageNumber_("number", "Image Number", 0, 0, 50, 1)
    {
        addPort(inport_);
        addPort(outport_);

        addProperty(imageNumber_);
}

void imageVectorSelector::process() {
    imageNumber_.setMaxValue(inport_.getData()->size() - 1);
    outport_.setData(inport_.getData()->at(imageNumber_));
}

} //namespace inviwo