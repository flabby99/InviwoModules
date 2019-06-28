#ifndef IVW_VECTOR_SELECTOR_H
#define IVW_VECTOR_SELECTOR_H

#include <modules/layereddepth/layereddepthmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/ports/datainport.h>

namespace inviwo {
    class IVW_MODULE_LAYEREDDEPTH_API imageVectorSelector : public Processor {
    public:
        imageVectorSelector();
        virtual ~imageVectorSelector() = default;

        virtual void process() override;

        virtual const ProcessorInfo getProcessorInfo() const override;
        static const ProcessorInfo processorInfo_;
    private:
        DataInport<std::vector<std::shared_ptr<Image>>> inport_;
        ImageOutport outport_;

        IntProperty imageNumber_;
    };
} //namespace inviwo 
#endif //IVW_VECTOR_SELECTOR_H

