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

#ifndef IVW_CLIPPINGRENDERPROCESSOR_H
#define IVW_CLIPPINGRENDERPROCESSOR_H

#include <modules/layereddepth/layereddepthmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/ports/imageport.h>
#include <modules/opengl/shader/shader.h>
#include <inviwo/core/datastructures/camera.h>
#include <inviwo/core/properties/cameraproperty.h>
#include <inviwo/core/interaction/cameratrackball.h>
#include <vector>
#include <inviwo/core/datastructures/geometry/plane.h>

//#include <modules/opengl/ext/glew/include/GL/glew.h>

namespace inviwo {

/** \docpage{org.inviwo.clippingRenderProcessor, clipping Render Processor}
 * ![](org.inviwo.clippingRenderProcessor.png?classIdentifier=org.inviwo.clippingRenderProcessor)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */

/**
 * \class clippingRenderProcessor
 * \brief VERY_BRIEFLY_DESCRIBE_THE_PROCESSOR
 * See https://prideout.net/clip-planes
 */
class IVW_MODULE_LAYEREDDEPTH_API clippingRenderProcessor : public Processor {
public:
    clippingRenderProcessor();
    virtual ~clippingRenderProcessor() override;

    virtual void process() override;

    virtual void initializeResources() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    void onAlignPlaneNormalToCameraNormalToggled();
    bool rayIntersectsPlane(
        const vec3 &rayOrig, const vec3 &rayDir, const vec3 &planeNormal, 
        const float &planeDistance, float &t, float &v);
    void calculatePlaneIntersectionPoints(std::vector<vec3> &out_points, const float &planeDistance, const vec3 &planeNormal);
    void calculatePlaneIntersectionPoint(std::vector<vec3> &out_points, const vec3 &rayOrig, const vec3 &rayDir, const float &planeDistance, const vec3 &planeNormal);
    void sortPlaneIntersectionPoints(std::vector<vec3> &out_points, const vec3 &planeNormal);
    void InviwoPlaneIntersectionPoints(std::vector<vec3> &out_points, const Plane& worldSpacePlane);
    MeshInport inport_;
    ImageOutport entryPort_;
    ImageOutport exitPort_;

    CameraProperty camera_;
    CameraTrackball trackball_;

    FloatVec3Property planeNormal_;
    BoolProperty useCameraNormalAsPlane_;
    FloatProperty planeDistance_;
    FloatProperty planeReverseDistance_;
    Shader shader_;
    Shader faceShader_;

    GLuint front_buffer_id_;
    GLuint back_buffer_id_;
};

}  // namespace inviwo

#endif  // IVW_CLIPPINGRENDERPROCESSOR_H
