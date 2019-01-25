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
#include <inviwo/core/datastructures/geometry/simplemesh.h>

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
    , useCameraNormalAsPlane_("camNormal", "Use camera normal as plane normal", true)
    , planeDistance_("distance", "Plane distance along normal", 0.0f, -10.0f, 10.0f)
    , planeReverseDistance_("reverse_distance", "Reverse plane distance along normal", 0.0f, -10.0f, 10.0f)
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

clippingRenderProcessor::~clippingRenderProcessor() {
    glDeleteBuffers(1, &front_buffer_id_);
    glDeleteBuffers(1, &back_buffer_id_);
}

void clippingRenderProcessor::onAlignPlaneNormalToCameraNormalToggled() {
    planeNormal_.setReadOnly(useCameraNormalAsPlane_.get());
    if (useCameraNormalAsPlane_.get()) {           
        planeNormal_.set(glm::normalize(camera_.getLookTo() - camera_.getLookFrom()));
    }
}

void clippingRenderProcessor::initializeResources() {
    shader_.build();
    glGenBuffers(1, &front_buffer_id_);
    glGenBuffers(1, &back_buffer_id_);
}

// Parallelepiped is laid out as:
    //      2-----3
    //     /|    /|          y
    //    6-+---7 |          |
    //    | 0---+-1          o--x
    //    |/    |/          /
    //    4-----5          z

void clippingRenderProcessor::calculatePlaneIntersectionPoints(std::vector<vec3> &out_points, const float &planeDistance, const vec3 &planeNormal) {
    const std::vector<vec3>* vertexList;
    mat4 worldMatrix = inport_.getData().get()->getCoordinateTransformer().getDataToWorldMatrix();

    
    if (auto simple = dynamic_cast<const SimpleMesh*>(inport_.getData().get())) {
        vertexList = &simple->getVertexList()->getRAMRepresentation()->getDataContainer();
        
        // Check it is parallelepiped
        if(vertexList->size() == 8) {
            unsigned int num_edges = 12;
            vec3 rayDir;
            vec3 rayOrig;
            vec3 rayEnd;

            // Build a set of edges to test as start->end
            unsigned int points[24] = {
                0, 1, 2, 3, 4, 5, 6, 7, // Test edges facing along x axis
                0, 2, 1, 3, 4, 6, 5, 7, // Test edges facing along y axis
                0, 4, 1, 5, 3, 7, 2, 6 // Test edges facing along z axis
            };
            
            auto geom = inport_.getData();
            auto worldToData = geom->getCoordinateTransformer().getWorldToDataMatrix();
            auto worldToDataNormal = glm::transpose(glm::inverse(worldToData));
            vec3 planePos = planeNormal * planeDistance;
            auto dataSpacePos = vec3(worldToData * vec4(planePos, 1.0));
            auto dataSpaceOrigin = vec3(worldToData * vec4(0, 0, 0, 1.0));
            auto dataSpaceDistance = glm::distance(dataSpaceOrigin, dataSpacePos);
            auto dataSpaceNormal = glm::normalize(vec3(worldToDataNormal * vec4(planeNormal, 0.0)));

            for(unsigned int i = 0; i < num_edges; ++i) {   
                // Rays are in world space
                //rayOrig = vec3(worldMatrix * vec4(vertexList->at(points[2 * i]), 1.f));
                //rayEnd = vec3(worldMatrix * vec4(vertexList->at(points[2 * i + 1]), 1.f));
                rayOrig = vertexList->at(points[2 * i]);
                rayEnd = vertexList->at(points[2 * i + 1]);
                rayDir = rayEnd - rayOrig;
                std::cout << i << rayOrig << rayDir << std::endl;
                calculatePlaneIntersectionPoint(out_points, rayOrig, rayDir, dataSpaceDistance, dataSpaceNormal);
            }
        }
        else {
            throw Exception("Unsupported mesh type, only parallelepipeds are supported");
        }
    }
    else {
        throw Exception("Unsupported mesh type, only simple meshes are supported");
    }
    
}

void clippingRenderProcessor::calculatePlaneIntersectionPoint(std::vector<vec3> &out_points, const vec3 &rayOrig, const vec3 &rayDir, const float &planeDistance, const vec3 &planeNormal) {
    float v = 0.f;
    float t = 0.f;
    if (rayIntersectsPlane(rayOrig, rayDir, planeNormal, planeDistance, t, v) && t > 0.f && t <= 1.f) {
        out_points.push_back(rayOrig + (t * rayDir));
    }
}

// This is based on subbing a ray r0 + t * rd into a plane equation Ax + By + Cz + D = 0 and solving for t
// If t lies between 0 and 1, then the intersection point of the ray and the plane lies on the ray
bool clippingRenderProcessor::rayIntersectsPlane(
    const vec3 &rayOrig, const vec3 &rayDir, const vec3 &planeNormal, 
    const float &planeDistance, float &t, float &v) {
    v = glm::dot(planeNormal, rayDir);
    // If the plane is parallel to the ray, they will not intersect
    if (v < 0.00001f)
        return false;
    float dotProduct = glm::dot(planeNormal, rayOrig);
    t = -(dotProduct + planeDistance) / v;
    return true;
}

void clippingRenderProcessor::sortPlaneIntersectionPoints(std::vector<vec3> &out_points, const vec3 &planeNormal) {
    if (out_points.size() == 0)
        return;
    const vec3 origin = out_points[0];
    
    std::sort(out_points.begin(), out_points.end(), [&](const vec3 &lhs, const vec3 &rhs) -> bool {
        vec3 v;
        v = glm::cross((lhs - origin), (rhs - origin));
        return glm::dot(v, planeNormal) < 0;
    });
}

void clippingRenderProcessor::InviwoPlaneIntersectionPoints(std::vector<vec3> &out_points, const Plane& worldSpacePlane) {
    auto geom = inport_.getData();
    auto worldToData = geom->getCoordinateTransformer().getWorldToDataMatrix();
    auto worldToDataNormal = glm::transpose(glm::inverse(worldToData));
    auto dataSpacePos = vec3(worldToData * vec4(worldSpacePlane.getPoint(), 1.0));
    auto dataSpaceNormal =
        glm::normalize(vec3(worldToDataNormal * vec4(worldSpacePlane.getNormal(), 0.0)));
    Plane plane(dataSpacePos, dataSpaceNormal);

    if (auto simple = dynamic_cast<const SimpleMesh*>(inport_.getData().get())) {
        const std::vector<vec3>* vertexList = &simple->getVertexList()->getRAMRepresentation()->getDataContainer();
        
        // Check it is parallelepiped
        if(vertexList->size() == 8) {
            unsigned int num_edges = 12;
            vec3 rayDir;
            vec3 rayOrig;
            vec3 rayEnd;

            // Build a set of edges to test as start->end
            unsigned int points[24] = {
                0, 1, 2, 3, 4, 5, 6, 7, // Test edges facing along x axis
                0, 2, 1, 3, 4, 6, 5, 7, // Test edges facing along y axis
                0, 4, 1, 5, 3, 7, 2, 6 // Test edges facing along z axis
            };
            
            for(unsigned int i = 0; i < num_edges; ++i) {   
                rayOrig = vertexList->at(points[2 * i]);
                rayEnd = vertexList->at(points[2 * i + 1]);
                rayDir = rayEnd - rayOrig;
                if(plane.isInside(rayOrig)) {
                    if(plane.isInside(rayEnd)) {
                        //Both lie inside the plane, no intersection point
                    }
                    else { //the end lies outside the plane so there is intersection
                        vec3 intersection = 
                        plane
                                .getIntersection(rayOrig, rayEnd)
                                .intersection_;
                        out_points.push_back(intersection);
                    }
                }
                else {
                    if(plane.isInside(rayEnd)) { // the end lies inside the plane
                        vec3 intersection = 
                        plane
                                .getIntersection(rayOrig, rayEnd)
                                .intersection_;
                        out_points.push_back(intersection);
                    }
                    //Otherwise the both lie outside of the plane so there is no need to do anything
                }
            }
        }
        else {
            throw Exception("Unsupported mesh type, only parallelepipeds are supported");
        }
    }
    else {
        throw Exception("Unsupported mesh type, only simple meshes are supported");
    }
    
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
    mat4 vpMatrix = projectionMatrix * viewMatrix;

    auto planeNormal = planeNormal_.get();
    auto planeReverseNormal = vec3(-planeNormal[0], -planeNormal[1], -planeNormal[2]);
    
    vec4 planeEquation = vec4(planeNormal[0], planeNormal[1], planeNormal[2], planeDistance_);
    vec4 reversePlaneEquation = vec4(planeReverseNormal[0], planeReverseNormal[1], planeReverseNormal[2], planeReverseDistance_);

    shader_.activate();
    shader_.setUniform("clipPlane", planeEquation);
    shader_.setUniform("reversePlane", reversePlaneEquation);
    shader_.setUniform("dataToClip", mvpMatrix);
    shader_.setUniform("worldMatrix", worldMatrix);
    auto drawer = MeshDrawerGL::getDrawObject(inport_.getData().get());

    // Draw the front faces
    {
        // Turn on clipping plane distances
        glEnable(GL_CLIP_DISTANCE0);
        glEnable(GL_CLIP_DISTANCE1);
        utilgl::activateAndClearTarget(entryPort_);
        utilgl::CullFaceState cull(GL_BACK);
        drawer.draw();
    }
    
    // Draw the front facing polygon intersection
    {
        std::vector<vec3> points;
        // Maximum 6 interesection points
        points.reserve(6);
        //calculatePlaneIntersectionPoints(points, planeDistance_.get(), planeNormal);
        Plane forwardPlane = Plane(-planeDistance_.get() * planeNormal, planeNormal);
        InviwoPlaneIntersectionPoints(points, forwardPlane);
        sortPlaneIntersectionPoints(points, planeNormal);
        std::cout << "Polygon front points:" << std::endl;
        for (vec3 point : points) {
            std::cout << point << std::endl;
        }
        glBindBuffer(GL_ARRAY_BUFFER, front_buffer_id_);
        glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(GLfloat), points.data(), GL_STATIC_DRAW);

        glDisable(GL_CLIP_DISTANCE0);
        glDisable(GL_CLIP_DISTANCE1);

        //shader_.setUniform("dataToClip", vpMatrix);
        glDrawArrays(GL_TRIANGLE_FAN, 0, points.size());
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        utilgl::deactivateCurrentTarget();
    }

    // Draw the back faces
    {
        // Turn on clipping plane distances
        glEnable(GL_CLIP_DISTANCE0);
        glEnable(GL_CLIP_DISTANCE1);
        utilgl::activateAndClearTarget(exitPort_);
        utilgl::CullFaceState cull(GL_FRONT);
        //shader_.setUniform("dataToClip", mvpMatrix);
        drawer.draw();
    }

    // Draw the back facing polygon intersection
    {
        std::vector<vec3> points;
        // Maximum 6 interesection points
        points.reserve(6);
        //calculatePlaneIntersectionPoints(points, planeReverseDistance_.get(), planeReverseNormal);
        Plane backwardPlane = Plane(-planeReverseDistance_.get() * planeReverseNormal, planeReverseNormal);
        InviwoPlaneIntersectionPoints(points, backwardPlane);
        sortPlaneIntersectionPoints(points, planeReverseNormal);
        std::cout << "Polygon back points:" << std::endl;
        for (vec3 point : points) {
            std::cout << point << std::endl;
        }
        glBindBuffer(GL_ARRAY_BUFFER, back_buffer_id_);
        glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(GLfloat), points.data(), GL_STATIC_DRAW);

        glDisable(GL_CLIP_DISTANCE0);
        glDisable(GL_CLIP_DISTANCE1);

        //shader_.setUniform("dataToClip", mvpMatrix);
        glDrawArrays(GL_TRIANGLE_FAN, 0, points.size());
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        utilgl::deactivateCurrentTarget();
    }

    shader_.deactivate();
}

}  // namespace inviwo
