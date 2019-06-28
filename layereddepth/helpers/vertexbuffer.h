#ifndef IVW_VERTEX_BUFFER_H
#define IVW_VERTEX_BUFFER_H

#include <modules/layereddepth/layereddepthmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <modules/opengl/inviwoopengl.h>

namespace inviwo {
    class IVW_MODULE_LAYEREDDEPTH_API VertexBuffer
    {
    public:
        VertexBuffer(const void* data, unsigned int size);
        ~VertexBuffer();
        void Bind() const;
        void Unbind() const;

    private:
        GLuint renderer_id_;
    };
} //namespace inviwo

#endif //IVW_VERTEX_BUFFER_H