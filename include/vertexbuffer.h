#ifndef IVW_VERTEX_BUFFER_H
#define IVW_VERTEX_BUFFER_H

#include <modules/layereddepth/layereddepthmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

namespace buffer {
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
} //namespace buffer

#endif //IVW_VERTEX_BUFFER_H