#ifndef IVW_VERTEX_ARRAY_H
#define IVW_VERTEX_ARRAY_H

#include <modules/layereddepth/layereddepthmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <modules/layereddepth/helpers/vertexbuffer.h>
#include <modules/opengl/inviwoopengl.h>

namespace inviwo {
    class IVW_MODULE_LAYEREDDEPTH_API VertexArray
    {
    public:
        VertexArray();
        ~VertexArray();
        void Bind() const;
        void Unbind() const;
        void Addbuffer_3f(const std::shared_ptr<VertexBuffer>& vb, const GLuint index);
        void Addbuffer_2f(const std::shared_ptr<VertexBuffer>& vb, const GLuint index);
    private:
        GLuint renderer_id_;
        std::list<std::shared_ptr<VertexBuffer>> attached_vbs;
    };
} //namespace inviwo

#endif //IVW_VERTEX_ARRAY_H