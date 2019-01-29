#ifndef VERTEX_ARRAY_H
#define VERTEX_ARRAY_H

#include <modules/layereddepth/include/vertexbuffer.h>

namespace buffer {
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
} //namespace buffer

#endif //VERTEX_ARRAY_H