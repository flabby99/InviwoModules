#ifndef VERTEX_BUFFER_H
#define VERTEX_BUFFER_H

namespace buffer {
    class VertexBuffer
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

#endif //VERTEX_BUFFER_H