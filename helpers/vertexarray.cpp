#include <modules/layereddepth/helpers/vertexarray.h>

namespace inviwo {
    VertexArray::VertexArray()
    {
        glGenVertexArrays(1, &renderer_id_);
    }

    VertexArray::~VertexArray()
    {
        glDeleteVertexArrays(1, &renderer_id_);
        attached_vbs.clear();
    }

    void VertexArray::Bind() const {
        glBindVertexArray(renderer_id_);
    }

    void VertexArray::Unbind() const {
        glBindVertexArray(0);
    }

    //Add a vertex buffer with 3 floats for each vertice at an array index 
    void VertexArray::Addbuffer_3f(const std::shared_ptr<VertexBuffer>& vb, const GLuint index) {
        Bind();
        vb->Bind();
        glEnableVertexAttribArray(index);
        glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, 0, NULL);
        attached_vbs.push_back(vb);
    }

    //Add a vertex buffer with 2 floats for each vertice at an array index 
    void VertexArray::Addbuffer_2f(const std::shared_ptr<VertexBuffer>& vb, const GLuint index)
    {
      Bind();
      vb->Bind();
      glEnableVertexAttribArray(index);
      glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, 0, NULL);
      attached_vbs.push_back(vb);
    }
} //namespace inviwo