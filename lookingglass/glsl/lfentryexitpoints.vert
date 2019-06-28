uniform mat4 dataToClip = mat4(1);

out vec4 color_;
out vec3 texCoord_;

void main() {
    color_ = in_Color;
    texCoord_ = in_TexCoord;
    gl_Position = dataToClip * in_Vertex;
}
