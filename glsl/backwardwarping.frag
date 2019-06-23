#include "utils/structs.glsl"
#include "utils/sampler2d.glsl"

uniform ImageParameters disparityParameters;
uniform sampler2D disparityColor;
uniform sampler2D disparityDepth;

uniform ImageParameters outportParameters;

// distance is included in this
uniform float disparityScale_x;
uniform float disparityScale_y;
uniform float shift;

uniform vec4 viewport;

void main() {
    vec2 texCoords = (gl_FragCoord.xy - viewport.xy) / (viewport.zw);
    float disparity = texture(disparityDepth, texCoords).x;
    float disparity_x = disparity * disparityScale_x;
    float disparity_y = disparity * disparityScale_y;

    //vec2 dimensions = disparityParameters.dimensions;
    float coord_x = clamp(texCoords.x + disparity_x, 0, 1);
    float coord_y = clamp(texCoords.y - disparity_y, 0, 1);

    FragData0 = texture(disparityColor, vec2(coord_x, coord_y));
    //FragData0 = vec4(abs(disparity_x), 0, 0, 1);
    //FragData0 = vec4(texCoords, 0, 1);
    gl_FragDepth = disparity;
}