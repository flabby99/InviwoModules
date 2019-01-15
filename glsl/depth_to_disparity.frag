#include "utils/sampler2d.glsl"
#include "utils/depth.glsl"
#include "utils/structs.glsl"

uniform ImageParameters entryParameters;
uniform sampler2D entryColor;
uniform sampler2D entryDepth;
uniform ImageParameters disparityParameters;

uniform CameraParameters camera;

uniform float cameraBaseline;

// This disparity is not in pixel units yet.
float depth_to_disparity(float depth, CameraParameters camera, float baseline) {
    float near = camera.nearPlane;
    float far = camera.farPlane;
    float focal_length = camera.viewToClip[0][0];

    float disparity = 2.0f * (depth - 1.0f);
    disparity = 2.0f * near * far / (near + far - disparity * (far - near));
    disparity = (baseline * focal_length) / disparity;
    return disparity;
}

void main() {
    vec2 texCoords = gl_FragCoord.xy * disparityParameters.reciprocalDimensions;
    float depth = texture(entryDepth, texCoords).x;

    //gl_FragDepth = depth;
    gl_FragDepth = depth_to_disparity(depth, camera, cameraBaseline);
    FragData0 = texture(entryColor, texCoords);
}
