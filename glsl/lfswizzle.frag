#include "utils/structs.glsl"

uniform ImageParameters outportParameters_;
uniform sampler2D tileTexture;

vec2 texArr(vec3 uvz) 
{
    const vec4 tile = vec4(5, 9, 0.999755859, 0.999755859);
    float z = floor(uvz.z * tile.x * tile.y);
    float x = (mod(z, tile.x) + uvz.x) / tile.x;
    float y = (floor(z / tile.x) + uvz.y) / tile.y;
    return clamp(vec2(x, y), vec2(0), vec2(1)) * tile.zw;
}

void main() 
{
    float subp = 1.0 / (2560.0 * 3.0);
    float tilt = 1600.0 / (2560.0 * -5.444456577301025);
    float screenInches = 2560.0 / 338.0;
    float pitch = 47.578365325927737 * screenInches;
    pitch *= cos(atan(1.0 / -5.444456577301025)); 
    float center = 0.4418478012084961;
    float invView = 1.0;
    float flipX = 0.0;
    float flipY = 0.0;

    vec4 aspect;
    aspect.x = 2560.0 / 1600.0;
    aspect.y = 2560.0 / 1600.0;
    aspect.z = 0.0;
    aspect.w = 0.0;

    vec3 nuv = vec3(gl_FragCoord.xy / vec2(2560, 1600), 0);
    nuv -= 0.5;
    if ((aspect.x > aspect.y && aspect.z < 0.5) ||
        (aspect.x < aspect.y && aspect.z > 0.5)
    ){
        nuv.x *= aspect.x / aspect.y;
    } else {
        nuv.y *= aspect.y / aspect.x;
    }

    nuv += 0.5;
    if(nuv.x < 0.0 || nuv.x > 1.0 || nuv.y < 0.0 || nuv.y > 1.0)
    {
        discard;
    }
    else
    {

        nuv.x = (1.0 - flipX) * nuv.x + flipX * (1.0 - nuv.x);
        nuv.y = (1.0 - flipY) * nuv.y + flipY * (1.0 - nuv.y);

        vec4 rgb[3];
        for (int i; i < 3; i++) {
            nuv.z = (nuv.x + i * subp + nuv.y * tilt) * pitch - center;
            nuv.z = mod(nuv.z + ceil(abs(nuv.z)), 1.0);
            nuv.z = (1.0 - invView) * nuv.z + invView * (1.0 - nuv.z);
            rgb[i] = texture(tileTexture, texArr(nuv)); 
        }

        FragData0 = vec4(rgb[0].r, rgb[1].g, rgb[2].b, 1);
    }
}
