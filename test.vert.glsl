#version 330 core
layout (location = 0) in vec3 aPos;
//layout (location = 1) in vec3 aColor;
layout (location = 2) in vec2 aOffset;

out vec3 fColor;

uniform float time;
uniform vec2 offsets[100];

void main()
{
    vec3 vPos = aPos * 0.1;
    vPos.xy += aOffset
    	+ (vec2(sin(time), cos(time)) * 0.1);
    
    //vPos.xy = vPos.xy * (gl_InstanceID / 100.0);
    
    //vec2 offset = offsets[gl_InstanceID];
    //gl_Position = vec4(aPos.xy + offset, 0.0, 1.0);
    gl_Position = vec4(vPos, 1.0);
    //fColor = aColor;
    fColor = vec3(0.5) + aPos;
}  