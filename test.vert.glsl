#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 2) in vec2 aOffset;

out vec3 fColor;

uniform float time;

void main()
{
    vec3 vPos = aPos * 0.1;
    vPos.xy += aOffset
    	+ (vec2(sin(time), cos(time)) * 0.1);
    
    gl_Position = vec4(vPos, 1.0);
    
    fColor = vec3(0.5) + aPos;
}  