#version 330 core

in vec2 texCoord;
in vec3 normal, position;
layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;


void main() {
	FragColor.rgb = vec3(cheap_self_occlusion);
	FragNormal.xyz = normal4(p, .01);
	FragPosition.xyz = p;
	gl_FragDepth = computeDepth(p, uViewProjectionMatrix);
}