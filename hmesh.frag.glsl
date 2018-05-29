#version 330 core

in vec2 texCoord;
in vec3 normal, position;
layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;


void main() {
	FragColor.rgb = vec3(texCoord, 0.5);
	FragNormal.xyz = normal;
	FragPosition.xyz = position;
	//gl_FragDepth = computeDepth(p, uViewProjectionMatrix);
}