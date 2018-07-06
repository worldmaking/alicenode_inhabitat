#version 330 core

in vec2 texCoord;
in vec3 normal, position;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;
layout (location = 3) out vec3 FragTexCoord;

void main() {
	FragColor.rgb = normal*0.5+0.5; //vec3(texCoord, 0.);
	FragNormal.xyz = normal;
	FragPosition.xyz = position;
	FragTexCoord.xy = texCoord;
}