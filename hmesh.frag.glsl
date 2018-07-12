#version 330 core

in vec2 texCoord;
in vec3 normal, position;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;
layout (location = 3) out vec3 FragTexCoord;

void main() {
	// uncomment this line for contours:
	//if (mod(position.y*4.,1.) > 0.2) discard;

	vec3 nnorm = normalize(normal);

	FragColor.rgb = vec3(0);
	FragNormal.xyz = nnorm;
	FragPosition.xyz = position;
	FragTexCoord.xy = texCoord;
}