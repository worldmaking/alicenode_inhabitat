#version 330 core

uniform sampler2D uFungusTex;

in vec2 texCoord;
in vec3 normal, position;
in float steepness;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;

void main() {
	FragColor.rgb = vec3(0.);
 	
	float fungus = texture(uFungusTex, texCoord).r;
	if (fungus > 0.) {
		FragColor.rgb = vec3(fungus * 0.7 + 0.2);
	} 
	FragColor.rgb *= vec3(1. - steepness*2.);

	FragColor.rgb *= vec3(min(1., position.y * 2.));

	FragNormal.xyz = normal;
	FragPosition.xyz = position;
}