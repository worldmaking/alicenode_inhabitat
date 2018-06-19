#version 330 core

uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix, uLandMatrix, uLandMatrixInverse;
uniform sampler2D uLandTex;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 normal, position;
out float steepness;

void main() {
	texCoord = aTexCoord;
	normal = aNormal;

	// basic grid mesh is 0..1, need to scale that to the world
	position = (uLandMatrixInverse * vec4(aPos, 1.)).xyz;

	// get the landscape data (normal + height) from the texture:
	vec4 land = texture(uLandTex, texCoord.xy);

	// height (land.w) needs to be scaled to the world
	vec3 deform = (uLandMatrixInverse * vec4(0., land.w, 0., 1.)).xyz;
	position += deform;
	normal = land.xyz;

	// steepness depends on normal:
	//steepness = abs(1. - dot(normal, vec3(0, 1, 0)));
	steepness = abs(1. - normal.y);
	
	gl_Position = uViewProjectionMatrix * vec4(position, 1.);
}