#version 330 core

uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix, uLandMatrix, uLandMatrixInverse;
uniform sampler2D uLandTex;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 normal, position;

void main() {
	texCoord = aTexCoord;
	normal = aNormal;

	// basic grid mesh is 0..1, need to scale that to the world
	position = (uLandMatrixInverse * vec4(aPos, 1.)).xyz;

	vec4 land = texture(uLandTex, texCoord.xy);
	normal = land.xyz;

	vec3 deform = (uLandMatrixInverse * vec4(0., land.w, 0., 1.)).xyz;

	position += deform;

	//position.y += land.w;

	gl_Position = uViewProjectionMatrix * vec4(position, 1.);
}