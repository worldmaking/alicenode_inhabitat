#version 330 core

uniform mat4 uViewProjectionMatrix;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 normal, position;

void main() {
	texCoord = aTexCoord;
	normal = aNormal;
	position = (aPos * 8.) + vec3(4., 1., 4.);
	
	gl_Position = uViewProjectionMatrix * vec4(position, 1.);
}