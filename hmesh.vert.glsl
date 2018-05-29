#version 330 core

uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 normal, position;

void main() {
	texCoord = aTexCoord;
	normal = aNormal;
	position = aPos;
	gl_Position = vec4(position, 1.0);
}
