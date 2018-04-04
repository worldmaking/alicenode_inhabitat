#version 330 core

uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix;

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 ray, origin;

void main() {
	gl_Position = vec4(aPos, 0., 1.0);
	texCoord = aTexCoord;
	
	// project a clip coordinate out into the world
	vec4 near = uViewProjectionMatrixInverse * vec4(gl_Position.xy, -1, 1);
	vec4 far = uViewProjectionMatrixInverse * vec4(gl_Position.xy, 1, 1);
	vec3 near3 = near.xyz/near.w;
	vec3 far3 = far.xyz/far.w;
	// get ray from this:
	ray = far3 - near3;
	
	// in theory origin is just the world-position
	// in practice we might want to add near-clip... 
	origin = near3;
	
}
