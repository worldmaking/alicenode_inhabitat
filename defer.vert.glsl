#version 330 core
uniform vec3 eye;

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 ray_direction;

void main() {
	gl_Position = vec4(aPos, 0., 1.0);
	texCoord = aTexCoord;


}
