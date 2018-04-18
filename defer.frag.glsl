#version 330 core

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;

in vec2 texCoord;
out vec4 FragColor;

void main() {
	vec4 color = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord);
	vec3 position = texture(gPosition, texCoord);
	FragColor = vec4(texCoord, 0.5, 1.);
}