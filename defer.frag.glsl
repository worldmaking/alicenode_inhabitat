#version 330 core

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;

in vec2 texCoord;
out vec4 FragColor;

void main() {
	FragColor = vec4(texCoord, 0.5, 1.);
	//FragColor = texture(gColor, texCoord);

	FragColor.r = gColor;
}