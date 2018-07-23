#version 330 core

uniform sampler2D uSourceTex;
in vec2 texCoord;
out vec4 FragColor;

void main() {
	vec4 basecolor = texture(uSourceTex, texCoord);
	vec3 color = basecolor.rgb;

	FragColor.rgb = color;
	FragColor.a = 0.5;
}