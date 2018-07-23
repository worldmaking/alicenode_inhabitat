#version 330 core

uniform sampler2D uSourceTex;
in vec2 texCoord;
out vec4 FragColor;

void main() {
	vec4 basecolor = texture(uSourceTex, texCoord);
	
	FragColor.rgb = basecolor.rgb;
	FragColor.a = 0.01;
	//FragColor.a = 0.01;
	//FragColor.rg = texCoord;
}