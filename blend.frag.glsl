#version 330 core

uniform sampler2D uSourceTex;
in vec2 texCoord;
out vec4 FragColor;

void main() {
	vec4 basecolor = texture(uSourceTex, texCoord);
	
	vec3 color = basecolor.rgb;

	float gamma = 0.5;
	color = pow(color, vec3(gamma));

	FragColor = vec4(color, 0.1);

	//FragColor.a = 1.;
}