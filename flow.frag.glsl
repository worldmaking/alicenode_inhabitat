#version 330 core
out vec4 FragColor;
in vec2 texCoord;
uniform sampler2D tex;

void main() {
	//FragColor = vec4(texCoord, 0.5, 1.);
	vec2 flow = texture(tex, texCoord).xy;

	flow = 0.5 + flow * 0.01;

	FragColor.rg = flow;
}