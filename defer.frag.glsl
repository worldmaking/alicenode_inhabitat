#version 330 core

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;

in vec2 texCoord;
out vec4 FragColor;


vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.g, n.r, 0.5);
	return mix(n, vec3(1.), 0.75);
}

void main() {
	vec4 color = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;
	//FragColor = color;
	FragColor.rgb = normal.xyz;
	//FragColor.rgb = position.xyz;


	// fog effect:
	//vec3 fogcolor = sky(ray);
	//float fogmix = length(world_position)/VERYFARAWAY;
	//color = mix(color, fogcolor, fogmix);

	color = vec4(gl_DepthRange.far * 0.5);

	FragColor.rgb = color.rgb;

	
}