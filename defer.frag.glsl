#version 330 core

/*
possible buffers:

base color RGB, alpha (opacity)
self-AO
material/shading model
material properties: metallic, roughness

other inputs:
depth (from position)

possible fx:
bloom, colour correction, antialiasing
*/

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;
uniform float uFarClip;

in vec2 texCoord;
in vec3 ray_direction, ray_origin, eye_position;

out vec4 FragColor;

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.g, n.r, 0.5);
	return mix(n, vec3(1.), 0.75);
}

void main() {
	vec4 basecolor = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;
	vec3 relative_position = position - eye_position;
	float depth = length(relative_position);
	float normalized_depth = depth/uFarClip;
	vec3 rd = normalize(ray_direction);

	vec3 color = sky(rd);

	// reflection vector 
	vec3 ref = reflect(rd, normal);
	float acute = abs(dot(normal, rd)); // how much surface faces us
	float oblique = 1.0 - acute; // how much surface is perpendicular to us

	//color *= 1. - 0.5*oblique;	

	//float metallic = acute;
	float metallic = oblique;
	//color.rgb *= mix(sky(ref), sky(normal), metallic);
		
	// fog effect:
	vec3 fogcolor = sky(rd);
	float fogmix = clamp(normalized_depth, 0., 1.);
	color.rgb = mix(color.rgb, fogcolor, fogmix);

	// pos viz:
	//color.rgb = position.xyz;

	// normal viz:
	//color.rgb = normal*0.5+0.5;

	// depth viz:
	//color.rgb = vec3(normalized_depth);


	

	FragColor.rgb = color;	

}