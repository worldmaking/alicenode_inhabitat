#version 330 core

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;

in vec2 texCoord;
in vec3 ray_direction, ray_origin, eye_position;

out vec4 FragColor;

float far_clip = 32.;

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.g, n.r, 0.5);
	return mix(n, vec3(1.), 0.75);
}

void main() {
	vec4 color = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;
	vec3 rd = normalize(ray_direction);

	// reflection vector 
	vec3 ref = reflect(rd, normal);
	float acute = abs(dot(normal, rd)); // how much surface faces us
	float oblique = 1.0 - acute; // how much surface is perpendicular to us

	color *= oblique;	

	float metallic = acute;
	color.rgb *= mix(sky(ref), sky(normal), metallic);
		
	
	// fog effect:
	vec3 fogcolor = sky(rd);
	float fogmix = clamp(length(position)/far_clip, 0., 1.);
	color.rgb = mix(color.rgb, fogcolor, fogmix);

	FragColor = color;	
}