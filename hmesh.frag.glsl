#version 330 core

uniform sampler2D uFungusTex;
uniform sampler2D uNoiseTex;

/*
	missing textures:
	- noise
	- mountain base color
	- fungus, blood, pherofood/nest
	- humanshadow
*/

in vec2 texCoord;
in vec3 normal, position;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;

vec3 blood_color = 4. * vec3(158./255., 73./255., 43./255.);
vec3 food_color = 2.5*vec3(0.7, 0.3, 0.45);
vec3 nest_color = 2.5*vec3(0.15, 0.2, 0.15);

void main() {
	// uncomment this line for contours:
	//if (mod(position.y*4.,1.) > 0.2) discard;

 	
	vec4 noise = texture(uNoiseTex, texCoord);
	vec2 texCoord1 = texCoord + (noise.xy - 0.5) * 0.003;
	vec4 fields = texture(uFungusTex, texCoord);
	vec4 fields1 = texture(uFungusTex, texCoord1);

	vec3 nnorm = normalize(normal);

	// steepness depends on normal:
	//float steepness = abs(1. - dot(normal, vec3(0, 1, 0)));
	float steepness = abs(1. - nnorm.y);

	float blood = fields.r;
	float food = fields.g;
	float nest = fields.b;
	float fungus = fields1.a;


	vec3 color = normalize(vec3(texCoord, 0.5));
	
	// TODO: color += rock colour

	if (fungus > 0.) {
		// fungus:
		float factor = fungus * 0.7;
		//factor += noise.z * 0.1;
		//color = mix(vec3(1) * min(1., position.y*0.5), color, factor);
		color *= vec3(fungus);
		
	} else {
		//color -= 0.8*(0.8-steepness);
		//color *= 0.25;
		// darken lowlands:
		color *= clamp(position.y*0.1, 0., 1.);
		//color = vec3(0);
	}

	// // darken steep slopes:
	color *= vec3(1. - steepness);

	// // add phero emissions
	// //color += vec3(blood, food, nest);
	// // smells:
	if(true) {
		color += blood * blood_color;
		color += food * food_color;
		color += nest * nest_color;
	}

	// humanshadow:
	//float hu = texture2D(tex4, texcoord0).z;
	//color *= clamp((1.-4.*hu), 0., 1);

	FragColor.rgb = color;
	//FragColor.rgb = nnorm;
	//FragColor.rgb = vec3(fungus);
	

	FragNormal.xyz = nnorm;
	FragPosition.xyz = position;
}