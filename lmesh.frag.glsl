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
layout (location = 3) out vec3 FragTexCoord;

void main() {
	// uncomment this line for contours:
	//if (mod(position.y*4.,1.) > 0.2) discard;

 	
	vec4 noise = texture(uNoiseTex, texCoord);
	vec2 texCoord1 = texCoord + (noise.xy - 0.5) * 0.003;
	vec4 fields = texture(uFungusTex, texCoord);
	vec4 fields1 = texture(uFungusTex, texCoord1);

	vec3 nnorm = normalize(normal + noise.xyz * 0.1);

	// steepness depends on normal:
	//float steepness = abs(1. - dot(normal, vec3(0, 1, 0)));
	float steepness = abs(1. - nnorm.y);

	vec3 chem = fields1.rgb;
	float fungus = fields1.a;


	vec3 color = normalize(vec3(texCoord , 0.5)) * 0.5;
	
	// TODO: color += rock colour

	if (fungus > 0.) {
		// fungus:
		float factor = fungus;
		//factor += noise.z * 0.1;
		color = mix(vec3(1) * min(1., position.y*0.05), color*fungus, factor);
		//color = vec3(0,1,0) * fungus;
		
	} else {

		// fungus ranges from -1 to 0 as it recovers toward life

		//color -= 0.8*(0.8-steepness);
		//color *= 0.25;
		// darken lowlands:
		
		//color = vec3(0);
		//color = vec3(0);//vec3(1,0,0) * -fungus;
		
	}

	// add chems:
	color += chem;

	// // darken steep slopes:
	color *= vec3(1. - steepness);

	// humanshadow:
	//float hu = texture2D(tex4, texcoord0).z;
	//color *= clamp((1.-4.*hu), 0., 1);

	color *= clamp((position.y - 5.)*0.1, 0., 1.);
  
  	//color = vec3(1.);

	vec2 div = ceil(texCoord * 10.);

	vec2 tc = mod(texCoord * 10., 1.);
	float tcr = length(tc - 0.5);
	vec2 tc1 = mod(texCoord * div, 1.);
	float tcr1 = length(tc1);

	if (tcr < 0.1) {

		color = vec3(texCoord, 1.);

	} else if (tcr1 < 0.2) {

		color = vec3(mod(div.x * 3., 0.8723), mod(div.y * 3., 0.7523), mod(div.x * 3., 0.34523));

	} else {
		//discard;
		color = vec3(0.25);
	}

	FragColor.rgb = color;
	//FragColor.rgb = nnorm;
	//FragColor.rgb = vec3(fungus);
	

	FragNormal.xyz = nnorm;
	FragPosition.xyz = position;
	FragTexCoord.xy = texCoord;
}