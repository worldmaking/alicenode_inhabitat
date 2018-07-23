#version 330 core

/*
possible buffers:

base color RGB, alpha (opacity)
self-AO
material/shading model
material properties: metallic, roughness
subsurface color

other inputs:
depth (from position)

possible fx:
bloom, colour correction, antialiasing
*/

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;
uniform sampler2D gTexCoord;
uniform sampler3D uDistanceTex;
//uniform sampler3D uLandTex;
uniform sampler3D uEmissionTex;
uniform sampler3D uFluidTex;

uniform float uFarClip;
uniform vec2 uDim;
uniform mat4 uViewMatrix;
uniform mat4 uFluidMatrix;
uniform float time;
uniform float uFade;

in vec2 texCoord;
in vec3 ray_direction, ray_origin, eye_position;

out vec4 FragColor;

#define PI 3.14159265359

vec3 sky(vec3 dir) {
	vec3 n0 = dir*0.5+0.5;
	vec3 n = n0;
	float a = time * 0.3;
	// detail
	n.r = sin(2.*PI* n.r*n.g + a)*0.5+0.5;
	n.g = cos(2.*PI* n.r*n.g - a)*0.5+0.5;
	// simplify
	n.g = mix(n.g, n.r, 0.5);
	// lighten
	n = mix(n, vec3(0.), 0.5);

	// below y=0 should be black
	

	return n * (n0.y - 0.2);
}

float fScene(vec3 p) {
	vec3 tc = (uFluidMatrix * vec4(p, 1.)).xyz;
    return texture(uDistanceTex, tc).x;
}

vec3 normal4(in vec3 p, float eps) {
	vec2 e = vec2(-eps, eps);
	// tetrahedral points
	float t1 = fScene(p + e.yxx), t2 = fScene(p + e.xxy), t3 = fScene(p + e.xyx), t4 = fScene(p + e.yyy); 
	vec3 n = (e.yxx*t1 + e.xxy*t2 + e.xyx*t3 + e.yyy*t4);
	// normalize for a consistent SDF:
	//return n / (4.*eps*eps);
	// otherwise:
	return normalize(n);
}

void main() {
	vec2 inverseDim = 1./uDim;
	vec3 sides = vec3(inverseDim, 0.);
	vec4 basecolor = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;
	vec3 objtexcoord = texture(gTexCoord, texCoord).xyz;

	// TODO: fluid scale / transform?
	vec3 fluidtexcoord = (uFluidMatrix * vec4(position, 1.)).xyz;
	//vec3 fluidtexcoord = position; //
	vec3 fluid = texture(uFluidTex, fluidtexcoord).xyz;
	vec3 density = texture(uEmissionTex, fluidtexcoord).xyz;
	//float land = texture(uLandTex, fluidtexcoord).x;

	float dist = texture(uDistanceTex, fluidtexcoord).x;
	

	vec3 view_position = (uViewMatrix * vec4(position, 1.)).xyz;
	float depth = length(view_position); 
	float normalized_depth = depth/uFarClip;
	vec3 rd = normalize(ray_direction);
	vec3 ro = (uFluidMatrix * vec4(ray_origin, 1.)).xyz;
	vec3 ro1 = fluidtexcoord;


	vec3 color;

	// compare with next point:
	vec2 texCoordl = texCoord - sides.xz;
	vec2 texCoordr = texCoord + sides.xz;
	vec2 texCoordu = texCoord - sides.zy;
	vec2 texCoordd = texCoord + sides.zy;

	color = basecolor.rgb;
	
	float gamma = 0.9;
	//color = pow(color, vec3(gamma));

	FragColor.rgb = color; //mix(color, vec3(0.), uFade);
}