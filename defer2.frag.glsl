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

float fBox(vec2 p, vec2 b) {
	vec2 d = abs(p) - b;
	vec2 inner = min(d, vec2(0));
	vec2 outer = max(d, vec2(0));
	return length(outer) + max(inner.x, inner.y);
}

void main() {
	vec2 inverseDim = 1./uDim;
	vec3 sides = vec3(inverseDim, 0.);
	vec4 basecolor = texture(gColor, texCoord);
	vec3 color = basecolor.rgb;
	
	float gamma = 0.9;
	//color = pow(color, vec3(gamma));

	
	// want to use a 2D distance function from boundary?
	
	float d = fBox(texCoord - vec2(0.5, 0.5), vec2(0.45));
	float edgefade = smoothstep(0.05, 0.0, d);

	color *= vec3(edgefade);
	FragColor.rgb = color; //mix(color, vec3(0.), uFade);
	FragColor.a = 0.01;
}