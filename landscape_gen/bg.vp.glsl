#version 120
#extension GL_ARB_texture_rectangle : enable

uniform float layer;
varying vec3 texcoord0;

void main() {	
	// vertex
	vec3 V = (gl_Vertex).xyz;
	
	vec3 Vs = V;
	Vs.xy *= 1.5;
	
	gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * vec4(Vs, 1.);	
	
	texcoord0 = V*0.5+0.5; 
	texcoord0.z = layer;
}