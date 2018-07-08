#version 120
#extension GL_ARB_texture_rectangle : enable

varying vec3 texcoord0;
uniform sampler3D tex;

void main() {
	vec4 col = texture3D(tex, texcoord0);
	
	if (col.r <= 0.5) discard;
	
	gl_FragColor = col;
	
	//gl_FragColor = vec4(texcoord0, 1.);
}