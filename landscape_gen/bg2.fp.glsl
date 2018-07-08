#version 120
#extension GL_ARB_texture_rectangle : enable

varying vec3 texcoord0;
uniform sampler2D tex;

void main() {
	vec4 col = texture2D(tex, texcoord0.xy);
	
	if (col.r <= texcoord0.z) discard;
	
	gl_FragColor = col;
	
	//gl_FragColor = vec4(texcoord0, 1.);
}