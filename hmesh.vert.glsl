#version 330 core

uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix, uLandMatrix, uLandMatrixInverse, uWorld2Map;
uniform float uMapScale;
uniform float uLandLoD;
uniform sampler2D uLandTex;
uniform sampler2D uHumanTex;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 normal, position;

#define LAND_DIM 200

void main() {
	texCoord = aTexCoord;

	//texCoord += 0.5 / float(LAND_DIM);

	normal = aNormal;

	mat4 landMat = uWorld2Map * uLandMatrixInverse;

	// basic grid mesh is 0..1, need to scale that to the world
	position = (landMat * vec4(aPos, 1.)).xyz;


	// get the landscape data (normal + height) from the texture:
	float human = texture(uHumanTex, texCoord.xy).r;
	float land = texture(uLandTex, texCoord.xy).w;
	//vec4 land = textureLod(uLandTex, texCoord.xy, uLandLoD);

	// height (land.w) needs to be scaled to the world
	vec3 deform = (uLandMatrixInverse * vec4(0., human, 0., 1.)).xyz;
	// push human below land:
	

	position += deform * uMapScale;
	normal = vec3(0,1,0);

	if ((human - land) < 0.02) position.y *= 0.5;
	
	gl_Position = uViewProjectionMatrix * vec4(position, 1.);
}