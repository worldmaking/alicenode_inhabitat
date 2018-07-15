#version 330 core

uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix, uLandMatrix, uLandMatrixInverse, uWorld2Map;
uniform float uMapScale;
uniform float uLandLoD;
uniform sampler2D uLandTex;
uniform sampler2D uFungusTex;
uniform sampler2D uNoiseTex;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;


out vec2 texCoord;
out vec3 normal, position;

void main() {
	texCoord = aTexCoord;
	normal = aNormal;

	vec4 noise = texture(uNoiseTex, texCoord);
	vec2 texCoord1 = texCoord + (noise.xy - 0.5) * 0.003;
	vec4 fields1 = texture(uFungusTex, texCoord1);
	float fungus = fields1.a;
	

	mat4 landMat = uWorld2Map * uLandMatrixInverse;

	// basic grid mesh is 0..1, need to scale that to the world
	position = (landMat * vec4(aPos, 1.)).xyz;

	// get the landscape data (normal + height) from the texture:
	vec4 land = texture(uLandTex, texCoord.xy);
	//vec4 land = textureLod(uLandTex, texCoord.xy, uLandLoD);

	// height (land.w) needs to be scaled to the world
	vec3 deform = (uLandMatrixInverse * vec4(0., land.w, 0., 1.)).xyz;
//position += deform * uMapScale;
//position.y += min(0., fungus);
	normal = land.xyz;
	
	gl_Position = uViewProjectionMatrix * vec4(position, 1.);
}