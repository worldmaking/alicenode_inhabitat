#version 330 core
uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix;
uniform float time;

layout (location = 0) in vec3 aPos;
layout (location = 2) in vec3 aLocation;

out vec3 worldpos, objectpos, eyepos;
out vec3 ray_direction, ray_origin;


void main()
{
    // basic vertex position:
    objectpos = aPos;
    // instance position:
    worldpos = aLocation;
    
    // final position of vertex in world space:
    vec3 vertexpos = worldpos + objectpos;

	// we want the raymarching to operate in object-local space, not world space
	// TODO: apply instance-local rotation/scale also? 
	ray_origin = objectpos;
    
    // calculate gl_Position in whatever way is appropriate first
	// e.g. gl_Position = uViewProjectionMatrix * vec4(worldPosition.xyz, 1.);
    gl_Position = uViewProjectionMatrix * vec4(vertexpos, 1.0); 
    
    // derive eye location in world space from current (model)view matrix:
    eyepos = -(uViewMatrix[3].xyz)*mat3(uViewMatrix);

	// ray direction computed from eye to vertex world position:
	ray_direction = vertexpos-eyepos; 
}  