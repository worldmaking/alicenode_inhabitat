#version 330 core
uniform mat4 uViewProjectionMatrix, uViewMatrix;
uniform vec3 uEyePos;
uniform float uMini2World;

// vertex in object space:
layout (location = 0) in vec3 aPos;
// object in world space:
layout (location = 2) in vec3 iLocation;
layout (location = 3) in vec4 iOrientation;
layout (location = 4) in float iScale;
layout (location = 5) in float iPhase;
layout (location = 6) in vec3 iVelocity;

// object pose & scale, needs careful handling in SDF calculation
out vec3 world_position;
out float world_scale;
out vec4 world_orientation;
out float phase;
out vec3 vertexpos;
// starting ray for this vertex, in object space.
out vec3 ray_direction, ray_origin;
out vec3 velocity;

//	q must be a normalized quaternion
vec3 quat_rotate(vec4 q, vec3 v) {
	vec4 p = vec4(
		q.w*v.x + q.y*v.z - q.z*v.y,	// x
		q.w*v.y + q.z*v.x - q.x*v.z,	// y
		q.w*v.z + q.x*v.y - q.y*v.x,	// z
		-q.x*v.x - q.y*v.y - q.z*v.z	// w
	);
	return vec3(
		p.x*q.w - p.w*q.x + p.z*q.y - p.y*q.z,	// x
		p.y*q.w - p.w*q.y + p.x*q.z - p.z*q.x,	// y
		p.z*q.w - p.w*q.z + p.y*q.x - p.x*q.y	// z
	);
}

// equiv. quat_rotate(quat_conj(q), v):
// q must be a normalized quaternion
vec3 quat_unrotate(in vec4 q, in vec3 v) {
	// return quat_mul(quat_mul(quat_conj(q), vec4(v, 0)), q).xyz;
	// reduced:
	vec4 p = vec4(
				  q.w*v.x - q.y*v.z + q.z*v.y,  // x
				  q.w*v.y - q.z*v.x + q.x*v.z,  // y
				  q.w*v.z - q.x*v.y + q.y*v.x,  // z
				  q.x*v.x + q.y*v.y + q.z*v.z   // w
				  );
	return vec3(
				p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y,  // x
				p.w*q.y + p.y*q.w + p.z*q.x - p.x*q.z,  // y
				p.w*q.z + p.z*q.w + p.x*q.y - p.y*q.x   // z
				);
}

void main() {

	// need to pass instance pose info to frag shader
	world_position = iLocation;
	world_scale = iScale;
	world_orientation = iOrientation;
	phase = iPhase;
	velocity = iVelocity+0.5;

	// converting vertex into world space:
	vec3 scaledpos = aPos * world_scale;

	vertexpos = (world_position + quat_rotate(world_orientation, scaledpos)) ;

	// calculate gl_Position the usual way
	gl_Position = uViewProjectionMatrix * vec4(vertexpos, 1.); 
	// derive eye location in world space from current view matrix:
	// (could pass this in as a uniform instead...)

	vec3 eyepos = uEyePos / uMini2World;
	//vec3 eyepos = -(uViewMatrix[3].xyz)*mat3(uViewMatrix);


	// we want the raymarching to operate in object-local space:
	ray_origin = scaledpos;
	ray_direction = quat_unrotate(world_orientation, vertexpos-eyepos); 
}  