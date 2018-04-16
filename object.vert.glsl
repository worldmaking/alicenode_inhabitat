#version 330 core
uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix;
uniform float time;

// vertex in object space:
layout (location = 0) in vec3 aPos;
// object in world space:
layout (location = 2) in vec3 iLocation;
layout (location = 3) in vec4 iOrientation;

// object pose & scale, needs careful handling in SDF calculation
out vec3 worldpos;
out float size;
out vec4 world_orientation;

out vec3 objectpos, eyepos;
out vec3 ray_direction, ray_origin;

vec4 quat_fromeuler(float az, float el, float ba) {
	float c1 = cos(az * 0.5);
	float c2 = cos(el * 0.5);
	float c3 = cos(ba * 0.5);
	float s1 = sin(az * 0.5);
	float s2 = sin(el * 0.5);
	float s3 = sin(ba * 0.5);
	// equiv Q1 = Qy * Qx; -- since many terms are zero
	float tw = c1*c2;
	float tx = c1*s2;
	float ty = s1*c2;
	float tz =-s1*s2;
	// equiv Q2 = Q1 * Qz; -- since many terms are zero
	return vec4(
		tx*c3 + ty*s3,
		ty*c3 - tx*s3,
		tw*s3 + tz*c3,
		tw*c3 - tz*s3
	);
}

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

void main()
{
    // size of object
	size = 1.;
    
    // basic vertex position:
    vec3 objectpos = aPos * size;
    
    // instance position:
    worldpos = iLocation;
  	world_orientation = iOrientation;
    
    // final position of vertex in world space:
    //vec3 vertexpos = worldpos + objectpos;
    vec3 vertexpos = worldpos + quat_rotate(iOrientation, objectpos);

	// we want the raymarching to operate in object-local space, not world space
	// TODO: apply instance-local rotation/scale also? 
	ray_origin = objectpos;
    
    // calculate gl_Position in whatever way is appropriate first
	// e.g. gl_Position = uViewProjectionMatrix * vec4(worldPosition.xyz, 1.);
    gl_Position = uViewProjectionMatrix * vec4(vertexpos, 1.0); 
    
    // derive eye location in world space from current (model)view matrix:
    eyepos = -(uViewMatrix[3].xyz)*mat3(uViewMatrix);

	// ray direction computed from eye to vertex world position:
	//ray_direction = (vertexpos-eyepos);
	ray_direction = quat_unrotate(iOrientation, vertexpos-eyepos); 
}  