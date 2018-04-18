#version 330 core
uniform mat4 uProjectionMatrix, uViewMatrix;
uniform float uViewPortHeight, uPointSize, time;

// object in world space:
layout (location = 0) in vec3 vertex_position;

out vec3 point_position, eye_position;
out float world_scale;
//out vec3

void main() {

	world_scale = uPointSize;

	// vertex in camera space:
	vec4 view_position = uViewMatrix * vec4(vertex_position, 1.);

	// vertex in screen space:
	gl_Position = uProjectionMatrix * view_position;
	//centre = (0.5 * gl_Position.xy/gl_Position.w + 0.5) * uViewPortHeight;
	gl_PointSize = world_scale * uViewPortHeight * uProjectionMatrix[1][1] / gl_Position.w;
	//radiusPixels = gl_PointSize / 2.0;



	// derive eye location in world space from current view matrix:
	// (could pass this in as a uniform instead...)
	eye_position = -(uViewMatrix[3].xyz)*mat3(uViewMatrix);

	// we want the raymarching to operate in object-local space:
	point_position = vertex_position;
	//ray_origin = scaledpos;
}  