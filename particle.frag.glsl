#version 330 core
uniform mat4 uViewMatrixInverse, uViewProjectionMatrix;
uniform float time;


in vec3 point_position, eye_position;
in float world_scale;
out vec4 FragColor;

#define PI 3.14159265359

// Maximum/minumum elements of a vector
float vmax(vec2 v) {
	return max(v.x, v.y);
}

float vmax(vec3 v) {
	return max(max(v.x, v.y), v.z);
}

float vmax(vec4 v) {
	return max(max(v.x, v.y), max(v.z, v.w));
}

float vmin(vec2 v) {
	return min(v.x, v.y);
}

float vmin(vec3 v) {
	return min(min(v.x, v.y), v.z);
}

float vmin(vec4 v) {
	return min(min(v.x, v.y), min(v.z, v.w));
}

// Box: correct distance to corners
float fBox(vec3 p, vec3 b) {
	vec3 d = abs(p) - b;
	return length(max(d, vec3(0))) + vmax(min(d, vec3(0)));
}

float fScene(vec3 p) {
	float s = length(p)-(world_scale);
	float b = fBox(p, vec3(0.5*world_scale));
	return min(s,b);
}

// p is the vec3 position of the surface at the fragment.
// viewProjectionMatrix would be typically passed in as a uniform
// assign result to gl_FragDepth:
float computeDepth(vec3 p, mat4 viewProjectionMatrix) {
	float dfar = gl_DepthRange.far;
	float dnear = gl_DepthRange.near;
	vec4 clip_space_pos = viewProjectionMatrix * vec4(p, 1.);
	float ndc_depth = clip_space_pos.z / clip_space_pos.w;	
	// standard perspective:
	return (((dfar-dnear) * ndc_depth) + dnear + dfar) / 2.0;
}

void main() {
	// signed-normalized coordinate over the billboard:
	vec2 snorm = vec2(2.*gl_PointCoord.x-1.,1.-2.*gl_PointCoord.y);

	// quick circular mask. 
	// this might not be accurate when using very wide FOV
	if (length(snorm) > 1.) discard; 

	// point_position is uniform over the fragements; we need to displace this according to the gl_PointCoord
	// but this is screen aligned; also need to unrotate to get world coordinate of the sprite
	
	// front face of a unit-radius sphere on this particle
	vec3 sphere = normalize(vec3(snorm, 1.));
	// rotated to face the camera just like the billboard itself
	// this is also thus the normal of a sphere centered at the particle
	vec3 spherenormal = mat3(uViewMatrixInverse) * sphere;
	// the billboard vertex, rotated & scaled to the world:
	vec3 billboard = world_scale * mat3(uViewMatrixInverse) * vec3(snorm, 0.);
	// this billboard vertex, located in world space:
	vec3 billboard_position = point_position + billboard;
	// use this to compute the ray direction from the eye:
	vec3 rd = normalize(billboard_position - eye_position);
	// the ray origin (relative to the particle location)
	// is computed by stepping back along the ray
	vec3 ro = billboard - rd * world_scale*sphere.z;
	
	float maxd = 2. * world_scale;
	float d = maxd;
	vec3 p = ro;
	float precis = 0.001;
	float count = 0.;
	float t = 0.;
	#define MAX_STEPS 8
	float STEP_SIZE = 1./float(MAX_STEPS);
	for( int i=0; i<MAX_STEPS; i++ ) {
	
        d = fScene(p);
        
        if (d < precis || t > maxd ) {
        	if (t <= maxd) count += STEP_SIZE * (d)/precis;
        	break; // continue;
        }
        
        // advance ray
        t += d;
        p = ro+rd*t;
        count += STEP_SIZE;
    }
	FragColor.rgb = vec3(1.-count);

	FragColor.rgb = spherenormal*0.5+0.5;
	//FragColor.rgb = rd;

	//FragColor.rgb = 40.*abs(rdiff);
	
	//FragColor.rgb = ro*0.5+0.5;

	FragColor.rgb = vec3(sphere.z);
	
	if (d < precis) {
	//	FragColor.rgb = vec3(1.);
	//} else if (t > maxd) {
	//	FragColor.rgb = vec3(0.);
	//	discard;
	} else {
		FragColor.rgb = vec3(0.);
		//discard;
	}
	
	// place this fragment properly in the depth buffer
	// if you don't do this, the depth will be at the billboard location
	// but this is super-expensive; better to skip it if the particles are small enough
	//gl_FragDepth = computeDepth(point_position + p, uViewProjectionMatrix);
}