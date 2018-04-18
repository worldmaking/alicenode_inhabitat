#version 330 core
uniform mat4 uViewMatrixInverse, uViewProjectionMatrix;
uniform float time;

in vec4 world_orientation;
in vec3 world_position, eye_position;
in float world_scale;
out vec4 FragColor;

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.g, n.r, 0.5);
	return mix(n, vec3(1.), 0.75);
}

#define EPS 0.01
#define VERYFARAWAY  64.

#define PI 3.14159265359

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
	float r = world_scale;
	float s = length(p)-r;
	// r/2 is the biggest fBox we can fit in our bounding sphere
	float b = fBox(p, vec3(r * 0.5));
	return s;
}


// compute normal from a SDF gradient by sampling 4 tetrahedral points around a location `p`
// (cheaper than the usual technique of sampling 6 cardinal points)
// `fScene` should be the SDF evaluator `float distance = fScene(vec3 pos)`  
// `eps` is the distance to compare points around the location `p` 
// a smaller eps gives sharper edges, but it should be large enough to overcome sampling error
// in theory, the gradient magnitude of an SDF should everywhere = 1, 
// but in practice this isn’t always held, so need to normalize() the result
vec3 normal4(in vec3 p, float eps) {
	vec2 e = vec2(-eps, eps);
	// tetrahedral points
	float t1 = fScene(p + e.yxx), t2 = fScene(p + e.xxy), t3 = fScene(p + e.xyx), t4 = fScene(p + e.yyy); 
	vec3 n = (e.yxx*t1 + e.xxy*t2 + e.xyx*t3 + e.yyy*t4);
	// normalize for a consistent SDF:
	//return n / (4.*eps*eps);
	// otherwise:
	return normalize(n);
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

	if (false) {
		// do this option for complex geometries:
		
		// front face of a unit-radius sphere on this particle
		vec3 sphere = normalize(vec3(snorm, 1.));
		// rotated to face the camera just like the billboard itself
		// this is also thus the normal of a sphere centered at the particle
		vec3 spherenormal = mat3(uViewMatrixInverse) * sphere;
		// the billboard vertex, rotated & scaled to the world:
		vec3 billboard = world_scale * mat3(uViewMatrixInverse) * vec3(snorm, 0.);
		// this billboard vertex, located in world space:
		vec3 billboard_position = world_position + billboard;
		// use this to compute the ray direction from the eye:
		vec3 rd = normalize(billboard_position - eye_position);
		// the ray origin (relative to the particle location)
		// is computed by stepping back along the ray
		vec3 ro = billboard - rd * world_scale;
		
		float maxd = 4. * world_scale;
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
				//if (t <= maxd) count += STEP_SIZE * (d)/precis;
				break; // continue;
			}
			
			// advance ray
			t += d;
			p = ro+rd*t;
			count += STEP_SIZE;
		}
		FragColor.rgb = vec3(count);

		//FragColor.rgb = spherenormal*0.5+0.5;
		//FragColor.rgb = rd;

		//FragColor.rgb = 40.*abs(rdiff);
		
		//FragColor.rgb = ro*0.5+0.5;
		
		if (d < precis) {
			// normal in object space:
			vec3 no = normal4(p, .01);
			// normal in world space
			vec3 n = quat_rotate(world_orientation, no);
			// ray direction in world space
			vec3 ray = quat_rotate(world_orientation, rd);

			// reflection vector 
			vec3 ref = reflect(ray, n);
			
			float acute = abs(dot(n, ray)); // how much surface faces us
			float oblique = 1.0 - acute; // how much surface is perpendicular to us
			
			//color += (n*1.)*0.1;
			//color += mix(color, vec3(0.8)*max(0., dot(n, vec3(1.))), 0.5);
			
			
			float metallic = acute;
			vec3 color = mix(sky(n), sky(ref), metallic);
			
			color *= 0.5;

			// fog effect:
			vec3 fogcolor = sky(ray);
			float fogmix = length(world_position)/VERYFARAWAY;
			color = mix(color, fogcolor, fogmix);
			
			FragColor.rgb = color;
		} else {
			FragColor.rgb = vec3(0.);
			discard;
		}
	} else {
		// this is a much simpler algorithm, for spheres only

		// front face of a unit-radius sphere on this particle
		vec3 sphere = normalize(vec3(snorm, 1.));
		// rotated to face the camera just like the billboard itself
		// this is also thus the normal of a sphere centered at the particle
		vec3 spherenormal = mat3(uViewMatrixInverse) * sphere;
		
		vec3 p = sphere;
		vec3 n = spherenormal;
		vec3 ray = normalize(world_position + p - eye_position);

		// reflection vector 
		vec3 ref = reflect(ray, n);
		
		vec3 color;
		
		//color = sky(-n); 
		color = sky(ref);
		
		// fog effect:
		vec3 fogcolor = sky(ray);
		float fogmix = length(world_position)/VERYFARAWAY;
		color = mix(color, fogcolor, fogmix);

		FragColor.rgb = color; 
	}
	
	// place this fragment properly in the depth buffer
	// if you don't do this, the depth will be at the billboard location
	// but this is super-expensive; better to skip it if the particles are small enough
	//gl_FragDepth = computeDepth(world_position + p, uViewProjectionMatrix);
}