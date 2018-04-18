#version 330 core
uniform mat4 uViewProjectionMatrix;
uniform float time;

in vec3 ray_direction, ray_origin;
in vec3 world_position;
in float world_scale;
in vec4 world_orientation;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;

#define PI 3.14159265359


vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.g, n.r, 0.5);
	return mix(n, vec3(1.), 0.75);
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

#define EPS 0.01
#define VERYFARAWAY  64.
#define MAX_STEPS 64
#define STEP_SIZE 1./float(MAX_STEPS)

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

// Cylinder standing upright on the xz plane
float fCylinder(vec3 p, float r, float height) {
	float d = length(p.xz) - r;
	d = max(d, abs(p.y) - height);
	return d;
}

float fSphere(vec3 p, float r) {
	return length(p) - r;
}

// Rotate around a coordinate axis (i.e. in a plane perpendicular to that axis) by angle <a>.
// Read like this: R(p.xz, a) rotates "x towards z".
// This is fast if <a> is a compile-time constant and slower (but still practical) if not.
void pR(inout vec2 p, float a) {
	p = cos(a)*p + sin(a)*vec2(p.y, -p.x);
}

// Shortcut for 45-degrees rotation
void pR45(inout vec2 p) {
	p = (p + vec2(p.y, -p.x))*sqrt(0.5);
}

// Repeat around the origin by a fixed angle.
// For easier use, num of repetitions is use to specify the angle.
float pModPolar(inout vec2 p, float repetitions) {
	float angle = 2*PI/repetitions;
	float a = atan(p.y, p.x) + angle/2.;
	float r = length(p);
	float c = floor(a/angle);
	a = mod(a,angle) - angle/2.;
	p = vec2(cos(a), sin(a))*r;
	// For an odd number of repetitions, fix cell index of the cell in -x direction
	// (cell index would be e.g. -5 and 5 in the two halves of the cell):
	if (abs(c) >= (repetitions/2)) c = abs(c);
	return c;
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

float fScene(vec3 p) {
	float osc = (0.3+abs(sin(time*7.)));
	float s = fSphere(p, world_scale*osc);
	float b = fBox(p, vec3(world_scale));
	
	// mouth
	float s0 = fSphere(p+vec3(0., 0., world_scale), world_scale*0.3*osc);
	float s1 = fSphere(p+vec3(0., 0., world_scale*0.25), world_scale*0.75);
	
	// eyes
	float se1 = fSphere(p+vec3( world_scale*0.5, world_scale*0.4, world_scale*0.6), world_scale*0.2);
	float se2 = fSphere(p+vec3(-world_scale*0.5, world_scale*0.4, world_scale*0.6), world_scale*0.2);
	
	float se = min(se1, se2);
	
	
	vec3 pc = p+vec3(0., 0., -world_scale*0.25);
	float a = pModPolar(pc.xz, 36.);
	pR(pc.yx, 0.+0.2*cos(time * 7. + abs(a*PI/3.)));
	
	
	float c1 = fCylinder(pc.zxy, world_scale*.02, world_scale*0.7);
	
	float z = max(s1, -s0); 
	z = min(se, z); //max(b,-z);
	z = min(c1, z);
	return z;
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

void main() {

	vec3 rd = normalize(ray_direction);
	vec3 ro = ray_origin; 
	
	float precis = EPS;
	float maxd = 4.;
	
	vec3 color = vec3(0.);
	float t = 0.0;
	float d = maxd;
	vec3 p = ro;
	float count = 0.;
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
		color = vec3(oblique);
		
		//color += (n*1.)*0.1;
		//color += mix(color, vec3(0.8)*max(0., dot(n, vec3(1.))), 0.5);
		
		float cheap_self_occlusion = 1.-pow(count, 0.75);
		
		float metallic = acute;
		color = mix(sky(n), sky(ref), metallic);
		
		color *= cheap_self_occlusion;
		
		// fog effect:
		vec3 fogcolor = sky(ray);
		float fogmix = length(world_position)/VERYFARAWAY;
		color = mix(color, fogcolor, fogmix);
		
		FragColor.rgb = color;
		FragNormal.xyz = n;
		
	} else if (t >= maxd) {
    	// shot through to background
    	discard;
    	
	} else {
		// too many ray steps
		FragColor = vec4(1.);
		discard;
	}
	
	// also write to depth buffer, so that landscape occludes other creatures:
	//gl_FragDepth = computeDepth(world_position + p, uViewProjectionMatrix);
	gl_FragDepth = computeDepth(world_position + quat_rotate(world_orientation, p), uViewProjectionMatrix);
	
	//FragColor.rgb = mod(rd * 8., 1.);
}