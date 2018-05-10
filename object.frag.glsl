#version 330 core
uniform mat4 uViewProjectionMatrix;
uniform float time;

in vec3 ray_direction, ray_origin;
in vec3 world_position;
in float world_scale;
in vec4 world_orientation;
in float phase;
in vec3 velocity;

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

#define EPS 0.001
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
vec2 pRot(in vec2 p, float a) {
	p = cos(a)*p + sin(a)*vec2(p.y, -p.x);
	return p;
}

vec3 pRotYZ(in vec3 p, float a) {
	p.yz = cos(a)*p.yz + sin(a)*vec2(p.z, -p.y);
	return p;
}

vec3 pTranslate(vec3 p, vec3 t) {
	return p + t;
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

vec3 closest_point_on_line_segment(vec3 P, vec3 A, vec3 B) {
	vec3 AB = B-A;
	float l2 = dot(AB, AB);	// length squared
	
	if (l2 < EPS) {
		// line is too short, just use an endpoint
		return A;
	}
	
	// Consider the line extending the segment,
	// parameterized as A + t (AB).
	// We find projection of point p onto the line.
	// It falls where t = [(AP) . (AB)] / |AB|^2
	
	vec3 AP = P-A;
	float t = dot(AP, AB) / l2;
	
	if (t < 0.0) {
		return A; 	// off A end
	} else if (t > 1.0) {
		return B; 	// off B end
	} else {
		return A + t * AB; // on segment
	}
}

vec4 closest_point_on_line_segment_with_t(vec3 P, vec3 A, vec3 B) {
	vec3 AB = B-A;
	float l2 = dot(AB, AB);	// length squared
	
	if (l2 < EPS) {
		// line is too short, just use an endpoint
		return vec4(A, 0.);
	}
	
	// Consider the line extending the segment,
	// parameterized as A + t (AB).
	// We find projection of point p onto the line.
	// It falls where t = [(AP) . (AB)] / |AB|^2
	
	vec3 AP = P-A;
	float t = dot(AP, AB) / l2;
	
	if (t < 0.0) {
		return vec4(A, 0.); 	// off A end
	} else if (t > 1.0) {
		return vec4(B, 1.); 	// off B end
	} else {
		return vec4(A + t * AB, t); // on segment
	}
}

vec4 closest_point_on_line_segment_with_t_leng(vec3 P, float l) {
	//vec3 AB = B-A;
	float l2 = l * l;	// length squared
	
	if (l2 < EPS) {
		// line is too short, just use an endpoint
		return vec4(0.);
	}
	
	// Consider the line extending the segment,
	// parameterized as A + t (AB).
	// We find projection of point p onto the line.
	// It falls where t = [(AP) . (AB)] / |AB|^2
	
	//vec3 AP = P-A;
	float t = P.z / l;
	
	if (t < 0.0) {
		return vec4(0.); 	// off A end
	} else if (t > 1.0) {
		return vec4(0., 0. , l, 1.); 	// off B end
	} else {
		return vec4(0., 0., l * t, t); // on segment
	}
}

// i.e. distance to line segment, with smoothness r
float sdCapsule1(vec3 p, vec3 a, vec3 b, float r) {
	vec3 p1 = closest_point_on_line_segment(p, a, b);
	return distance(p, p1) - r;
}

vec3 sdCapsule1_tex(vec3 p, vec3 a, vec3 b, float r) {
	vec4 p1 = closest_point_on_line_segment_with_t(p, a, b);
	return vec3(0., p1.w, distance(p, p1.xyz) - r);
}

vec3 sdCapsule1_tex_z(vec3 p, float l, float r) {
	//vec4 p1 = closest_point_on_line_segment_with_t_leng(p, l);

	vec4 p1 = vec4(0.);
	vec2 uv;
	//vec3 AB = B-A;
	float l2 = l * l;	// length squared
	if (l2 > EPS) {
		// Consider the line extending the segment,
		// parameterized as A + t (AB).
		// We find projection of point p onto the line.
		// It falls where t = [(AP) . (AB)] / |AB|^2
		//vec3 AP = P-A;
		float t = p.z / l;
		if (t > 1.0) {
			p1 = vec4(0., 0. , l, 1.); 	// off B end
		} else if (t > 0.) {
			p1 = vec4(0., 0., l * t, t); // on segment
		}
		uv.x = p1.w;
		uv.y = atan(p.y, p.x);
	}
	// other texcoord is a function of p.xy's angle, mapped 0..1

	return vec3(uv, distance(p, p1.xyz) - r);
}

/*p = position of ray
* a and b = endpoints of the line (capsule)
* ra = radius of a
* rb = radius of b
*/
float sdCapsule2(vec3 p, vec3 a, vec3 b, float ra, float rb) {
	float timephase = time+phase;
	vec3 pa = p - a, ba = b - a;
	float t = dot(pa,ba)/dot(ba,ba);	// phase on line from a to b
	float h = clamp( t, 0.0, 1.0 );
	
	// add some ripple:
	float h1 = h + 0.2*sin(PI * 4. * (t*t + timephase* 0.3));
	
	// basic distance:
	vec3 rel = pa - ba*h;
	float d = length(rel);
	
	d = d - mix(ra, rb, h1);
	
	return d;
}

vec3 sdCapsule2_tex(vec3 p, vec3 a, vec3 b, float ra, float rb) {
	float timephase = time+phase;
	vec3 pa = p - a, ba = b - a;
	float t = dot(pa,ba)/dot(ba,ba);	// phase on line from a to b
	float h = clamp( t, 0.0, 1.0 );
	
	// add some ripple:
	float h1 = h + 0.2*sin(PI * 4. * (t*t + timephase* 0.3));
	
	// basic distance:
	vec3 rel = pa - ba*h;
	float d = length(rel);
	
	d = d - mix(ra, rb, h1);
	
	return vec3(1., 0., d);
}

vec3 sdCapsule2_tex_z(vec3 p, float l, float ra, float rb) {
	float timephase = time+phase;

	vec4 p1 = vec4(0.);
	vec2 uv;
	float d;
	//vec3 AB = B-A;
	float l2 = l * l;	// length squared
	if (l2 > EPS) {

		float t = p.z / l;
		if (t > 1.0) {
			p1 = vec4(0., 0. , l, 1.); 	// off B end
		} else if (t > 0.) {
			p1 = vec4(0., 0., l * t, t); // on segment
		}
		uv.x = p1.w;
		uv.y = atan(p.y, p.x);

		float h = clamp( t, 0.0, 1.0 );
	
		// add some ripple:
		float h1 = h + 0.2*sin(PI * 4. * (t*t + timephase* 0.3));
	
		// basic distance:
		vec3 rel = p - vec3(0., 0., l*h);
		d = length(rel);
		
		d = d - mix(ra, rb, h1);
	}
	// other texcoord is a function of p.xy's angle, mapped 0..1

	//return vec3(uv, distance(p, p1.xyz) - r);
	
	return vec3(uv, d);
}

//Rotate function by:
// http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
mat4 rotateY(float theta) {
    float c = cos(theta);
    float s = sin(theta);

    return mat4(
        vec4(c, 0, s, 0),
        vec4(0, 1, 0, 0),
        vec4(-s, 0, c, 0),
        vec4(0, 0, 0, 1)
    );
}

/**
 * Signed distance function for a cube centered at the origin
 * Credit:
 * http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
 */
float sdCube(in vec3 p){
	vec3 d = abs(p) - vec3(0.2, 0.2, 0.2);

	// Assuming p is inside the cube, how far is it from the surface?
    // Result will be negative or zero.
	float inDist = min(max(d.x, max(d.y, d.z)), 0.0);

	// Assuming p is outside the cube, how far is it from the surface?
    // Result will be positive or zero.
	float outDist = length(max(d, 0.0));

	return inDist + outDist;

}

// iq has this version, which seems a lot simpler?
float sdEllipsoid1( in vec3 p, in vec3 r ) {
	return (length( p/r ) - 1.0) * min(min(r.x,r.y),r.z);
}



// polynomial smooth min (k = 0.1);
float smin( float a, float b, float k ) {
	float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );
	return mix( b, a, h ) - k*h*(1.0-h);
}

vec3 smin_tex( vec3 a, vec3 b, float k ) {
	float h = clamp( 0.5+0.5*(b.z-a.z)/k, 0.0, 1.0 );
	return mix( b, a, h ) - k*h*(1.0-h);
}

float smax( float a, float b, float k )
{
	float k1 = k*k;
	float k2 = 1./k1;
	return log( exp(k2*a) + exp(k2*b) )*k1;
}

float ssub(in float A, in float B, float k) {
	return smax(A, -B, k);
}

// NOTE scale := f(p/s)*s

float fScene(vec3 p) {
	float timephase = time + phase;
	float scl = world_scale;

	p /= scl;

	//p = p.yxz;
	// basic symmetry:
	p.y = abs(p.y);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*timephase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;

	//z = -1.0 of the second parameter of the sdCapsules is the front of the object.
	float a = sdCapsule1(p, vec3(0., 0., -0.25), vec3(0., y, z), w*w);
	float b = sdCapsule2(p, vec3(0., -0., -0.25), vec3(z, w, y), 0.125, 0.1);
	float c = sdCapsule2(p, vec3(0., -0., -0.0), vec3(z, w, y), 0.125, 0.1);
	float cube = sdCube((rotateY(-timephase) * vec4(p, 1.0)).xyz);
	//float a = 0.7;
	//float b = 0.7;
	float d = smin(a, b, 0.5);
	float e = smin(d, c, 0.1);
	float f = smin(e, cube, 0.1);

	
	//float mouth = sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05));
	float mouth = sdEllipsoid1((rotateY(-timephase) * vec4(p.yzx, 1.0)).xyz, vec3(0.25, 0.5, 0.05)); //Rotating the mouth Ellipsoid

	//return d * world_scale;
	return scl * ssub(f, mouth, 0.2);
}

vec3 fScene_tex(vec3 p) {
	float timephase = time + phase;
	float scl = world_scale;

	p /= scl;

	//p = p.yxz;
	// basic symmetry:
	p.y = abs(p.y);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*timephase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;

	vec3 a = sdCapsule1_tex(p, vec3(0., 0., -0.25), vec3(0., y, z), w*w);
	vec3 b = sdCapsule2_tex(p, vec3(0., -0., -0.25), vec3(z, w, y), 0.125, 0.1);
	//float a = 0.7;
	//float b = 0.7;
	vec3 d = a ;//smin_tex(a, b, 0.5);

	//float mouth = sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05));

	//return d * world_scale;
	//return scl * ssub(d, mouth, 0.125);
	return vec3(d.xy, d.z * scl);
}

vec3 fScene_tex_z(vec3 p) {
	float timephase = time + phase;
	float scl = world_scale;

	p /= scl;

	//p = p.yxz;
	// basic symmetry:
	p.y = abs(p.y);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*timephase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;

	//sdCapsule1_tex(p, vec3(0., 0., -0.25), vec3(0., y, z), w*w);
	vec3 a = sdCapsule1_tex_z(pRotYZ(pTranslate(p, vec3(0, 0, -0.25)), PI / -6.), 0.5, w*w);
	vec3 b = sdCapsule2_tex_z(pRotYZ(pTranslate(p, vec3(0, 0, -0.25)), PI / -2.), 0.25, 0.125, 0.1);
	//float a = 0.7;
	//float b = 0.7;
	vec3 d = smin_tex(a, b, 0.5);


	//float mouth = sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05));

	//return d * world_scale;
	//return scl * ssub(d, mouth, 0.125);
	return vec3(d.xy, d.z * scl);
}
 
float fScene_old(vec3 p) {
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
	vec3 d_tex;
	for( int i=0; i<MAX_STEPS; i++ ) {
	
		d_tex = fScene_tex_z(p);
        d = d_tex.z;
        
        if (d < precis || t > maxd ) {
        	if (t <= maxd) count += STEP_SIZE * (d)/precis;
        	break; // continue;
        }
        
        // advance ray
        t += d;
        p = ro+rd*t;
        count += STEP_SIZE;
    }
    FragColor = vec4(1.);
    
    if (d < precis) {
		float cheap_self_occlusion = 1.-count; //pow(count, 0.75);
		FragColor.rgb = vec3(d_tex.xy, 0.); //vec3(cheap_self_occlusion);
		FragNormal.xyz = quat_rotate(world_orientation, normal4(p, .01));
		
	} else if (t >= maxd) {
    	// shot through to background
    	discard;
    	
	} else {
		// too many ray steps
		
		//FragNormal.xyz = rd;
		//FragNormal.xyz = quat_rotate(world_orientation, normal4(p, .01));
		discard;
	}
	
	// also write to depth buffer, so that landscape occludes other creatures:
	FragPosition.xyz = world_position + quat_rotate(world_orientation, p);
	gl_FragDepth = computeDepth(FragPosition.xyz, uViewProjectionMatrix);
}