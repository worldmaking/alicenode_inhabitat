#version 330 core
uniform mat4 uViewProjectionMatrix;

in vec3 world_position;
in float world_scale;
in vec4 world_orientation;
in vec3 ray_direction, ray_origin, eyepos;
in vec3 vertexpos;
in float phase;
in vec3 basecolor;
in vec4 params;
in vec3 flow;
flat in int id;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;
layout (location = 3) out vec3 FragTexCoord;

#define EPS 0.001
#define VERYFARAWAY  64.
#define MAX_STEPS 64
#define STEP_SIZE 1./float(MAX_STEPS)

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

/////////////////////////////////////////////////////////////////////////////

#define PI 3.14159265359
#define TWOPI 6.283185307

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

float quant(float v, float s) {
	return floor(v/s)*s;
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

// Cylinder standing upright on the xz plane
float fCylinder(vec3 p, float r, float height) {
	float d = length(p.xz) - r;
	d = max(d, abs(p.y) - height);
	return d;
}

float fSphere(vec3 p, float r) {
	return length(p) - r;
}

// Capsule: A Cylinder with round caps on both sides
float fCapsule(vec3 p, float r, float c) {
	return mix(length(p.xz) - r, length(vec3(p.x, abs(p.y) - c, p.z)) - r, step(c, abs(p.y)));
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

vec3 pRotXZ(in vec3 p, float a) {
	p.xz = cos(a)*p.xz + sin(a)*vec2(p.z, -p.x);
	return p;
}

vec3 pRotXY(in vec3 p, float a) {
	p.xy = cos(a)*p.xy + sin(a)*vec2(p.y, -p.x);
	return p;
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

// Shortcut for 45-degrees rotation
void pR45(inout vec2 p) {
	p = (p + vec2(p.y, -p.x))*sqrt(0.5);
}

vec3 pTranslate(vec3 p, vec3 t) {
	return p + t;
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

float pModPolarShift(inout vec2 p, float repetitions, float shift) {
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

//https://www.shadertoy.com/view/Xds3zN
vec3 opRep( vec3 p, vec3 c )
{
    return mod(p,c)-0.5*c;
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

/*p = position of ray
* a and b = endpoints of the line (capsule)
* ra = radius of a
* rb = radius of b
*/
float sdCapsule2(vec3 p, vec3 a, vec3 b, float ra, float rb) {
	float timephase = phase;
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

// iq has this version, which seems a lot simpler?
float sdEllipsoid1( in vec3 p, in vec3 r ) {
	return (length( p/r ) - 1.0) * min(min(r.x,r.y),r.z);
}

//https://www.shadertoy.com/view/Xds3zN
float sdCone( in vec3 p, in vec3 c )
{
    vec2 q = vec2( length(p.xz), p.y );
    float d1 = -q.y-c.z;
    float d2 = max( dot(q,c.xy), q.y);
    return length(max(vec2(d1,d2),0.0)) + min(max(d1,d2), 0.);
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

vec3 smax_tex( vec3 a, vec3 b, float k )
{
	float k1 = k*k;
	float k2 = 1./k1;
	float d = log( exp(k2*a.z) + exp(k2*b.z) )*k1;
	// if d is closer to a.z, use a.xy
	// if d is closer to b.z, use b.xy
	// diff is b-a
	// t is d-a
	// ratio is t/diff
	float h = (d-a.z)/(b.z-a.z);
	return vec3(
		mix(a.xy, b.xy, h), //log(exp(a.xy) + exp(b.xy)), 
		d);
}

vec3 max_tex(vec3 a, vec3 b) {
	return (a.z > b.z) ? a : b;
}

vec3 min_tex(vec3 a, vec3 b) {
	return (a.z < b.z) ? a : b;
}

float ssub(in float A, in float B, float k) {
	return smax(A, -B, k);
}

vec3 ssub_tex(vec3 a, vec3 b, float k) {
	return smax_tex(a, -b, k);
}

vec3 sub_tex(vec3 a, vec3 b) {
	return max_tex(a, -b);
}

/////////////////////////////////////////////////////////////////////////////

// distorted capsules:

vec3 sdCapsule1_tex_z(vec3 p, float l, float r) {

	vec3 a = vec3(0);
	vec3 b = vec3(0, 0, l);
	vec2 uv;

	vec3 pa = p, ba = b;
	float t = p.z / l;
	float h = clamp( t, 0.0, 1.0 );
	
	// add some ripple:
	float h1 = h + 0.2*sin(PI * 4. * (t*t + phase* 0.3));
	h1 = clamp( h1, 0.0, 1.0 );
	
	// basic distance:
	vec3 rel = p - b*h;
	float d = length(rel);

	vec3 reln = rel/d;
	float angle = atan(reln.y, reln.x);
	uv.y = angle / TWOPI + 0.5;
	
	float tr = (p.z + r) / (l + r*2);
	uv.x = clamp(tr, 0., 1.);
	
	vec2 pt = uv * 10.;
	vec2 pf = fract(pt)-0.5;
	float ptpt = dot(pf, pf);
	pt *= 0.7;
	pt -= vec2(0.3, 0.123);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	pt *= 0.7;
	pt -= vec2(0.3, 0.123);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	pt *= 0.7;
	pt -= vec2(0.3, 0.123);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	float tiledeform = (0.5 - ptpt)*0.1;
	d += tiledeform;
	return vec3(uv, d - r);
}

// assumes l is not zero
// Leave l positive. Object seems to be using positive z as it's front, not negative z
vec3 sdCapsule2_tex_z(vec3 p, float l, float ra, float rb) {

	vec3 a = vec3(0);
	vec3 b = vec3(0, 0, l);
	vec2 uv;
	vec4 p1 = vec4(0.);

	vec3 pa = p, ba = b;
	float t = p.z / l;//(p.z * l) / (l * l)
	float h = clamp( t, 0.0, 1.0 );
	
	// add some ripple:
	float h1 = h + 0.2*sin(PI * 4. * (t*t + phase* 0.3));
	h1 = clamp( h1, 0.0, 1.0 );
	// TODO should clamp after ripple really
	
	// basic distance:
	vec3 rel = p - b*h;
	float d = length(rel);

	vec3 reln = rel/d;
	float angle = atan(reln.y, reln.x);
	uv.y = angle / TWOPI + 0.5;
	
	
	float lab = l + ra + rb;
	float tab = (p.z + ra) / lab;
	uv.x = clamp(tab, 0., 1.);
	
	d = d - mix(ra, rb, h1);

	vec2 pt = uv * 10.;
	vec2 pf = fract(pt)-0.5;
	float ptpt = dot(pf, pf);
	pt *= 0.7;
	pt -= vec2(0.3, 0.123);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	pt *= 0.7;
	pt -= vec2(0.3, 0.123);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	pt *= 0.7;
	pt -= vec2(0.3, 0.123);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	float tiledeform = (0.5 - ptpt)*0.1;
	d -= tiledeform;

	return vec3(uv, d);
}

/////////////////////////////////////////////////////////////////////////////
// NOTE scale := f(p/s)*s


vec3 part_ant_backbone(vec3 p) {
	// cheap symmetry:
	p.xy = abs(p.xy);
	float w = 0.15*abs(2.+0.5*sin(14.*p.z - 8.8*phase));
	float w2 = w*w;
	vec3 backbone = sdCapsule1_tex_z(pTranslate(p, vec3(0.,0.,-0.25)), -1+w2, w2);
	return backbone;
}

float fScene_old0(vec3 p) {

	//p = p.yxz;
	// basic symmetry:
	p.y = abs(p.y);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*phase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;

	//z = -1.0 of the second parameter of the sdCapsules is the front of the object.
	float a = sdCapsule1(p, vec3(0., 0., -0.25), vec3(0., y, z), w*w);
	float b = sdCapsule2(p, vec3(0., -0., -0.25), vec3(z, w, y), 0.125, 0.1);
	float c = sdCapsule2(p, vec3(0., -0., -0.0), vec3(z, w, y), 0.125, 0.1);
	float cube = sdCube((rotateY(-phase) * vec4(p, 1.0)).xyz);
	//float a = 0.7;
	//float b = 0.7;
	float d = smin(a, b, 0.5);
	float e = smin(d, c, 0.1);
	float f = smin(e, cube, 0.1);

	
	//float mouth = sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05));
	float mouth = sdEllipsoid1((rotateY(-phase) * vec4(p.yzx, 1.0)).xyz, vec3(0.25, 0.5, 0.05)); //Rotating the mouth Ellipsoid

	//return d * world_scale;
	return ssub(f, mouth, 0.2);
}

float fScene_old(vec3 p) {

	//p = p.yxz;
	// basic symmetry:
	p.x = abs(p.x);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*phase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;

	//z = -1.0 of the second parameter of the sdCapsules is the front of the object.
	float a = sdCapsule1(p, vec3(0., 0., -0.25), vec3(y, 0., z), w*w);
	float b = sdCapsule2(p, vec3(0., 0., -0.25), vec3(w, z, y), 0.125, 0.1);
	float c = sdCapsule2(p, vec3(0., 0., -0.0), vec3(w, z, y), 0.125, 0.1);
	float d = smin(a, b, 0.5);
	float f = smin(d, c, 0.1);

	float mouth = sdEllipsoid1(p.zxy, vec3(0.25, 0.5, 0.05));
	//float mouth = sdEllipsoid1((rotateY(-phase) * vec4(p.yzx, 1.0)).xyz, vec3(0.25, 0.5, 0.05)); //Rotating the mouth Ellipsoid

	//return d * world_scale;
	return f; //ssub(f, mouth, 0.1);
}

vec3 fScene_tex_z_old(vec3 p) {
	float dist = fScene_old(p/world_scale);
	return vec3(normalize(p).zx*0.5+0.5, dist * world_scale); 
}

float fScene_quadpod(vec3 p0) {
	float timephase = phase;

	vec3 p = p0;

	// cheap symmetry:
	p.xy = abs(p.xy);
	
	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*timephase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.25;
	float fz = sin(timephase*5.) * sign(p0.y)*0.25+0.25;

	float w2 = w*w;
	float backbone = part_ant_backbone(p).z;//sdCapsule1(p, vec3(0., 0., 1.-w2), vec3(0., 0., -1.+w2), w*w);
	float limbs = sdCapsule2(p, vec3(fz, fz, y), vec3(0., -0., -0.25), 0.125, 0.1);
	//float a = 0.7;
	//float b = 0.7;
	float d = smin(backbone, limbs, 0.5);
	
	//d = length(p.xy) - d;

	return ssub(d, sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05)), 0.125);
}

vec3 fScene_tex_z_quadpod(vec3 p) {
	float dist = fScene_quadpod(p/world_scale);
	return vec3(normalize(p).zx*0.5+0.5, dist * world_scale); 
}


vec3 fScene_tex_z_ant(vec3 p0) {
	float scl = world_scale;
	vec3 p = p0 / world_scale;	

	vec3 d = part_ant_backbone(p);
	return vec3(d.xy, d.z * world_scale); 
}

vec3 fScene_tex_z_nick(vec3 p) {
	float scl = world_scale;

	int species = id % 4;

	p /= scl;
	vec3 copyP = p;

	//p = p.yxz;
	// basic symmetry:
	p.x = abs(p.x);
	if(species == 2 || species == 1) p.y = abs(p.y);

	//p.z = quant(p.z, 0.05);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8 * phase));
	float wX = 0.125*abs(2.+0.5*sin(14.*p.x - 8.8 * phase));
	float wY = 0.125*abs(2.+0.5*sin(14.*p.y - 8.8 * phase));  //TODO: Play with undulation value.
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;
	float jointSpeed = 1.;

	//Starfish
	vec3 sf_a = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI / 2.), 0.5, w*w);
	vec3 sf_b = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI ), 0.5, w*w);
	vec3 sf_c = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI * 3./ 2.), 0.5, w*w);
	vec3 sf_d = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI * 2.), 0.5, w*w);
	vec3 sf_final = smin_tex(sf_a, sf_b, 0.3);

	//sdCapsule1_tex(p, vec3(0., 0., -0.25), vec3(0., y, z), w*w);
	vec3 a = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI / -6.), 0.5, w*w);
	//a = sdCapsule2_tex_z(pRotYZ(pTranslate(p, vec3(0, 0, -0.25)), PI / -6.), 0.25, 0.125, 0.1);
	vec3 b = sdCapsule2_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI / -2.5), 0.25, 0.15, 0.125);
	//vec3 b2 = sdCapsule2_tex_z(pRotYZ(pTranslate(p, vec3(0, 0, 0.35)), PI / -2.), 0.25, 0.02, 0.025);
	vec3 c = sdCapsule2_tex_z(pRotXZ(pTranslate(p, vec3(-0.2, 0., 0.2)), PI / -7.), 0.3, 0.1, 0.2);
	vec3 e = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0.2, 0)), PI / -8.), 0.4, w*w*0.8);

	//Creature 1: original blobby creature, uses vectors a, b, c, e
	vec3 d = smin_tex(a, b, 0.4);
	//d = smin_tex(d, a, 0.05);
	d = smin_tex(d, c, 0.3);
	d = smin_tex(d, e, 0.4);
	//d = a;
	//d = sub_tex(e, d);

	//wide backed creature [NOT USED]
	vec3 test = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., -0.5, 0.4)), 7 * PI / 4), 0.5, w*0.8);
	vec3 test2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., -0.5, 0.4)), TWOPI), 0.4, w*0.8);
	vec3 wings = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., -0.2, 0.)), phase*0.5*(3. * PI) / 2.), 0.75, 0.2);
	vec3 testFinal = smin_tex(test, wings, 0.4);

	//Creature 2: Worm-Like, bulbous head
	vec3 roundHeadX = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., 0., 0.65)), 3*PI/2), 0.3, wX*0.8);
	vec3 roundHeadY = sdCapsule1_tex_z(pRotYZ(pRotXZ(pTranslate(p, vec3(0., 0., 0.65)), 3*PI/2), 3*PI/2), 0.3, wY*0.8);
	vec3 roundHeadX_2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., 0., 0.6)), 3*PI/2), 0.2, wX*0.55);
	vec3 roundHeadY_2 = sdCapsule1_tex_z(pRotYZ(pRotXZ(pTranslate(p, vec3(0., 0., 0.55)), 3*PI/2), 3*PI/2), 0.2, wY*0.6);
	vec3 longBody = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., 0., 0.2)), TWOPI), 0.8, w*w*0.8 + 0.3);

	vec3 bulbWorm = smin_tex(roundHeadX, roundHeadY, 0.5);
	bulbWorm = smin_tex(bulbWorm, roundHeadX_2, 0.5);
	bulbWorm = smin_tex(bulbWorm, roundHeadY_2, 0.5);
	bulbWorm = smin_tex(bulbWorm, longBody, 0.2);




	//Creature 3: Squid/Octopus-Like
	//limbs:	TopBottom = limb closest to the top or bottom
	//			LeftRight = limb closest to the left or right
	//	This creature is identical in 4 xy quadrants (abs.xy)
	float sinPhase = sin(phase*4.5) * 0.125 + 0.125;
	float sinPhase2 = sin(phase*4.5) * 0.0625 + 0.1875;
	float sinPhase3 = sin(phase*4.5) * 0.0625 + 0.0625;

	vec3 body1 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., 0., 0.2)), 3*PI/2), 0.3, wX*0.3);
	vec3 body2 = sdCapsule1_tex_z(pRotYZ(pRotXZ(pTranslate(p, vec3(0., 0., 0.2)), 3*PI/2), 3*PI/2), 0.3, wY*0.3);
	vec3 limbTopBottom = sdCapsule1_tex_z(pRotYZ(pRotXZ(pTranslate(p, vec3(0., 0., 0.)), (3*PI/2) * -sinPhase), (5 * PI / 3) * -sinPhase2), 0.75, w*0.5);
	vec3 limbLeftRight = sdCapsule1_tex_z(pRotYZ(pRotXZ(pTranslate(p, vec3(0., 0., 0.)), (3*PI/2) * -sinPhase2), (11 * PI / 6) * -sinPhase3), 0.75, w*0.5);

	vec3 squidLike = smin_tex(limbTopBottom, limbLeftRight, 0.05);
	vec3 squidBod = smin_tex(body1, body2, 0.7);
	squidLike = smin_tex(squidLike, squidBod, 0.1);


	//Creature 4: Waterbear? see https://en.wikipedia.org/wiki/Tardigrade
	vec3 legs = sdCapsule1_tex_z(pRotYZ(pTranslate(p, vec3(0., -0.2, 0.4)), phase*-jointSpeed*PI), 0.45, 0.1);
	vec3 legs2 = sdCapsule1_tex_z(pRotYZ(pTranslate(p, vec3(-0.125, -0.2, 0.2)), phase*-jointSpeed*PI - 1.33), 0.45, 0.1);
	vec3 legs3 = sdCapsule1_tex_z(pRotYZ(pTranslate(p, vec3(-0.25, -0.2, 0.)), phase*-jointSpeed*PI - 2.66), 0.45, 0.1);
	vec3 legs4 = sdCapsule1_tex_z(pRotYZ(pTranslate(p, vec3(-0.375, -0.2, -0.2)), phase*-jointSpeed*PI - 4), 0.45, 0.1);
	vec3 legsHead = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., -0.4, 0.7)), 3*PI/2), 0.25, w*0.8);
	//TODO: Scale joint speed with the "forward" velocity of this creature
	legs = min_tex(legs, legs2);
	legs = min_tex(legs, legs3);
	legs = min_tex(legs, legs4);
	vec3 walkTest = smin_tex(test, test2, 0.5);
	walkTest = smin_tex(walkTest, legs, 0.5);
	walkTest = min_tex(walkTest, legsHead);

	//testFinal = walkTest;

	//Creature 5: Horeshoe Crab
	vec3 wing1 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., 0., 0.2)), TWOPI), 1.0, w*w*0.6);
	vec3 wing1_2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0., -0.2, 0.2)), TWOPI), 0.3, w*w*0.7);
	vec3 wing2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(-0.2, 0., 0.2)), TWOPI), 0.3, w*w*0.7);
	vec3 wing3 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(-0.4, 0., 0.)), TWOPI), 0.3, w*w*0.7);
	vec3 wing4 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(-0.6, 0., -0.2)), TWOPI), 0.3, w*w*0.7);
	vec3 wing2_2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(-0.1, -0.2, 0.2)), TWOPI), 0.3, w*w*0.7);
	vec3 wing3_2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(-0.3, -0.2, 0.)), TWOPI), 0.3, w*w*0.7);
	vec3 wing4_2 = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(-0.5, -0.1, -0.2)), TWOPI), 0.3, w*w*0.7);
	

	vec3 wingFinal;
	//wingFinal = smin_tex(wing1, wing2, 0.2);
	wingFinal = smin_tex(wing1_2, wing2, 0.2);
	wingFinal = smin_tex(wingFinal, wing2_2, 0.2);
	wingFinal = smin_tex(wingFinal, wing3, 0.2);
	wingFinal = smin_tex(wingFinal, wing3_2, 0.2);
	wingFinal = smin_tex(wingFinal, wing4, 0.2);
	wingFinal = smin_tex(wingFinal, wing4_2, 0.2);
	wingFinal = min_tex(wingFinal, wing1);
	

	vec3 f = smin_tex(a, c, 0.4);
	f = smin_tex(f, e, 0.2);

	vec3 j = smin_tex(b, c, 0.3);
	j = smin_tex(j, f, 0.4);

	//float mouth = sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05));

	//return d * world_scale;
	//return scl * ssub(d, mouth, 0.125);


	float gridripple = 0.01 * dot(sin(p.xyz * PI * 8.), cos(p.zxy * PI * 32.));

	//d.z += gridripple;
	//d.z -= tiledeform;

	//return vec3(d.xy, d.z * scl);
	vec3 baseGeo;
	
	if(species == 0){
		baseGeo = d;
	}else if (species <= 1){
		baseGeo = bulbWorm;
	}else if (species <= 2){
		baseGeo = squidLike;
	}else if (species <= 3){
		baseGeo = walkTest;
	}else if (species <= 4){
		baseGeo = wingFinal;
	}else{
		baseGeo = d;
	}
	vec3 baseGeometry = baseGeo;
    
	if(species == 2 || species == 1) {
		return vec3(baseGeometry.xy, baseGeometry.z * scl);
	}

	//making grass/hair on the creature
	//--------------------------------------------------
	//--- from https://www.shadertoy.com/view/ltSyzd ---
	//--------------------------------------------------
	vec3 norm = (p);
	float height = 0.12;// * d.y;
	float heightvar = 0.05;// * d.y;
	float density = 0.05 * (p.y + 0.2);
	float thickness = 0.5;// * d.y;

	const mat2 rot1 = mat2(0.99500416527,0.0998334166,-0.0998334166,0.99500416527);
	const mat2 rot2 = mat2(0.98006657784,0.19866933079,-0.19866933079,0.98006657784);
	const mat2 rot3 = mat2(0.95533648912,0.29552020666,-0.29552020666,0.95533648912);
	const mat2 rot4 = mat2(0.921060994,0.3894183423,-0.3894183423,0.921060994);
	const mat2 rot5 = mat2(0.87758256189,0.4794255386,-0.4794255386,0.87758256189);
	const mat2 rot6 = mat2(0.82533561491,0.56464247339,-0.56464247339,0.82533561491);
	const mat2 rot7 = mat2(0.76484218728,0.64421768723,-0.64421768723,0.76484218728);
	const mat2 rot8 = mat2(0.69670670934,0.7173560909,-0.7173560909,0.69670670934);
	const mat2 rot9 = mat2(0.62160996827,0.78332690962,-0.78332690962,0.62160996827);
	const mat2 rot10 = mat2(0.54030230586,0.8414709848,-0.8414709848,0.54030230586);

	vec3 normP = normalize(p);
	float angleP = acos(normP.y);

	p = pRotXZ(p, angleP);
	//p = pRotXZ(p, angleP);
    p.y = baseGeometry.z;
	//p = baseGeometry.zzz;
    //float hvar = texture(iChannel0, p.xz*0.075).x;
    float h = height + heightvar;//hvar*heightvar;
    
    vec2 t = phase * vec2(5.0, 4.3);
    vec2 windNoise = sin(p.xz*2.5 + flow.xz);//sin(p.xz*2.5 + t);
    //vec2 windNoise2 = sin(vec2(phase*1.5, phase + PI) + p.xz*1.0) * 0.5 + vec2(2.0, 1.0);
	vec2 windNoise2 = sin(vec2(phase*1.5, phase + PI) + p.xz*1.0) * 0.5 + vec2(2.0, 1.0);
    vec2 wind = (windNoise*0.45 + windNoise2*0.3) * (p.y);

    p.xz += wind;// + flow.xz;
	
	//TODO: Replace wind with flow

    vec3 p1 = opRep(p, vec3(density));
    p1 = vec3(p1.x, p.y - h, p1.z);
    float g1 = sdCone(p1, vec3(1.0, thickness, h));
    
    p.xz *= rot5;
	//p.xz *= the normal of baseGeometry.z ? TODO figure out how to wrap grass normals around creature normals
    vec3 p2 = opRep(p, vec3(density)*0.85);
    p2 = vec3(p2.x, p.y - h, p2.z);
    float g2 = sdCone(p2, vec3(1.0, thickness, h));
    
    p.xz *= rot10;
    vec3 p3 = opRep(p, vec3(density)*0.7);
    p3 = vec3(p3.x, p.y - h, p3.z);
    float g3 = sdCone(p3, vec3(1.0, thickness, h));
    
    p.xz *= rot3;
    vec3 p4 = opRep(p, vec3(density)*0.9);
    p4 = vec3(p4.x, p.y - h, p4.z);
    float g4 = sdCone(p4, vec3(1.0, thickness, h));
    
    float g = min(min(g1, g2), min(g3, g4));
    
    //float id = 1.0;
    
    //if(baseGeometry.z < epsilon)
   	//	id = 0.0;

	float gg = smin(g, baseGeometry.z, 0.01);
	//return vec3(min(g, baseGeometry.x), id, h);
	
	return vec3(baseGeometry.xy, gg * scl);
	//*/
	
}

 
float fScene_other(vec3 p) {
	float osc = (0.3+abs(sin(phase*7.)));
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
	pR(pc.yx, 0.+0.2*cos(phase * 7. + abs(a*PI/3.)));
	
	
	float c1 = fCylinder(pc.zxy, world_scale*.02, world_scale*0.7);
	
	float z = max(s1, -s0); 
	z = min(se, z); //max(b,-z);
	z = min(c1, z);
	return z;
}

vec3 fScene_tex_z_cashew(vec3 p) {
	float scl = world_scale;

	p /= scl;

	//p = p.yxz;
	// basic symmetry:
	p.x = abs(p.x);

	//p.z = quant(p.z, 0.05);

	// blobbies
	
	vec3 A = vec3(0., 0., -0.5);
	vec3 B = vec3(0., 0., 0.5);
	float w = 0.125*abs(2.+0.5*sin(14.*p.z - 8.8*phase));
	//float w = 0.4;
	float z = 0.25;
	float y = 0.5;

	//sdCapsule1_tex(p, vec3(0., 0., -0.25), vec3(0., y, z), w*w);
	vec3 a = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI / -6.), 0.5, w*w);
	//a = sdCapsule2_tex_z(pRotYZ(pTranslate(p, vec3(0, 0, -0.25)), PI / -6.), 0.25, 0.125, 0.1);
	vec3 b = sdCapsule2_tex_z(pRotXZ(pTranslate(p, vec3(0, 0, 0.2)), PI / -2.5), 0.25, 0.15, 0.125);
	//vec3 b2 = sdCapsule2_tex_z(pRotYZ(pTranslate(p, vec3(0, 0, 0.35)), PI / -2.), 0.25, 0.02, 0.025);
	vec3 c = sdCapsule2_tex_z(pRotXZ(pTranslate(p, vec3(-0.2, 0., 0.2)), PI / -7.), 0.3, 0.1, 0.2);
	vec3 e = sdCapsule1_tex_z(pRotXZ(pTranslate(p, vec3(0, 0.2, 0)), PI / -8.), 0.4, w*w*0.8);
	
	vec3 d = smin_tex(a, b, 0.4);
	//d = smin_tex(d, a, 0.05);
	d = smin_tex(d, c, 0.3);
	d = smin_tex(d, e, 0.4);
	//d = a;

	vec3 f = smin_tex(b, c, 0.5);

	//float mouth = sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05));

	//return d * world_scale;
	//return scl * ssub(d, mouth, 0.125);

	
	float gridripple = 0.01 * dot(sin(p.xyz * PI * 8.), cos(p.zxy * PI * 32.));

	//d.z += gridripple;
	//d.z -= tiledeform;

	return vec3(d.xy, d.z * scl);
}

vec3 fScene_tex_oldsegment(vec3 p0) {
	p0 = p0 / world_scale;

	vec3 p = p0;	
	// cheap symmetry:
	p.xy = abs(p.xy);
	float fz = (sin(phase*5) * sign(p.x)*0.5+0.5) * 3.;
	vec3 limbs = sdCapsule2_tex_z(pRotXZ(pRotYZ(pTranslate(p, vec3(0., 0., 0.15)), PI / (fz - 5)), PI / -6), 0.35, 0.03, 0.03);

	vec3 backbone = part_ant_backbone(p);
	vec3 d = smin_tex(backbone, limbs, 0.3);
	return vec3(d.xy, d.z * world_scale); //ssub(d, sdEllipsoid1(p.yzx, vec3(0.25, 0.5, 0.05)), 0.125);
}

vec3 fScene_tex_z_graham(vec3 p0) {
	float scl = world_scale;
	vec3 p = p0 / scl - vec3(0, 0.5, 0);

	// bounding box
	float d0 = fBox(p, vec3(1.));
	// bouding sphere
	float d1 = fSphere(p, 1.);

	vec3 ps = -p.xzy;
	ps.x = abs(ps.x);

	// cylinder
	/*
	
	float ac = sin(phase * PI * -3.);
	float lc = (ac+1.) * 0.2; 
	vec3 pc = pRotXY(ps, 1.);
	pc = pRotYZ(pc, -PI/2. * ac)
		+ vec3(0, lc, 0);
		*/

	vec3 pc = ps;
	float creps = 27.;
	//pc.xz = pc.zx;
	//pc = pRotXZ(pc, phase * 3.);
	float ca = pModPolarShift(pc.xz, creps, phase)/creps;
	float cra = (phase*-4. + ca*PI*2.);
	float cr = sin(cra);
	float cs = cos(cra);
	float cl = 0.25*(1.5*abs(cr));
	pc = pRotXY(pc, PI*(-0.8+0.3*cs)); 
	pc += vec3(0, -cl, 0);;

	float d2 = fCapsule(pc, 0.1, cl);

	float d4 = fSphere(ps-vec3(0.1,0.2,0.1), 0.4);
	float d3 = smin(d2, d4, 0.5);

	float d = d3;
	return vec3(p.yz*0.25+0.75, d * scl);
}


vec3 fScene_tex_z(vec3 p) {

	/*
	int species = 2;//id % 6;
	switch(species) {
		case 1: return fScene_tex_z_graham(p);
		case 2: return fScene_tex_z_quadpod(p);
		case 3: return fScene_tex_z_old(p);
		case 4: return fScene_tex_z_cashew(p);
		case 5: return fScene_tex_oldsegment(p);
		default: return fScene_tex_z_ant(p); 
	}*/
	return fScene_tex_z_old(p);
}

// compute normal from a SDF gradient by sampling 4 tetrahedral points around a location `p`
// (cheaper than the usual technique of sampling 6 cardinal points)
// `fScene` should be the SDF evaluator `float distance = fScene(vec3 pos)`  
// `eps` is the distance to compare points around the location `p` 
// a smaller eps gives sharper edges, but it should be large enough to overcome sampling error
// in theory, the gradient magnitude of an SDF should everywhere = 1, 
// but in practice this isn’t always held, so need to normalize() the result
// vec3 normal4(in vec3 p, float eps) {
// 	vec2 e = vec2(-eps, eps);
// 	// tetrahedral points
// 	float t1 = fScene(p + e.yxx), t2 = fScene(p + e.xxy), t3 = fScene(p + e.xyx), t4 = fScene(p + e.yyy); 
// 	vec3 n = (e.yxx*t1 + e.xxy*t2 + e.xyx*t3 + e.yyy*t4);
// 	// normalize for a consistent SDF:
// 	//return n / (4.*eps*eps);
// 	// otherwise:
// 	return normalize(n);
// }

vec3 normal4_tex(in vec3 p, float eps) {
	vec2 e = vec2(-eps, eps);
	// tetrahedral points
	float t1 = fScene_tex_z(p + e.yxx).z, 
		  t2 = fScene_tex_z(p + e.xxy).z, 
		  t3 = fScene_tex_z(p + e.xyx).z, 
		  t4 = fScene_tex_z(p + e.yyy).z; 
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
	
	    //d = fScene(p);
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
	//FragPosition.xyz = vertexpos;
	
	// also write to depth buffer, for detailed occlusion:
	FragPosition.xyz = (world_position + quat_rotate(world_orientation, p));
	gl_FragDepth = computeDepth(FragPosition.xyz, uViewProjectionMatrix);
	
    
    if (d < precis) {
		float cheap_self_occlusion = 1.-count; //pow(count, 0.75);
		FragColor.rgb = basecolor * cheap_self_occlusion;
		FragNormal.xyz = quat_rotate(world_orientation, normal4_tex(p, .01));

		vec3 pn = normalize(p);
		//FragTexCoord.xy = pn.zx*0.5+0.5;
		FragTexCoord.xy = d_tex.xy;
		
	} else if (t >= maxd) {
    	// shot through to background
    	discard;
    	
	} else {
		// too many ray steps
		
		//FragNormal.xyz = rd;
		//FragNormal.xyz = quat_rotate(world_orientation, normal4(p, .01));
		discard;
	}
	
	
}