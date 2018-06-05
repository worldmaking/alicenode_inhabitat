#version 330 core
uniform sampler2D tex;
uniform float time;
uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix;
uniform float uNearClip, uFarClip;
uniform sampler2D uFungusTex;
uniform sampler2D uLandTex;
uniform sampler3D uDistanceTex;
uniform mat4 uLandMatrix;

in vec2 texCoord;
in vec3 ray, origin, eyepos;
layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 FragNormal;
layout (location = 2) out vec3 FragPosition;

#define PI 3.14159265359
#define EPS 0.01

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

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.b, n.r, 0.8);
	return mix(n, vec3(1.), 0.75);
}

// for gl_FragDepth:
float computeDepth(vec3 p, mat4 viewProjectionMatrix) {
	float dfar = gl_DepthRange.far;
	float dnear = gl_DepthRange.near;
	vec4 clip_space_pos = viewProjectionMatrix * vec4(p, 1.);
	float ndc_depth = clip_space_pos.z / clip_space_pos.w;	
	// standard perspective:
	return (((dfar-dnear) * ndc_depth) + dnear + dfar) / 2.0;
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

// Repeat in two dimensions
vec2 pMod2(inout vec2 p, vec2 size) {
	vec2 c = floor((p + size*0.5)/size);
	p = mod(p + size*0.5,size) - size*0.5;
	return c;
}

// Repeat only a few times: from indices <start> to <stop> (similar to above, but more flexible)
vec2 pModInterval2(inout vec2 p, vec2 size, vec2 start, vec2 stop) {
	vec2 halfsize = size*0.5;
	vec2 c = floor((p + size*0.5)/size);
	p = mod(p+halfsize, size) - halfsize;
	if (c.x > stop.x) { //yes, this might not be the best thing numerically.
		p.x += size.x*(c.x - stop.x);
		c.x = stop.x;
	}
	if (c.x < start.x) {
		p.x += size.x*(c.x - start.x);
		c.x = start.x;
	}
	if (c.y > stop.y) { //yes, this might not be the best thing numerically.
		p.y += size.y*(c.y - stop.y);
		c.y = stop.y;
	}
	if (c.y < start.y) {
		p.y += size.y*(c.y - start.y);
		c.y = start.y;
	}
	return c;
}

// Rotate around a coordinate axis (i.e. in a plane perpendicular to that axis) by angle <a>.
// Read like this: R(p.xz, a) rotates "x towards z".
// This is fast if <a> is a compile-time constant and slower (but still practical) if not.
void pR(inout vec2 p, float a) {
	p = cos(a)*p + sin(a)*vec2(p.y, -p.x);
}

// Plane with normal n (n is normalized) at some distance from the origin
float fPlane(vec3 p, vec3 n, float distanceFromOrigin) {
	return dot(p, n) + distanceFromOrigin;
}

float fSphere(vec3 p, float r) {
	return length(p) - r;
}

// Box: correct distance to corners
float fBox(vec3 p, vec3 b) {
	vec3 d = abs(p) - b;
	return length(max(d, vec3(0))) + vmax(min(d, vec3(0)));
}

// Capsule: A Cylinder with round caps on both sides
float fCapsule(vec3 p, float r, float c) {
	return mix(length(p.xz) - r, length(vec3(p.x, abs(p.y) - c, p.z)) - r, step(c, abs(p.y)));
}

float smin( float a, float b, float k ) {
	float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );
	return mix( b, a, h ) - k*h*(1.0-h);
}

float fScene_test(vec3 p) {
	
	float x1 = min(0., sin(p.x * 4.)*sin(p.z* 4.));
	float plane = fPlane(p, vec3(0.,1.,0.), 0.01);
	
	vec3 pc = p;// + vec3(0, x1, 0);
	//vec2 c = pModInterval2(pc.xz, vec2(1.), vec2(-32.), vec2(32.));
	vec2 c = pMod2(pc.xz, vec2(0.25));
	float h = abs(sin(c.y*0.12)*sin(c.x*0.12));
	
	//pR(pc.yx, h*0.1*sin(c.y+time*1.3));
	//pR(pc.yz, h*0.1*sin(c.x+time*3.7));
	
	//float s = fSphere(pc, h); //
	//float b = fBox(pc, vec3(0.3, h, 0.3));
	float z = fCapsule(pc - vec3(0, h, 0), 0.04, 0.1);
	return min(z, plane);
}

float quant(float v, float s) {
	return floor(v/s)*s;
}

// Cylinder standing upright on the xz plane
float fCylinder(vec3 p, float r, float height) {
	float d = length(p.xz) - r;
	d = max(d, abs(p.y) - height);
	return d;
}

//https://www.shadertoy.com/view/Xds3zN
float sdCone( in vec3 p, in vec3 c )
{
    vec2 q = vec2( length(p.xz), p.y );
    float d1 = -q.y-c.z;
    float d2 = max( dot(q,c.xy), q.y);
    return length(max(vec2(d1,d2),0.0)) + min(max(d1,d2), 0.);
}

//https://www.shadertoy.com/view/Xds3zN
float sdCylinder( vec3 p, vec2 h )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

//https://www.shadertoy.com/view/Xds3zN
vec3 opRep( vec3 p, vec3 c )
{
    return mod(p,c)-0.5*c;
}

vec2 sinNoise(vec2 p)
{
    vec2 p1 = p;
    vec2 p2 = p * rot2 * 0.4;
    vec2 p3 = p * rot6 * 0.7;
    vec2 p4 = p * rot10 * 1.5;
	vec4 s1 = sin(vec4(p1.x, p1.y, p2.x, p2.y));
    vec4 s2 = sin(vec4(p3.x, p3.y, p4.x, p4.y));
    
    return (s1.xy + s1.zw + s2.xy + s2.zw) * 0.25;
}


float fScene(vec3 p) {
	vec3 tc = (uLandMatrix * vec4(p, 1.)).xyz;
	// distance to nearest floor (not same as direct down)
    float d = texture(uDistanceTex, tc).x;

	// surface detail:
	/*vec3 p1 = p * vec3(1,0,1) + vec3(0, d, 0);
	float d0 = fScene_test(p);
	float d1 = fScene_test(p1) + d;

	// a repetitive object:
	vec3 pt = p * 3.;
	vec3 pf = fract(pt)-0.5;
	float ptpt = dot(pf, pf);
	pt *= 0.7;
	pt -= vec3(0.3, 0.123, 0.89324);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	pt *= 0.7;
	pt -= vec3(0.3, 0.123, 0.89324);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	pt *= 0.7;
	pt -= vec3(0.3, 0.123, 0.89324);
	pf = fract(pt)-0.5;
	ptpt = min(ptpt, dot(pf, pf));
	float tiledeform = ptpt * -0.02 - 0.01;

	//d -= tiledeform;*/

	vec3 baseGeo = vec3(tc.xz, d);

	//making grass/hair on the creature
	//--------------------------------------------------
	//--- from https://www.shadertoy.com/view/ltSyzd ---
	//--------------------------------------------------
	const float height = 0.001;
	const float heightvar = 0.005;
	const float density = 1.0;
	const float thickness = 0.9;

	vec3 baseGeometry = baseGeo;

	float phase = time;
       
    p.y = baseGeometry.z;
    //float hvar = sinNoise(p.xz * 100.).x; //texture(iChannel0, p.xz*0.075).x;
	vec3 landtexcoord = (uLandMatrix * vec4(p, 1.)).xyz;
	float fungus = texture(uFungusTex, landtexcoord.xz).r;
    float h = height + fungus*heightvar;
    
    vec2 t = phase * vec2(5.0, 4.3);
    vec2 windNoise = sinNoise(p.xz*6.5 + t);
    vec2 windNoise2 = sin(vec2(phase*1.5, phase + PI) + p.xz*1.0) * 0.5 + vec2(2.0, 1.0);
    vec2 wind = (windNoise*0.95 + windNoise2*2.3) * (p.y);

    p.xz += wind;


    vec3 p1 = opRep(p, vec3(density));
    p1 = vec3(p1.x, p.y - h, p1.z);
    float g1 = sdCone(p1, vec3(1.0, thickness, h));
	//float g1 = fCylinder(p1, thickness, h);
    
    p.xz *= rot5;
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

	//Cylinder test*/
	/*
	vec3 p1 = opRep(p, vec3(density));
    p1 = vec3(p1.x, p.y - h, p1.z);
    float g1 = sdCylinder(p1, vec2(1.0, h));
    
    p.xz *= rot5;
    vec3 p2 = opRep(p, vec3(density)*0.85);
    p2 = vec3(p2.x, p.y - h, p2.z);
    float g2 = sdCylinder(p1, vec2(1.0, h));
    
    p.xz *= rot10;
    vec3 p3 = opRep(p, vec3(density)*0.7);
    p3 = vec3(p3.x, p.y - h, p3.z);
    float g3 = sdCylinder(p1, vec2(1.0, h));
    
    p.xz *= rot3;
    vec3 p4 = opRep(p, vec3(density)*0.9);
    p4 = vec3(p4.x, p.y - h, p4.z);
    float g4 = sdCylinder(p1, vec2(1.0, h));
    */
    float g = min(min(g1, g2), min(g3, g4));
	
    
    //float id = 1.0;
    
    //if(baseGeometry.z < epsilon)s
   //     id = 0.0;
    
	//return vec3(min(g, baseGeometry.x), id, h);
	//return vec3(baseGeometry.xy, min(g, baseGeometry.z));
	//*/

	//.float d3 = p.y + dot(sin(p/2. + cos(p.yzx/2. + 3.14159/2.)), vec3(.5)) - 0.1;

	return smin(d, g, 0.5); //min(d, d3);
}

float fScene0(vec3 p) {

	float a = fScene_test(p);
	vec3 landtexcoord = (uLandMatrix * vec4(p, 1.)).xyz;
	//float b = texture(uFungusTex, landtexcoord).r;
	return a; // - dy; // min(a, b);
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
	vec3 rd = normalize(ray);
	vec3 ro = origin; // plus a little ray? 

	

	#define MAX_STEPS 256
	#define STEP_SIZE 1./float(MAX_STEPS)
	float precis = 1./100.; //EPS;
	float maxd = uFarClip-uNearClip;
	
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
	
	vec3 fogcolor = sky(rd);

	//.if (d < 0.) discard;
	
	if (d < precis) {

		// DEBUG!!
		//if (mod(p.x*4.f, 1.f) < 0.75 && mod(p.z*4.f, 1.f) < 0.75) discard;
	
		
		float cheap_self_occlusion = 1.-pow(count, 0.75);
		FragColor.rgb = vec3(cheap_self_occlusion);
		FragNormal.xyz = normal4(p, .005);

		vec3 landtexcoord = (uLandMatrix * vec4(p, 1.)).xyz;
		float fungus = texture(uFungusTex, landtexcoord.xz).r;

		if (fungus > 0.) {
			// fungus:
			float smog = 0.7; // between 0.1 and 0.9
			float factor = fungus*smog;
			factor += 0.2; //factor += noise.z * 0.4; 
			//float w = min(1., h * 4.);
			float w = 1.;
			FragColor.rgb = mix(vec3(w), FragColor.rgb, factor);
		
		} else {
			//FragColor.rgb -= 0.8*(0.8-steepness);
		}

		vec4 land = texture(uLandTex, landtexcoord.xz);
		FragColor.rgb = vec3(fungus);
		//FragNormal.xyz = land.xyz;
		
	} else if (t >= maxd) {
    	// shot through to background
    	p = ro+maxd*rd;

		// ray direction to far clip
		vec3 rd1 = normalize(p - eyepos);
    	
		//discard;
    	FragColor.rgb = vec3(1.);
		FragNormal.xyz = -rd1;
    	
	} else { 

		// ray direction to point
		vec3 rd1 = normalize(p - eyepos);

		// too many ray steps
		FragColor.rgb = vec3(1.);
		FragNormal.xyz = -rd1;
	}

	
	
    	
	FragPosition.xyz = p;
	// also write to depth buffer, so that landscape occludes other creatures:
	gl_FragDepth = computeDepth(p, uViewProjectionMatrix);
}