#version 330 core
uniform sampler2D tex;
uniform float time;
uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse, uViewMatrix;

in vec2 texCoord;
in vec3 ray, origin, eyepos;
out vec4 FragColor;

#define PI 3.14159265359
#define EPS 0.01
#define VERYFARAWAY  400.
#define MAX_STEPS 128
#define STEP_SIZE 1./float(MAX_STEPS)

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = min(n.b, n.r);
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

float fScene(vec3 p) {
	
	float plane = fPlane(p, vec3(0.,1.,0.), 0.0);
	
	vec3 pc = p;
	//vec2 c = pModInterval2(pc.xz, vec2(1.), vec2(-32.), vec2(32.));
	vec2 c = pMod2(pc.xz, vec2(1.));
	float h = abs(sin(c.y*0.2)*sin(c.x*0.2));
	
	pR(pc.yx, h*0.2*sin(c.y+time*1.3));
	pR(pc.yz, h*0.2*sin(c.x+time*3.7));
	
	//float s = fSphere(pc, h); //
	//float b = fBox(pc, vec3(0.3, h, 0.3));
	float z = fCapsule(pc, 0.1, h);
	return min(z, plane);
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
	
	float precis = EPS;
	float maxd = VERYFARAWAY;
	
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
        t += d * 0.75;
        p = ro+rd*t;
        count += STEP_SIZE;
    }
	
	//FragColor = vec4(texCoord, 0.5, 1.);
	//FragColor = vec4(rd, 0.);
	//FragColor = texture(tex, texCoord);
	color = vec3(mix(1., 0., pow(count, 0.5)));
	
	vec3 fogcolor = sky(rd);
	
	if (d < precis) {
		vec3 matcolor = color; //vec3(0.7, 0.5, 0.3);
	
		vec3 n = normal4(p, .01);
		color = sky(n) * matcolor;
		//color *= vec3(0.5) * max(0., dot(n, vec3(1.)));
		//color += texture2D(tex0, pos2texcoord(p)).rgb;
		
		
		// fog effect:
		color = mix(color, fogcolor, pow(count, 2.));
		
		FragColor.rgb = color;
		
	} else if (t >= maxd) {
    	// shot through to background
    	p = ro+maxd*rd;
    	
    	//FragColor = vec4(clamp(fScene(p), 0., 1.));
    	//discard;
    	FragColor.rgb = fogcolor;
    	
	} else {
		// too many ray steps
		FragColor.rgb = fogcolor; //vec3(1.);
	}
	
    	
	
	// also write to depth buffer, so that landscape occludes other creatures:
	gl_FragDepth = computeDepth(p, uViewProjectionMatrix);
}