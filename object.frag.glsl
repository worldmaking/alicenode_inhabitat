#version 330 core
uniform mat4 uViewProjectionMatrix, uViewProjectionMatrixInverse;
uniform float time;

in vec3 ray_direction, ray_origin;
in vec3 worldpos, objectpos, eyepos;

out vec4 FragColor;

#define PI 3.14159265359
#define EPS 0.01
#define VERYFARAWAY  4.
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

float fSphere(vec3 p, float r) {
	return length(p) - r;
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
	float s = fSphere(p, 0.99+0.5*abs(sin(time)));
	float b = fBox(p, vec3(1.));
	return max(b,-s);
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
	
    FragColor = vec4(rd, 1.0);
	
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
        t += d;
        p = ro+rd*t;
        count += STEP_SIZE;
    }
    
    FragColor = vec4(count);
    
    if (d < precis) {
		vec3 n = normal4(p, .01);
		color = n*0.5+0.5;
		//color *= vec3(0.5) * max(0., dot(n, vec3(1.)));
		//color += texture2D(tex0, pos2texcoord(p)).rgb;
		
		FragColor.rgb = color;
		//FragColor.rb += n.xz;
		
	} else if (t >= maxd) {
    	// shot through to background
    	
    	// locate on floor plane instead:
    	if (abs(rd.y) > 1e-6) { 
			t = -ro.y / rd.y; 
			p = ro+rd*t;
		} 
    	
    	//FragColor = vec4(clamp(fScene(p), 0., 1.));
    	//FragColor.rgb = mod((ro+0.5)*0.5,0.5)+0.5;
    	//FragColor.rgb = mod(rd,0.5)+0.5;
    	discard;
    	
	} else {
		// too many ray steps
		FragColor = vec4(1.);
	}
	
	// also write to depth buffer, so that landscape occludes other creatures:
	gl_FragDepth = computeDepth(worldpos + p, uViewProjectionMatrix);
	
	//FragColor.rgb = mod(rd * 8., 1.);
}