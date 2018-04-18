#version 330 core
uniform mat4 uViewMatrixInverse;
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
	float s = length(p)-(0.5*world_scale);
	float b = fBox(p, vec3(0.5*world_scale));
	return b;
}

void main() {
	// point_position is uniform over the fragements; we need to displace this according to the gl_PointCoord
	// but this is screen aligned; also need to unrotate to get world coordinate of the sprite
	vec3 offset = mat3(uViewMatrixInverse) * world_scale * vec3(2.*gl_PointCoord.x-1.,1.-2.*gl_PointCoord.y, 0.);
	vec3 vertex_position = point_position + offset;
	vec3 rd = normalize(vertex_position - eye_position);
	vec3 ro = vertex_position - rd * world_scale - point_position;

	float maxd = 2. * world_scale;
	float d = maxd;
	vec3 p = ro;
	float precis = 0.001;
	float count = 0.;
	float t = 0.;
	#define MAX_STEPS 4
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

	if (d < precis) {
		FragColor.rgb = vec3(1.);
	} else if (t > maxd) {
		FragColor.rgb = vec3(0.);
		discard;
	} else {
		FragColor.rgb = vec3(0.5);
		discard;
	}
	FragColor.rgb = vec3(1.-count);

	//FragColor = vec4(vec3(abs(d)), 0.5);
	//FragColor.rgb = ro;
}