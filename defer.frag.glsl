#version 330 core

/*
possible buffers:

base color RGB, alpha (opacity)
self-AO
material/shading model
material properties: metallic, roughness
subsurface color

other inputs:
depth (from position)

possible fx:
bloom, colour correction, antialiasing
*/

uniform sampler2D gColor;
uniform sampler2D gNormal;
uniform sampler2D gPosition;
uniform sampler2D gTexCoord;
uniform sampler3D uDistanceTex;
//uniform sampler3D uLandTex;
uniform sampler3D uEmissionTex;
uniform sampler3D uFluidTex;

uniform float uFarClip;
uniform vec2 uDim;
uniform mat4 uViewMatrix;
uniform mat4 uFluidMatrix;
uniform float time;
uniform float uFade;

in vec2 texCoord;
in vec3 ray_direction, ray_origin, eye_position;

out vec4 FragColor;

#define PI 3.14159265359

float daymix(float time, vec3 position) {
	return 0.65 + 0.3*sin(time*0.1 + (position.x + position.z * 0.2) * 0.004);
}

vec3 landcolor(vec2 texCoord) {
	vec3 aa = vec3(0.2, 0.2, 0.24);
	vec3 ab = vec3(0.1, 0.1, 0.1);
	vec3 ba = vec3(0.01, 0.01, 0.01);
	vec3 bb = vec3(0.015, 0.02, 0.025);

	vec3 a1 = mix(aa, ab, texCoord.x);
	vec3 a2 = mix(ba, bb, texCoord.x);
	return normalize(mix(a1, a2, texCoord.y));
}

vec3 sky(vec3 dir, vec3 pos) {
	vec3 n0 = dir*0.5+0.5;
	float day = daymix(time, pos + dir*200.);
	
	vec3 n = n0;
	float a = day; //time * 0.3;
	// detail
	n.r = sin(2.*PI* n.r*n.g + a)*0.5+0.5;
	n.g = cos(2.*PI* n.r*n.g - a)*0.5+0.5;
	// simplify
	n.g = mix(n.g, n.r, 0.5);

	// lighten
	n = mix(n, vec3(0.), 0.5);

	// below y=0 should be black
	float fogzone = sin((dir.y*0.8 + 0.3) * PI);
	// but also, looking up should go black?
	return n * clamp(fogzone, 0., 1.);
}

float fScene(vec3 p) {
	vec3 tc = (uFluidMatrix * vec4(p, 1.)).xyz;
    return texture(uDistanceTex, tc).x;
}

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
	vec2 inverseDim = 1./uDim;
	vec3 sides = vec3(inverseDim, 0.);
	vec4 basecolor = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;
	vec3 objtexcoord = texture(gTexCoord, texCoord).xyz;

	// TODO: fluid scale / transform?
	vec3 fluidtexcoord = (uFluidMatrix * vec4(position, 1.)).xyz;
	//vec3 fluidtexcoord = position; //
	vec3 fluid = texture(uFluidTex, fluidtexcoord).xyz;
	vec3 density = texture(uEmissionTex, fluidtexcoord).xyz;
	//float land = texture(uLandTex, fluidtexcoord).x;

	float dist = texture(uDistanceTex, fluidtexcoord).x;
	

	vec3 view_position = (uViewMatrix * vec4(position, 1.)).xyz;
	float depth = length(view_position); 
	float normalized_depth = depth/uFarClip;
	vec3 rd = normalize(ray_direction);
	vec3 ro = (uFluidMatrix * vec4(ray_origin, 1.)).xyz;
	vec3 ro1 = fluidtexcoord;


	vec3 color;

	// compare with next point:
	vec2 texCoordl = texCoord - sides.xz;
	vec2 texCoordr = texCoord + sides.xz;
	vec2 texCoordu = texCoord - sides.zy;
	vec2 texCoordd = texCoord + sides.zy;

/*
	vec3 positionl = texture(gPosition, texCoordl).xyz;
	float depthl = length((uViewMatrix * vec4(positionl, 1.)).xyz); 
	vec3 positionr = texture(gPosition, texCoordr).xyz;
	float depthr = length((uViewMatrix * vec4(positionr, 1.)).xyz); 
	vec3 positionu = texture(gPosition, texCoordu).xyz;
	float depthu = length((uViewMatrix * vec4(positionu, 1.)).xyz); 
	vec3 positiond = texture(gPosition, texCoordd).xyz;
	float depthd = length((uViewMatrix * vec4(positiond, 1.)).xyz); 
	float depthn = (depthl + depthr + depthu + depthd) / 4.;

	// ao should kick in if the near pixels are closer (depthx is smaller)
	// but this should also decay exponentially
	float aol = 1.-abs(max(depth - depthn, 0.) - inverseDim.y*20.);

	// what we are really looking for here is the curvature (convex or concave)
	// and this depends on the normal
	// that is, the normal should tell us what the expected depth would be
	// dot of the normal with the ray, scaled by pixel size?
	float rayDotN = dot(rd, normal);
	float expectedDepthl = depth + rayDotN*sides.x;
	float diffl = depthl - expectedDepthl;
*/
	// except, that we should be ignoring them if they are too large

	// set peak around 0.1, null around 1.
	//color.r = bump1(diff1, 0.1, 1.);
	
	color = basecolor.rgb;
	
	// reflection vector 
	vec3 ref = reflect(rd, normal);
	float acute = abs(dot(normal, rd)); // how much surface faces us
	float oblique = 1.0 - acute; // how much surface is perpendicular to us
	color *= 1. - 0.5*oblique;	

	// get environmental light from emissive sources
	// by lookup in the normal direction
	float nearby = 2.;
	vec3 texcoord_for_normal = (uFluidMatrix * vec4(position + normal*nearby, 1.)).xyz;
	vec3 envcolor = texture(uEmissionTex, texcoord_for_normal).rgb;
	vec3 texcoord_for_ref = (uFluidMatrix * vec4(position + ref*nearby, 1.)).xyz;
	vec3 envcolor_ref = texture(uEmissionTex, texcoord_for_ref).rgb;

	//float metallic = acute;
	float metallic = acute;
	//color.rgb = mix(vec3(0.5), normal*0.5+0.5, 0.2);
	color.rgb = mix(sky(ref, position), sky(normal, position), metallic);
	
	// edge finding by depth difference:
	//float edges = 1.-clamp(depth-depthn, 0., 1.)*.5;
	//color.rgb *= edges;

	// patterning
	vec2 tc = basecolor.xy;
	tc = tc * 4.5/4.;
	tc = mod(tc, 2.) - 1.;
	float d = length(tc) - 0.9;
	color.rgb += d < -0.1 ? vec3(0.3) : vec3(0.);

	// env color
	//color.rgb = mix(color.rgb, envcolor, clamp(d + 0.5, 0., 1.));
	//color.rgb = envcolor;
	//color.rgb += 0.1*basecolor.xyz;

	//color.rgb = color.rgb * 0.1 + density;
	
	// fog effect:
	vec3 fogcolor = sky(rd, position);
	//float fogmix = clamp(normalized_depth, 0., 1.);
	float fogmix = smoothstep(uFarClip*0.125, uFarClip, depth* 2.5);



	// base viz:
	color.rgb = basecolor.xyz;
	//color.rg = vec2(color.b);

	// emissive:
	//color.rgb = envcolor;
	//color.rgb = envcolor_ref;
	//color.rgb = mix(color.rgb, envcolor_ref, 0.25);
	

	// uv grid viz:
	vec2 uvgrid = clamp(pow((mod(8.*basecolor.xy,1.)-0.5)*2., vec2(16.)), 0., 1.);
	//color.rgb = vec3(uvgrid.y);
	//color.rgb = vec3(sin(basecolor.xy * 2 * PI) *0.5 + 0.5, 0.);
	//color.rgb += vec3(max(uvgrid.x, uvgrid.y));

	// uv application test
	vec2 uvgridTest = clamp(pow((mod(8.*basecolor.xy,1.)-0.5)*2., vec2(16.)), 0., 1.);
	//vec2 uvgridTest = clamp(pow((8.*basecolor.xy-0.5)*2., vec2(16.)), 0., 1.);
	//color.rgb = vec3(sin(basecolor.xy * 2 * PI) *0.5 + 0.5, 0.);
	//color.rgb += vec3(max(uvgridTest.x, uvgridTest.y));

	// pos viz:
	//color.rgb = position.xyz;
	//color.rgb = (position.xyz - 40.)/40.;
	//color.rgb = mod(position.xyz * vec3(1.), 1.);
	
	// viewspace:
	//color.rgb = view_position;

	// normal viz:
	//color.rgb = normal*0.5+0.5;

	// reflection vectors
	//color.rgb = ref.xyz*0.5+0.5;

	// depth viz:
	//color.rgb = vec3(normalized_depth);
	//color.rgb = vec3(mod(normalized_depth * vec3(1., 8., 64.), 1.));

	// fluid viz:
	//color.rgb = mod(fluidtexcoord, 1.);
	//color.rgb = mod(fluidtexcoord * 32., 1);
	//color.rgb += fluid.xyz*50. - 0.25;
	//color.rgb = 0.5 + fluid.xyz*100;

	// paint bright when normals point in the same direction as fluid:
	float sameness =  dot(fluid.xyz * 100., normal);
	//color.rgb = mix(vec3(0.25), color.rgb, sameness);
	//color.rgb = density;
	//color.rgb = (normalize(density)*0.5+0.5) * length(density);

	//color.rg = vec2(land / 32.);

	//color.rgb = vec3(texCoord, 0.);
	//color.rgb = rd;

	//color.rgb = vec3(vec2(mod(dist * 16., 1.)), mod(position.x, 1.));

	//color += normal*0.125;

	// TODO: how about fog by cone tracing through uEmissionTex?
	
	


	color.rgb = mix(color.rgb, vec3(0), fogmix);

	//color.rgb = ro;

	/*
	#define FOG_STEPS 32
	//float perstep = 1./float(FOG_STEPS);

	// what is n such that a*(n^FOG_STEPS)  == 1?
	// n = (1/a)^(1/FOG_STEPS)?
	
	// should be non-zero:
	float t = 0.125 / float(FOG_STEPS);
	// the step multipler that gets t to 1 after FOG_STEPS steps
	float n = pow(1./t, 1./float(FOG_STEPS));
	//float n = pow(1./t, 1./float(FOG_STEPS));

	// t1 is 1 when looking across the entire field dim
	float t1 = length(ro1 - ro);
	// TODO: actually we want to quit when we reach far_clip, which could be lower than the world_dim
	// how to get rid of banding??
	vec3 fcolor = vec3(0);
	for (int i=0; i<FOG_STEPS; i++) {
		float t2 = t * n;
		float overshoot = max(0.,t2-t1);
		float dt = (t2 - t) - overshoot;
		vec3 pt = ro + t*rd;
		vec3 c = textureLod(uEmissionTex, pt, 0.).rgb;
		float luma = dot(c,c);
		c = mix(c, fogcolor, t);
		//color = mix(color, vec3(luma), luma);
		fcolor += c * dt;
		color = mix(color, c, min(1., dt * luma * 0.1));
		//color = mix(color, vec3(luma), luma);
		//t += dt;	// this ought to grow as we go
		t = t2;
		if (t2 >= t1) {
			// something clever here for banding
			break;
		}
	}
*/
	//color = mix(color + fcolor, fcolor, t1);
	

	//color = objtexcoord;

	//FragColor.rgb += vec3(texCoord, 0.);

	float gamma = 0.9;
	//color = pow(color, vec3(gamma));

	FragColor.rgb = mix(color, vec3(0.), uFade);
	FragColor.a = 1.; //0.25;//0.05;
}