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
uniform sampler3D uDensityTex;
uniform sampler3D uFluidTex;

uniform float uFarClip;
uniform vec2 uDim;
uniform mat4 uViewMatrix;
uniform mat4 uFluidMatrix;
uniform float time;

in vec2 texCoord;
in vec3 ray_direction, ray_origin, eye_position;

out vec4 FragColor;

#define PI 3.14159265359

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	float a = time * 0.3;
	// detail
	n.r = sin(2.*PI* n.r*n.g + a)*0.5+0.5;
	n.g = cos(2.*PI* n.r*n.g - a)*0.5+0.5;
	// simplify
	n.g = mix(n.g, n.r, 0.5);
	// lighten
	return mix(n, vec3(0.), 0.75);
}

void main() {
	vec2 inverseDim = 1./uDim;
	vec3 sides = vec3(inverseDim, 0.);
	vec4 basecolor = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;

	// TODO: fluid scale / transform?
	vec3 fluidtexcoord = (uFluidMatrix * vec4(position, 1.)).xyz;
	//vec3 fluidtexcoord = position; //
	vec3 fluid = texture(uFluidTex, fluidtexcoord).xyz;
	vec3 density = texture(uDensityTex, fluidtexcoord).xyz;

	vec3 view_position = (uViewMatrix * vec4(position, 1.)).xyz;
	float depth = length(view_position); 
	float normalized_depth = depth/uFarClip;
	vec3 rd = normalize(ray_direction);


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
	//color *= 1. - 0.5*oblique;	

	//float metallic = acute;
	float metallic = oblique;
	//color.rgb = mix(vec3(0.5), normal*0.5+0.5, 0.2);
	color.rgb *= mix(sky(ref), sky(normal), metallic);
	
	// edge finding by depth difference:
	//float edges = 1.-clamp(depth-depthn, 0., 1.)*.5;
	//color.rgb *= edges;


	color.rgb = color.rgb * 0.1 + density;
	
	// fog effect:
	vec3 fogcolor = sky(rd);
	//float fogmix = clamp(normalized_depth, 0., 1.);
	float fogmix = smoothstep(uFarClip*0.25, uFarClip, depth);
	color.rgb = mix(color.rgb, fogcolor, fogmix);

	// base viz:
	//color.rgb = basecolor.xyz;

	// pos viz:
	//color.rgb = position.xyz;
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
	//color.rgb = (normalize(density)*0.5+0.5) * length(density);

	float gamma = 1.4;
	//color = pow(color, vec3(gamma));

	//color.rgb = vec3(texCoord, 0.);
	//color.rgb = rd;
	
	FragColor.rgb = color;	
}