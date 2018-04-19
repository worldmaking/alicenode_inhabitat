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
uniform float uFarClip;
uniform vec2 uDim;
uniform mat4 uViewMatrix;

in vec2 texCoord;
in vec3 ray_direction, ray_origin, eye_position;

out vec4 FragColor;

vec3 sky(vec3 dir) {
	vec3 n = dir*0.5+0.5;
	n.g = mix(n.g, n.r, 0.5);
	return mix(n, vec3(1.), 0.75);
}

float bump(float x, float peak, float null) {
	return smoothstep(0., peak, texCoord.x) * (1.-smoothstep(peak, null, texCoord.x));
}

void main() {
	vec2 inverseDim = 1./uDim;
	vec3 sides = vec3(inverseDim, 0.);
	vec4 basecolor = texture(gColor, texCoord);
	vec3 normal = texture(gNormal, texCoord).xyz;
	vec3 position = texture(gPosition, texCoord).xyz;
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
	float bumpl = bump(diff1, sides.x, sides.x*8.)


	// except, that we should be ignoring them if they are too large


	color.r = bump1 * 100.;
	
	//color = basecolor.rgb;
	
	// reflection vector 
	vec3 ref = reflect(rd, normal);
	float acute = abs(dot(normal, rd)); // how much surface faces us
	float oblique = 1.0 - acute; // how much surface is perpendicular to us
	//color *= 1. - 0.5*oblique;	

	//float metallic = acute;
	float metallic = oblique;
	//color.rgb *= mix(sky(ref), sky(normal), metallic);
		
	// fog effect:
	vec3 fogcolor = sky(rd);
	float fogmix = clamp(normalized_depth, 0., 1.);
	//color.rgb = mix(color.rgb, fogcolor, fogmix);

	// pos viz:
	//color.rgb = basecolor.xyz;


	// pos viz:
	//color.rgb = position.xyz;

	// viewspace:
	//color.rgb = view_position;

	// normal viz:
	//color.rgb = normal*0.5+0.5;

	// reflection vectors
	//color.rgb = ref.xyz;

	// depth viz:
	//color.rgb = vec3(normalized_depth);

	// edge finding by depth difference:
	float edges = clamp(depth-depthn, 0., 1.)*.5;
	//color.rgb = -rayDotN * vec3(inverseDim, 0.) * 5.;
	

	FragColor.rgb = color;	

}