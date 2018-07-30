#include "al/al_console.h"
#include "al/al_math.h"
#include "al/al_distance.h"
#include "al/al_field3d.h"
#include "al/al_field2d.h"
#include "al/al_gl.h"
#include "al/al_obj.h"
#include "al/al_pod.h"
#include "al/al_kinect2.h"
#include "al/al_mmap.h"
#include "al/al_hmd.h"
#include "al/al_time.h"
#include "al/al_hashspace.h"
#include "al/al_jxf.h"
#include "al/al_json.h"
#include "al/al_opencv.h"
#include "alice.h"
#include "state.h"

struct Profiler {
	Timer timer;
	std::vector<std::string> logs;

	void reset() {
		logs.clear();
		timer.measure();
	}

	void log(std::string msg, double dt) {
		double m = timer.measure();
		char message[256];
		snprintf(message, 256, "%s: %f%%", msg.c_str(), m * 100./dt);
		logs.push_back(std::string(message));
	}

	void dump() {
		for (int i=0; i<logs.size(); i++) {
			console.log(logs[i].c_str());
		}
	}
};

/*
	A helper for deferred rendering
*/
struct GBuffer {

	static const int numBuffers = 4;

	unsigned int fbo;
	unsigned int rbo;
	std::vector<unsigned int> textures;
	std::vector<unsigned int> attachments;

	GLint min_filter = GL_LINEAR; 
	GLint mag_filter = GL_LINEAR;
	//GLint min_filter = GL_NEAREST;
	//GLint mag_filter = GL_NEAREST;

	glm::ivec2 dim = glm::ivec2(1024, 1024);

	GBuffer() {
		textures.resize(numBuffers);
		attachments.resize(numBuffers);
	}

	void configureTexture(int attachment=0, GLint internalFormat = GL_RGBA, GLenum format = GL_RGBA, GLenum type = GL_UNSIGNED_BYTE) {
		attachments[attachment] = GL_COLOR_ATTACHMENT0+attachment;
		glBindTexture(GL_TEXTURE_2D, textures[attachment]);
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, dim.x, dim.y, 0, format, type, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
		glBindTexture(GL_TEXTURE_2D, 0);
	}
	
	void dest_changed() {
		dest_closing();

		// create the GPU objects:
		glGenFramebuffers(1, &fbo);
		glGenRenderbuffers(1, &rbo);
		glGenTextures(textures.size(), &textures[0]);

		// configure textures:
		// color buffer
		configureTexture(0, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
		// normal buffer
		configureTexture(1, GL_RGB16F, GL_RGB, GL_FLOAT);
		// position buffer
		configureTexture(2, GL_RGB16F, GL_RGB, GL_FLOAT);
		// texcoord buffer
		configureTexture(3, GL_RGB16F, GL_RGB, GL_FLOAT);
		
		// configure RBO:
		glBindRenderbuffer(GL_RENDERBUFFER, rbo);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, dim.x, dim.y);
		// configure FBO:
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		// specify RBO:
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);
		// specify drawbuffers:
		glDrawBuffers(attachments.size(), &attachments[0]);
		// specify colour attachments:
		for (int i=0; i<textures.size(); i++) {
			glFramebufferTexture2D(GL_FRAMEBUFFER, attachments[i], GL_TEXTURE_2D, textures[i], 0);
		}
		// check if framebuffer is complete
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			std::cout << "Framebuffer not complete!" << std::endl;
			dest_closing();
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void dest_closing() {
		// TODO
		if (fbo) {
			glDeleteFramebuffers(1, &fbo);
			fbo = 0;
		}
		if (rbo) {
			glDeleteRenderbuffers(1, &rbo);
			rbo = 0;
		}
		if (textures[0]) {
			glDeleteTextures(textures.size(), &textures[0]);
			textures[0] = 0;
		}
	}

	void begin() { glBindFramebuffer(GL_FRAMEBUFFER, fbo); }
	static void end() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

	void bindTextures() {
		for (int i=textures.size()-1; i>=0; i--) {
        	glActiveTexture(GL_TEXTURE0+i);
        	glBindTexture(GL_TEXTURE_2D, textures[i]);
		}
	}

	void unbindTextures() {
        for (int i=textures.size()-1; i>=0; i--) {
        	glActiveTexture(GL_TEXTURE0+i);
        	glBindTexture(GL_TEXTURE_2D, 0);
		}
	}
};


// re-orient a vector `v` to the nearest direction 
// that it is orthogonal to a given `normal`
// without changing the vector's length
glm::vec3 make_orthogonal_to(glm::vec3 const v, glm::vec3 const normal) {
	// get component of v along normal:
	glm::vec3 normal_component = normal * (dot(v, normal));
	// remove this component from v:
	glm::vec3 without_normal_component = v - normal_component;
	// and re-scale to original magnitude:
	return safe_normalize(without_normal_component) * glm::length(v);
}

// re-orient a quaternion `q` to the nearest orientation 
// whose "up" vector (+Y) aligns to a given `normal`
// assumes `normal` is already normalized (length == 1)
glm::quat align_up_to(glm::quat const q, glm::vec3 const normal) {
	const float eps = 0.00001f;
	const float similar = 1.f-eps;
	// get q's up vector:
	glm::vec3 uy = quat_uy(q);
	// get similarity with normal:
	float dp = glm::dot(uy, normal);
	// if `direction` is already similar to `dir`, leave as-is
	if (fabsf(dp) > similar) return q; 
	// find an orthogonal axis to rotate around:
	glm::vec3 axis = glm::cross(uy, normal);
	// get the rotation needed around this axis:
	glm::quat diff = glm::angleAxis(acosf(dp), axis);
	// rotate the original quat to align to the normal:
	return safe_normalize(diff * q);
}

glm::quat get_forward_rotation_to(glm::quat const q, glm::vec3 const direction) {
	const float eps = 0.00001f;
	const float similar = 1.f-eps;
	float d = glm::length(direction);
	// if `direction` is too small, any direction is as good as any other... no change needed
	if (fabsf(d) < eps) return q; 
	// get similarity with direction:
	glm::vec3 desired = safe_normalize(direction);
	glm::vec3 uf = quat_uf(q);
	float dp = glm::dot(uf, desired); 
	// if `direction` is already similar to `dir`, leave as-is
	if (fabsf(dp) > similar) return q; 
	// get an orthogonal axis to rotate around:
	glm::vec3 axis = glm::cross(uf, desired);
	// get the rotation needed around this axis:
	glm::quat diff = glm::angleAxis(acosf(dp), axis);
	// rotate the original quat to align to the normal:
	return safe_normalize(diff);	
}

// re-orient a quaternion `q` to the nearest orientation 
// whose "forward" vector (-Z) aligns to a given `direction`
glm::quat align_forward_to(glm::quat const q, glm::vec3 const direction) {
	const float eps = 0.00001f;
	const float similar = 1.f-eps;
	float d = glm::length(direction);
	// if `direction` is too small, any direction is as good as any other... no change needed
	if (fabsf(d) < eps) return q; 
	// get similarity with direction:
	glm::vec3 desired = safe_normalize(direction);
	glm::vec3 uf = quat_uf(q);
	float dp = glm::dot(uf, desired); 
	// if `direction` is already similar to `dir`, leave as-is
	if (fabsf(dp) > similar) return q; 
	// get an orthogonal axis to rotate around:
	glm::vec3 axis = glm::cross(uf, desired);
	// get the rotation needed around this axis:
	glm::quat diff = glm::angleAxis(acosf(dp), axis);
	// rotate the original quat to align to the normal:
	return get_forward_rotation_to(q, direction) * q;
}

struct Projector {
	SimpleFBO fbo;

	// from calibration
	/*"ground_position" : [ -0.072518, -0.247243, -2.588981 ],
	"ground_quat" : [ -0.010204, -0.015899, -0.018713, 0.999646 ],
	"position" : [ -0.136746, -0.264017, 0.320403 ],
	"frustum" : [ -0.643185, 0.593786, -0.356418, 0.423955, 1.0, 10.0 ],
	"rotatexyz" : [ -1.656007, -2.649486, -2.022311 ],
	"quat" : [ -0.014037, -0.022858, -0.017975, 0.999479 ]*/

	glm::quat orientation;
	glm::vec3 location;
	glm::vec2 frustum_min, frustum_max; // assuming a near-clip distance of 1
	float near_clip, far_clip;

	glm::mat4 viewMat;
	glm::mat4 projMat;

	bool altShader = false;

	Projector() {
		fbo.dim.x = 1920;
		fbo.dim.y = 1080;
	}

	glm::mat4 view() {
		return glm::inverse(
			glm::translate(location) 
			* glm::mat4_cast(orientation)
		);
	}

	glm::mat4 projection() {
		return glm::frustum(
			near_clip * frustum_min.x,
			near_clip * frustum_max.x,
			near_clip * frustum_min.y,
			near_clip * frustum_max.y,
			near_clip,
			far_clip);
	}
};

//// RENDER STUFF ////

Shader objectShader;
Shader creatureShader;
Shader particleShader;
Shader landShader;
Shader landMeshShader;
Shader humanMeshShader;
Shader deferShader, deferShader2; 
Shader simpleShader;
Shader debugShader;
Shader flowShader;
Shader blendShader;

QuadMesh quadMesh;
GLuint colorTex;
FloatTexture3D fluidTex;
FloatTexture3D emissionTex;
FloatTexture3D distanceTex;

FloatTexture2D fungusTex;
FloatTexture2D landTex;
FloatTexture2D humanTex;
FloatTexture2D flowTex;
FloatTexture2D noiseTex;

TextureDrawer texDraw;

SimpleOBJ tableObj("island.obj", true, 1.f);

std::vector<Vertex> gridVertices;
std::vector<unsigned int> gridElements;

VAO gridVAO;
VBO gridVBO;
EBO gridEBO;
unsigned int grid_elements;

VAO tableVAO;
VBO tableVBO;
EBO tableEBO;
unsigned int table_elements;

VBO cubeVBO(sizeof(positions_cube), positions_cube);
VAO creatureVAO;
VBO creaturePartsVBO(sizeof(State::creatureparts));
VAO particlesVAO;
VBO particlesVBO(sizeof(State::particles));
VAO debugVAO;
VBO debugVBO(sizeof(State::debugdots));

int rendercreaturecount = 0;
int livingcreaturecount = 0;
int numants = 0;
int numboids = 0;


#define NUM_PROJECTORS 3
Projector projectors[NUM_PROJECTORS];

GBuffer gBufferVR;
GBuffer gBufferProj;
SimpleFBO slowFbo;

glm::mat4 viewMat;
glm::mat4 projMat;
glm::mat4 viewProjMat;
glm::mat4 viewMatInverse;
glm::mat4 projMatInverse;
glm::mat4 viewProjMatInverse;
Viewport viewport;
glm::vec3 eyePos;
glm::vec3 headPos; // in world space
// the location of the VR person in the world
glm::vec3 vrLocation;

glm::vec3 nextVrLocation;
int fadeState = 0;
int vrIsland = 0;
float timeToVrJump = 0;

uint8_t humanchar0[LAND_TEXELS];
uint8_t humanchar1[LAND_TEXELS];

//// DEBUG STUFF ////
int debugMode = 0;
int camMode = 2; 
int objectSel = 0; //Used for changing which object is in focus
int camModeMax = 4;
float camera_speed_default = 40.f;
float camera_turn_default = 3.f;
bool camFast = true;
glm::vec3 camVel, camTurn;
glm::vec3 cameraLoc = glm::vec3(0);
glm::quat cameraOri;
static int flip = 0;
int kidx = 0;
int sliceIndex = 0;
int soloView = 0;
bool showFPS = 0;

bool enablers[10];
#define SHOW_LANDMESH 0
#define SHOW_AS_GRID 1
#define SHOW_MINIMAP 2
#define SHOW_OBJECTS 3
#define SHOW_TIMELAPSE 4
#define SHOW_PARTICLES 5
#define SHOW_DEBUGDOTS 6
#define USE_OBJECT_SHADER 7
#define SHOW_HUMANMESH 8
#define CALIBRATE 9

Profiler profiler;

//// RUNTIME STUFF ///.

std::mutex sim_mutex;
MetroThread simThread(25);
MetroThread fieldThread(25);
MetroThread fluidThread(10);
MetroThread landThread(10);
bool isRunning = 1;
bool teleporting = false;

State * state;
Mmap<State> statemap;

AudioState * audiostate;
Mmap<AudioState> audiostatemap;

void fluid_update(double dt) { 
	if (Alice::Instance().isSimulating) state->fluid_update(dt); 
}

void State::fluid_update(float dt) {

	// disturb the fluid:
	// for (int y=0, i=0; y<LAND_DIM; y++) {
	// 	for (int x=0; x<LAND_DIM; x++, i++) {
	// 		glm::vec3 norm = glm::vec3(
	// 			x/float(LAND_DIM), 
	// 			human.front()[i], 
	// 			y/float(LAND_DIM)
	// 		);

	// 		glm::vec3 push = glm::vec3(
	// 			state->flow[i].x, 
	// 			0.f, 
	// 			state->flow[i].y); 
	// 		push = push * (flow_scale * dt);

	// 		al_field3d_addnorm_interp(field_dim, fluid_velocities.front(), norm, push);
	// 	}
	// }
	
	const glm::ivec3 dim = fluid_velocities.dim();
	const glm::vec3 field_dimf = glm::vec3(field_dim);
	// diffuse the velocities (viscosity)
	fluid_velocities.swap();
	al_field3d_diffuse(dim, fluid_velocities.back(), fluid_velocities.front(), glm::vec3(fluid_viscosity), fluid_passes);

	
	// apply boundary effect to the velocity field
	// boundary effect is the landscape, forcing the fluid to align to it when near
	{
		float minspeed = 100000.f;
		float avgspeed = 0.f;
		float maxspeed = 0.f;
		
		int i = 0;
		glm::vec3 * velocities = fluid_velocities.front();
		for (size_t z = 0; z<field_dim.z; z++) {
			for (size_t y = 0; y<field_dim.y; y++) {
				for (size_t x = 0; x<field_dim.x; x++, i++) {
					
					// get norm'd coordinate:
					glm::vec3 norm = glm::vec3(x,y,z) / field_dimf;
					glm::vec2 norm2 = glm::vec2(norm.x, norm.z);

					// sample flow field:
					auto flo = al_field2d_readnorm_interp(land_dim2, flowsmooth, norm2);

					// limit magnitude?
					float flospd = glm::length(flo);
					minspeed = glm::min(flospd, minspeed);
					maxspeed = glm::max(flospd, maxspeed);
					avgspeed += flospd;

					flo = flospd > fluid_flow_min_threshold ? flo : glm::vec2(0.f);

					// use this to sample the landscape:
					float sdist;
					al_field3d_readnorm_interp(sdf_dim, distance, norm, &sdist);
					float dist = fabsf(sdist);

					// TODO: what happens 'underground'?
					// should velocities here be zeroed? or set to a slight upward motion?	

					// generate a normalized influence factor -- the closer we are to the surface, the greater this is
					//float influence = glm::smoothstep(0.05f, 0.f, dist);
					// s is the amount of dist where the influence is 50%
					float s = fluid_contour_follow;
					float influence = s / (s + dist);


					glm::vec3& vel = velocities[i];

					vel.x += flo.x * flow_scale;
					vel.z += flo.y * flow_scale;
					
					// get a normal for the land:
					// TODO: or read from state->land xyz?
					glm::vec3 normal = sdf_field_normal4(sdf_dim, distance, norm, 1.f/SDF_DIM);
					// re-orient to be orthogonal to the land normal:
					glm::vec3 rescaled = make_orthogonal_to(vel, normal);
					// update:
					vel = mix(vel, rescaled, influence);	

					// also provide an outer boundary:
					glm::vec3 central = 0.5f-norm;
					central.y = 0.f;
					auto factor = glm::dot(central, central);
					central = glm::length(vel) * safe_normalize(central);
					// update:
					//vel = mix(vel, central, factor);	
				}
			}
		}
		//console.log("flow min speed %f max speed %f avg speed %f", minspeed, maxspeed, avgspeed / FIELD_VOXELS);
	}


	// stabilize:
	// prepare new gradient data:
	al_field3d_zero(dim, fluid_gradient.back());
	al_field3d_derive_gradient(dim, fluid_velocities.back(), fluid_gradient.back()); 
	// diffuse it:
	al_field3d_diffuse(dim, fluid_gradient.back(), fluid_gradient.front(), 0.5f, fluid_passes / 2);
	// subtract from current velocities:
	al_field3d_subtract_gradient(dim, fluid_gradient.front(), fluid_velocities.front());
	
	// advect:
	fluid_velocities.swap(); 
	al_field3d_advect(dim, fluid_velocities.front(), fluid_velocities.back(), fluid_velocities.front(), fluid_advection);

	// friction:
	al_field3d_scale(dim, fluid_velocities.front(), glm::vec3(fluid_decay));
	
}

void fields_update(double dt) { 
	if (Alice::Instance().isSimulating) state->fields_update(dt); 
}

void State::fields_update(float dt) {
	const glm::ivec2 dim = glm::ivec2(FUNGUS_DIM, FUNGUS_DIM);
	const glm::vec2 invdim = 1.f/glm::vec2(dim);

	if (1) {
		// this block is quite expensive, apparently
		float * const src_array = fungus_field.front(); 
		float * dst_array = fungus_field.back(); 


		for (int i=0, y=0; y<dim.y; y++) {
			for (int x=0; x<dim.x; x++, i++) {
				const glm::vec2 cell = glm::vec2(float(x),float(y));
				const glm::vec2 norm = cell*invdim;
				float C = src_array[i];
				float C1 = C - 0.1;
				//float h = 20 * .1;//heightmap_array.sample(norm);
				glm::vec4 l;
				al_field2d_readnorm_interp(glm::ivec2(LAND_DIM, LAND_DIM), land, norm, &l);
				float hm = l.w * field2world_scale - coastline_height;
				float h = l.w;
				float hu = al_field2d_readnorm_interp(land_dim2, human.front(), norm);
				float hum = hu * field2world_scale - coastline_height;
				float hlm = hum - hm;
				float dst = C;
				if (hm <= 0 || hlm > 2.) {
					// force lowlands to be vacant
					// (note, human will also do this)
					dst = 0;
					
				} else if (C < 0.) {
					// very negative values gradually drift back to zero
					// maybe this "fertility" should also depend on height!!
					dst = C + dt*fungus_recovery_rate;
				} else if (hm < 0.) {
					dst = 0.;
				} else if (rnd::uni() < hm * fungus_seeding_chance * dt) {
					// seeding chance
					dst = rnd::uni();
				} else if (rnd::uni() < fungus_decay_chance * dt / hm) {
					// if land lower than vitality, decrease vitality
					// also random chance of decay for any living cell
					//dst = C * rnd::uni(); //- dt*fungus_recovery_rate;
					dst = glm::max(C, rnd::uni());
				} else if (rnd::uni() < hm * fungus_migration_chance * dt) {
					// migration chance increases with altitude
					// pick a neighbour cell:
					glm::vec2 tc = cell + glm::vec2(floor(rnd::uni()*3)-1, floor(rnd::uni()*3)-1);
					float tv = al_field2d_read(dim, src_array, tc); //src_array.samplepix(tc);
					// if alive, copy it
					if (tv > 0.) { dst = tv; }
				}
				dst_array[i] = glm::clamp(dst, -1.f, 1.f);
			}
		}
		fungus_field.swap();
	}

	if (1) {
		// diffuse & decay the chemical fields:
		// copy front to back, then diffuse
		// TODO: is this any different to just diffuse (front, front) ? if not, we could eliminate this copy
		// (or, would .swap() rather than .copy() work for us?)
		chemical_field.swap();	
		al_field2d_diffuse(fungus_dim, chemical_field.back(), chemical_field.front(), chemical_diffuse, 2);
		size_t elems = chemical_field.length();
		for (size_t i=0; i<elems; i++) {
			// the current simulated field values:
			glm::vec3& chem = chemical_field.front()[i];
			float f = fungus_field.front()[i];
			// the current smoothed fields as used by the renderer:
			glm::vec4& tex = field_texture[i];
			// the current smoothed fungus value:
			float f0 = tex.w; 

			// fungus increases smoothly, but death is immediate:
			float f1 = (f <= 0) ? f : glm::mix(f0, f, 0.1f);

			// other fields just clamp & decay:
			chem = glm::clamp(chem * chemical_decay, 0.f, 1.f);

			// now copy these modified results back to the field_texture:
			tex = glm::vec4(chem, f1);
		}
	}
		
	if (1) {
		// diffuse and decay the emission field:
		al_field3d_scale(field_dim, emission_field.back(), glm::vec3(emission_decay));
		al_field3d_diffuse(field_dim, emission_field.back(), emission_field.front(), emission_diffuse);
		emission_field.swap();
	}
}

void land_update(double dt) { 
	if (Alice::Instance().isSimulating) state->land_update(dt); 
	state->generate_land_sdf_and_normals();
}

void State::land_update(float dt) {
	for (int y=0; y<LAND_DIM; y++) {
		for (int x=0; x<LAND_DIM; x++) {
			auto land_idx = al_field2d_index_nowrap(land_dim2, x, y);

			float h = human.front()[land_idx];
			glm::vec4& landpt = land[land_idx];
			//landpt.w = h;

			if (h == 0.f) continue;

			float h1;

			// glm::vec4& landpt = land[land_idx];
			if (h <= landpt.w) {
				// fall quickly:
				h1 = glm::mix(landpt.w, h, land_fall_rate * dt);
			} else {
				// fall quickly:
				h1 = glm::mix(landpt.w, h, land_rise_rate * dt);
			}

			landpt.w = h1;//glm::mix(landpt.w, h1, 0.2f);
		}
	}

	// maybe diffuse too to smoothen land?
	
	// next, calculate normals
	// THIS IS TOO SLOW!!!!
	//generate_land_sdf_and_normals();
}

void sim_update(double dt) { 
	if (Alice::Instance().isSimulating) state->sim_update(dt); 
}

void State::sim_update(float dt) {

	// inverse dt gives rate (per second)
	float idt = 1.f/dt;

	Alice& alice = Alice::Instance();


	// get the most recent complete frame:
	flip = !flip;
	const CloudDevice& cd = alice.cloudDeviceManager.devices[0];
	const CloudFrame& cloudFrame = cd.cloudFrame();
	const glm::vec3 * cloud_points = cloudFrame.xyz;
	const glm::vec2 * uv_points = cloudFrame.uv;
	const glm::vec3 * rgb_points = cloudFrame.rgb;
	uint64_t max_cloud_points = sizeof(cloudFrame.xyz)/sizeof(glm::vec3);
	glm::vec2 kaspectnorm = 1.f/glm::vec2(float(cColorHeight)/float(cColorWidth), 1.f);
	glm::vec3 kinectloc = world_centre + glm::vec3(0,0,-4);

	// deal with Kinect data:
	CloudDevice& kinect0 = alice.cloudDeviceManager.devices[0];
	CloudDevice& kinect1 = alice.cloudDeviceManager.devices[1];

	// map depth data onto land:
	if (1) {
		// first, dampen the human field:
		for (int i=0; i<LAND_TEXELS; i++) {
			human.front()[i] *= human_height_decay;
		}
		
		// for each Kinect
		for (int k=0; k<2; k++) {
			//if (k == 11) continue;

			
			const CloudFrame& cloudFrame0 = k ? kinect1.cloudFrame() : kinect0.cloudFrame();
			const CloudFrame& cloudFrame1 = k ? kinect1.cloudFramePrev() : kinect0.cloudFramePrev();
			const glm::vec3 * cloud_points0 = cloudFrame0.xyz;
			const glm::vec2 * uv_points0 = cloudFrame0.uv;
			const glm::vec3 * rgb_points0 = cloudFrame0.rgb;
			const uint16_t * depth0 = cloudFrame0.depth;

			// now update with live data:
			for (int i=0, y=0; y < cDepthHeight; y++) {
				for (int x=0; x < cDepthWidth; x++, i++) {

					// masks:
					if (k==1 && (x > cDepthWidth * 0.7 && y > cDepthWidth * 0.5)) continue;
					if (k==1 && (x > cDepthWidth * 0.85)) continue;
					if (k==0 && (y < cDepthHeight * 0.3 && x > cDepthWidth * 0.75)) continue;

					auto uv = (uv_points0[i] - glm::vec2(0.5f, 0.5f)) * kaspectnorm;
					if (k==0 && glm::length((uv_points0[i] - glm::vec2(0.17f, 0.93f))* kaspectnorm) < 0.2f) continue;

					if (k==0 && glm::length((uv_points0[i] - glm::vec2(0.57f, 0.8f))* kaspectnorm) < 0.15f) continue;	

					if (k==0 && glm::length((uv_points0[i] - glm::vec2(0.9f, 0.93f))* kaspectnorm) < 0.35f) continue;

					auto pt = cloud_points0[i];
					// filter out bad depths
					// mask outside a circular range
					// skip OOB locations:
					uint16_t min_dist = 3000;
					uint16_t max_dist = 6000;
					if (
						depth0[i] <= min_dist 
						|| depth0[i] >= max_dist
						|| pt.x < world_min.x
						|| pt.z < world_min.z
						|| pt.x > world_max.x
						|| pt.z > world_max.z
						|| pt.y > (2.0 * kinect2world_scale)
						) continue;

					// find nearest land cell for this point:
					// get norm'd coordinate:
					glm::vec3 norm = transform(world2field, pt);
					glm::vec2 norm2 = glm::vec2(norm.x, norm.z);
					// get cell index for this location:
					int landidx = al_field2d_index_norm(land_dim2, norm2);
					
					// set the land value accordingly:
					float& humanpt0 = human.front()[landidx];
					float& humanpt1 = human.back()[landidx];

					float h = world2field_scale * pt.y;
					humanpt1 = glm::mix(humanpt0, h, 0.4f);

					// in archi15 we also did spatial filtering

				}
			}
		} // end 2 kinects
		
		//al_field2d_diffuse(land_dim2, human.back(), human.front(), 0.5f, 3);
		human.swap();
		// NOW FLOW
#ifdef AL_WIN
		if (1) {

			// copy human to char arrays for CV:
			for (int i=0; i<LAND_TEXELS; i++) {
				humanchar0[i] = humanchar1[i];
				humanchar1[i] = human.front()[i] * 255;
			}
			
			int levels = 3; // default=5;
			double pyr_scale = 0.5;
			int winsize = 13;
			int iterations = 3; // default = 10;
			int poly_n = 5;
			double poly_sigma = 1.2; // default = 1.1
			int flags = 0;
			
			// create CV mat wrapper around Jitter matrix data
			// (cv declares dim as numrows, numcols, i.e. dim1, dim0, or, height, width)
			void * src;
			cv::Mat prev(LAND_DIM, LAND_DIM, CV_8UC(1), (void *)humanchar0);
			cv::Mat next(LAND_DIM, LAND_DIM, CV_8UC(1), (void *)humanchar1);
			cv::Mat flow(LAND_DIM, LAND_DIM, CV_32FC(2), (void *)state->flow);
			cv::calcOpticalFlowFarneback(prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);

			for (int i=0; i<LAND_TEXELS; i++) {
				flowsmooth[i] += flow_smoothing * (state->flow[i] - flowsmooth[i]);
			}
			
		}
#endif

		
	}
	if (!alice.isSimulating) return;

	if (1) {
		{
			const CloudFrame& cloudFrame1 = kinect1.cloudFrame();
			const glm::vec3 * cloud_points1 = cloudFrame1.xyz;
			const glm::vec2 * uv_points1 = cloudFrame1.uv;
			const glm::vec3 * rgb_points1 = cloudFrame1.rgb;
			const uint16_t * depth1 = cloudFrame1.depth;

			const CloudFrame& cloudFrame0 = kinect0.cloudFrame();
			const glm::vec3 * cloud_points0 = cloudFrame0.xyz;
			const glm::vec2 * uv_points0 = cloudFrame0.uv;
			const glm::vec3 * rgb_points0 = cloudFrame0.rgb;
			const uint16_t * depth0 = cloudFrame0.depth;


			int drawn = 0;
			
			for (int i=0, ki=0; i<NUM_DEBUGDOTS; i++) {
				DebugDot& o = debugdots[i];
				
				if (kinect0.capturing && i < max_cloud_points) {
					auto uv = (uv_points0[ki] - 0.5f) * kaspectnorm;
					if (depth0[ki] > 0 && glm::length(uv) < 0.5f) {
						o.location = cloud_points0[ki];
						o.color = glm::vec3(0.8, 0.5, 0.3);
						drawn++;
					}
					ki = (ki+1) % max_cloud_points;
				} else if (kinect1.capturing && i >= max_cloud_points) {
					auto uv = (uv_points1[ki] - 0.5f) * kaspectnorm;
					if (depth1[ki] > 0 && glm::length(uv) < 0.5f) {
						o.location = cloud_points1[ki];
						o.color = glm::vec3(0.5, 0.8, 0.3);
						drawn++;
					}
					ki = (ki+1) % max_cloud_points;
				}
			}
			//console.log("%d %d / %d", cDepthHeight*cDepthWidth, max_cloud_points, drawn);


		}
	}


	if (1) {
		for (int i=0; i<NUM_PARTICLES; i++) {
			Particle &o = particles[i];

			// get norm'd coordinate:
			glm::vec3 norm = transform(world2field, o.location);
			float h = al_field2d_readnorm_interp(land_dim2, land, glm::vec2(norm.x, norm.z)).w * field2world_scale;
			

			//glm::vec3 flow;
			//fluid.velocities.front().readnorm(transform(world2field, o.location), &flow.x);
			glm::vec3 flow = al_field3d_readnorm_interp(field_dim, fluid_velocities.front(), norm);

			// noise:
			flow += glm::sphericalRand(particle_noise);

			o.velocity = flow * idt;

			// chance of becoming egg?
			float hdist = fabsf(o.location.y - h);
			if (hdist < particle_to_egg_distance) {
				// spawn new?
				//console.log("creature pool count %d", creature_pool.count);
				if (creature_pool.count) {
					auto i = creature_pool.pop();
					//birthcount++;
					//console.log("spawn %d", i);
					creature_reset(i);
					Creature& a = creatures[i];
					a.location = o.location;
					//a.island = nearest_island(a.location);
				}

			} 
			
			if (rnd::uni() < (creature_to_particle_chance * dt)) {
				int idx = i % NUM_CREATURES;
				o.location = creatures[idx].location;
			} else if (
				o.location.y < h 
				|| o.location.y > world_centre.y
				 || o.location.x < world_min.x
				 || o.location.x > world_max.x
				 || o.location.z < world_min.z
				 || o.location.z > world_max.z
				) {
				
				o.location = random_location_above_land(coastline_height);
				int idx = i % NUM_CREATURES;
				//o.location = creatures[idx].location;
			} else {
				o.location.x = wrap(o.location.x, world_min.x, world_max.x);
				o.location.z = wrap(o.location.z, world_min.z, world_max.z);
			}



			
		}
	} 

	// update all creatures
	creatures_health_update(dt);

	// simulate creature pass:
	for (int i=0; i<NUM_CREATURES; i++) {
		auto &o = creatures[i];
		AudioState::Frame& audioframe = audiostate->frames[o.idx % NUM_AUDIO_FRAMES];

		if (o.state == Creature::STATE_ALIVE) {
			creature_alive_update(o, dt);

			audioframe.state = float(o.type) * 0.1f;
			audioframe.state = float(rnd::integer(4) + 1) * 0.1f;
			audioframe.speaker = o.island * 0.1f;
			audioframe.health = o.health;
			audioframe.age = o.phase;
			audioframe.size = o.scale;
			audioframe.params = glm::vec3(o.params);
			
		} else {

			audioframe.state = 0;
		}
	}
}

glm::vec3 State::random_location_above_land(float h) {
	glm::vec3 result;
	glm::vec2 p;
	glm::vec4 landpt;
	int runaway = 100;
	while (runaway--) {
		p = glm::linearRand(glm::vec2(0.f), glm::vec2(1.f));
		landpt = al_field2d_readnorm_interp(land_dim2, land, p);
		result = transform(field2world, glm::vec3(p.x, landpt.w, p.y));
		if (result.y > h) {
			break;
		}
	}
	//console.log("runaway %d", runaway);
	return result;
}

int State::nearest_island(glm::vec3 pos) {
	int which = -1;
	float d2 = 1000000000.f;
	for (int i=0; i<NUM_ISLANDS; i++) {
		auto r = island_centres[i] - pos;
		auto d = glm::dot(r, r);
		if (d < d2) {
			which = i;
			d2 = d;
		}
	}
	return which;
}

void State::creature_reset(int i) {
	int island = rnd::integer(NUM_ISLANDS);
	Creature& a = creatures[i];
	a.idx = i;
	//a.type = (rnd::integer(2) + 1) * 2;
	a.type = (rnd::integer(2) + 1);
	//if (rnd::uni() < 0.01) a.type = Creature::TYPE_PREDATOR_HEAD;
	a.state = Creature::STATE_ALIVE;
	a.health = rnd::uni();

	a.location = random_location_above_land(coastline_height * 1.2);
	//a.location = island_centres[island];
	a.scale = rnd::uni(0.5f) + 0.75f;
	a.orientation = glm::angleAxis(rnd::uni(float(M_PI * 2.)), glm::vec3(0,1,0));
	a.color = glm::linearRand(glm::vec3(0.25), glm::vec3(1));
	a.phase = rnd::uni();
	a.params = glm::linearRand(glm::vec4(0), glm::vec4(1));

	a.velocity = glm::vec3(0);
	a.rot_vel = glm::quat();
	a.island = nearest_island(a.location);

	switch(a.type) {
		case Creature::TYPE_ANT:
			a.ant.nest_idx = island;
			a.location = island_centres[island];
			a.ant.food = 0;
			a.ant.nestness = 1;
			a.scale *= 0.75f;
			break;
		case Creature::TYPE_BUG:
			break;
		case Creature::TYPE_BOID:
			a.scale *= 0.75f;
			break;
		case Creature::TYPE_PREDATOR_HEAD:
			a.pred_head.victim = -1;
			break;
	}
}

void State::creatures_health_update(float dt) {

	int birthcount = 0;
	int deathcount = 0;
	int recyclecount = 0;

	// spawn a predator?
	// if (rnd::integer(NUM_CREATURES) < creature_pool.count/4
	// 	&& creature_pool.count > 7) {

	// 	auto idx = creature_pool.pop();
	// 	creature_reset(idx);
	// 	Creature& a = creatures[idx];
	// 	a.type = Creature::TYPE_PREDATOR_HEAD;
	// 	for (int i=0; i<6; i++) {
	// 		auto seg = creature_pool.pop();
	// 	}
	// }

	if (0) {
		// spawn new?
		//console.log("creature pool count %d", creature_pool.count);
		if (rnd::integer(NUM_CREATURES) < creature_pool.count) {
			auto i = creature_pool.pop();
			birthcount++;
			//console.log("spawn %d", i);
			creature_reset(i);
			Creature& a = creatures[i];
			//a.location = glm::linearRand(world_min, world_max);
			//a.island = nearest_island(a.location);
		}
	}

	// visit each creature:
	for (int i=0; i<NUM_CREATURES; i++) {
		Creature& a = creatures[i];
		if (a.state == Creature::STATE_ALIVE) {
			if (a.health < 0) {
				//console.log("death of %d", i);
				deathcount++;
				// remove from hashspace:
				hashspace.remove(i);
				// set new state:
				a.state = Creature::STATE_DECAYING;
				
				continue;
			}

			// simulate as alive
			hashspace.move(i, glm::vec2(a.location.x, a.location.z));

			// // birth chance?
			// if (rnd::uni() < a.health * reproduction_health_min * dt
			// 	&& rnd::integer(NUM_CREATURES) < creature_pool.count
			// 	&& (a.type == Creature::TYPE_BOID 
			// 	|| a.type == Creature::TYPE_ANT)) {
			// 	auto j = creature_pool.pop();
			// 	birthcount++;
			// 	//console.log("child %d", i);
			// 	creature_reset(j);
			// 	Creature& child = creatures[j];
			// 	child.type = a.type;
			// 	if (child.type != Creature::TYPE_ANT) child.location = a.location;
			// 	child.island = nearest_island(child.location);
			// 	child.orientation = glm::slerp(child.orientation, a.orientation, 0.5f);
			// 	child.params = glm::mix(child.params, a.params, 0.9f);
			// }


			// TODO: make this species-dependent?
			a.health -= dt * alive_lifespan_decay;// * (1.+rnd::bi()*0.1);

		} else if (a.state == Creature::STATE_DECAYING) {
			
			glm::vec3 norm = transform(world2field, a.location);
			glm::vec2 norm2 = glm::vec2(norm.x, norm.z);
			
			// decay complete?
			if (a.health < -1) {
				//console.log("recycle of %d", i);
				recyclecount++;
				a.state = Creature::STATE_BARDO;
				dead_space.unset(norm2);
				creature_pool.push(i);
				continue;
			}

			// retain corpse in deadspace:
			// (TODO: Is this needed, or could a hashspace query do what we need?)
			dead_space.set_safe(i, norm2);

			// simulate as dead:
			
			// rate of decay:
			float decay = dt * dead_lifespan_decay;// * (1.+rnd::bi()*0.1);
			a.health -= decay;

			// blend to dull grey:
			float grey = (a.color.x + a.color.y + a.color.z)*0.25f;
			a.color += dt * (glm::vec3(grey) - a.color);

			// deposit blood:
			al_field2d_addnorm_interp(fungus_dim, chemical_field.front(), norm2, decay * blood_color);
		}
	}
	//console.log("%d deaths, %d recycles, %d births", deathcount, recyclecount, birthcount);
}

float State::ant_sniff_turn(Creature& a, float p1, float p2) {
		
	//-- is there any pheromone near?
	//-- (use random factor to avoid over-reliance on pheromone trails)
	float pnear = p1+p2; // - rnd::uni();
	if (pnear * rnd::uni() > ant_sniff_min) {
		//-- turn left or right?
		a.rot_vel = glm::angleAxis(p1 > p2 ? -ant_follow : ant_follow, quat_uf(a.orientation)) * a.rot_vel;
	} 
	return pnear;
}

void State::creature_alive_update(Creature& o, float dt) {
	float idt = 1.f/dt;
	// float daylight_factor = sin(daytime + 1.5 * a.pos.x) * 0.4 + 0.6; // 0.2 ... 1


	/// FOR ALL CREATURE TYPES ///


	// SENSE LOCATION & ORIENTATION
	// get orientation:
	glm::quat& oq = o.orientation;
	// derive from orientation:
	glm::vec3 up = quat_uy(oq);
	glm::vec3 uf = quat_uf(oq);
	glm::vec3 ux = quat_ux(oq);
	// go slower when moving uphill? 
	// positive value means going uphill, range is -1 to 1
	//float downhill = glm::dot(uf, glm::vec3(0.f,1.f,0.f)); 
	float uphill = uf.y;  
	float downhill = -uphill;

	// get norm'd coordinate:
	glm::vec3 norm = transform(world2field, o.location);
	glm::vec2 norm2 = glm::vec2(norm.x, norm.z);


	// look ahead to where we will be in 1 sim frame
	float lookahead_m = glm::length(o.velocity) * dt; 
	glm::vec3 ahead_pos = o.location + uf * lookahead_m;
	glm::vec3 norm_ahead = transform(world2field, ahead_pos);
	glm::vec2 norm_ahead2 = glm::vec2(norm_ahead.x, norm_ahead.z);

	

	// SENSE LAND
	// get a normal for the land:
	//glm::vec3 land_normal = sdf_field_normal4(sdf_dim, distance, norm, 0.05f/SDF_DIM);

	auto landpt = al_field2d_readnorm_interp(land_dim2, land, norm2);
	auto landpt_ahead = al_field2d_readnorm_interp(land_dim2, land, norm_ahead2);
	glm::vec3 land_normal = safe_normalize(glm::vec3(landpt));
	glm::vec3 land_normal_ahead = safe_normalize(glm::vec3(landpt_ahead));



	// 0..1 factors:
	//float flatness = fabsf(glm::dot(land_normal, glm::vec3(0.f,1.f,0.f)));
	float flatness = fabsf(land_normal.y);
	float flatness_ahead = fabsf(land_normal_ahead.y);
	float steepness = 1.f-flatness;
	// a direction along which the ground is horizontal (contour)
	//glm::vec3 flataxis = safe_normalize(glm::cross(land_normal, glm::vec3(0.f,1.f,0.f)));
	glm::vec3 flataxis = safe_normalize(glm::vec3(-land_normal.z, 0.f, land_normal.x));
	// get my distance from the ground:
	float sdist; // creature's distance above the ground (or negative if below)
	al_field3d_readnorm_interp(sdf_dim, distance, norm, &sdist);

	// SENSE FLUID
	// get fluid flow:
	//glm::vec3 flow;
	//fluid.velocities.front().readnorm(norm, &flow.x);
	glm::vec3 fluid = al_field3d_readnorm_interp(field_dim, fluid_velocities.front(), norm);
	// convert to meters per second:
	// (why is this needed? shouldn't it be m/s already?)
	fluid *= idt;

	// NEIGHBOUR SEARCH:
	// how far in the future want to focus attention?
	// maximum number of agents a spatial query can return
	const int NEIGHBOURS_MAX = 8;
	// see who's around:
	std::vector<int32_t> neighbours;
	float agent_range_of_view = o.scale * 10.f;
	float agent_personal_space = o.scale * 1.f;
	float field_of_view = -0.5f; // in -1..1
	// TODO: figure out why the non-toroidal mode isn't working
	int nres = hashspace.query(neighbours, NEIGHBOURS_MAX, glm::vec2(ahead_pos.x, ahead_pos.z), o.idx, agent_range_of_view, 0.f, true);

	// initialize for navigation
	// zero out by default:
	// TODO: or decay?
	o.rot_vel = glm::quat();

	// set my speed in meters per second:
	float speed = o.scale * creature_speed 
			// modify by params:
			* (0.5f + o.params.x)
			//* (1. + downhill*0.5f)		// tends to make them stay downhill
			// * (1.f + 0.1f*rnd::bi()) 
			// * glm::min(1.f, a.health * 10.f)
			// * daylight_factor*3.f
			;
	
	

	switch (o.type) {
	case Creature::TYPE_ANT: {

		speed *= ant_speed;

		// add some field stuff:
		glm::vec3 chem = food_color * o.ant.food
			+ nest_color * o.ant.nestness;
		// add to land, add to emission:
		al_field2d_addnorm_interp(fungus_dim, chemical_field.front(), norm2, chem * dt);
		al_field3d_addnorm_interp(field_dim, emission_field.back(), norm, chem * dt * emission_scale);

		auto nest = island_centres[o.ant.nest_idx];
		// cheat
		nest.y = o.location.y;
		glm::vec3 nest_relative = nest - o.location;
		float nestdist = glm::length(nest_relative) - ant_nestsize;

		//-- my own food decays too:
		o.ant.food *= ant_phero_decay;
		o.ant.nestness *= ant_phero_decay;


		// get antenna locations:
		glm::vec3 a1 = o.location + ant_sensor_size * o.scale * (uf + ux);
		glm::vec3 a2 = o.location + ant_sensor_size * o.scale * (uf - ux);
		// look for nest if we have food
		// or if we lack health
		if (o.ant.food > ant_food_min || o.health < 0.5) {
			//-- are we there yet?
			if (nestdist < 0.) {
				// TODO sounds.ant_nest(a)	
				// drop food and search again:
				//nestfood = nestfood + a.food
				o.ant.food = 0;
				o.ant.nestness = 1;
				o.health = o.ant.food;
			} else {
				//-- look for nest:
				auto normp1 = transform(world2field, a1);
				auto normp2 = transform(world2field, a2);
				float p1 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp1.x, normp1.z)).z;
				float p2 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp2.x, normp2.z)).z;
				ant_sniff_turn(o, p1, p2);
			}
		} else {
			// look for food (blood)
			float f = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), norm2).x;
			if (f > ant_food_min) {
				//sounds.ant_food(a)	
				// remove it:
				f = glm::clamp(f, 0.f, 1.f);
				al_field2d_addnorm_interp(fungus_dim, chemical_field.back(), norm2, glm::vec3(-f, 0.f, 0.f));
				o.ant.food += f;
				//a.vel = -a.vel;
				o.health = 1;
			
			} else {
				// look for food:
				auto normp1 = transform(world2field, a1);
				auto normp2 = transform(world2field, a2);
				float p1 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp1.x, normp1.z)).y;
				float p2 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp2.x, normp2.z)).y;
				
				//DPRINT("%f %f", p1, p2);
				
				ant_sniff_turn(o, p1, p2);
			}

			o.color = chem;
		}

	} break;
	case Creature::TYPE_BOID: {

		// SENSE FUNGUS
		size_t fungus_idx = al_field2d_index_norm(fungus_dim, norm2);
		//float fungal = fungus_field.front()[fungus_idx];
		float fungal = al_field2d_readnorm_interp(fungus_dim, fungus_field.front(), norm2);
		//if(i == objectSel) console.log("fungal %f", fungal);
		float eat = glm::max(0.f, fungal) * fungus_to_boid_transfer;
		//al_field2d_addnorm_interp(fungus_dim, fungus_field.front(), norm2, -eat);
  		fungus_field.front()[fungus_idx] -= eat;

		o.color = glm::vec3(o.params) * 0.75f + 0.25f;

		//o.color = glm::vec3(0, 1, 0);
		
	} break;
	case Creature::TYPE_BUG: {

	} break;
	case Creature::TYPE_PREDATOR_HEAD: {
		speed = speed * 2.f;

	} break;
	case Creature::TYPE_PREDATOR_BODY: {

	} break;
	} // end switch

	glm::vec3 copy;
	int copycount = 0;
	glm::vec3 center;
	int centrecount = 0;
	glm::vec3 avoid;
	int avoidcount = 0;
	//for (auto j : neighbours) {
	for (int j=0; j<nres; j++) {
		auto& n = creatures[neighbours[j]];
		// its future location
		auto nfut = n.location + n.scale * n.velocity*dt;

		// get relative vector from a to n:
		glm::vec3 rel = nfut - ahead_pos; //o.location;
		float cdistance = glm::length(rel);
		// skip if we are right on top of each other -- no point
		if (cdistance < 0.0001f || cdistance > agent_range_of_view) continue;

		//o.color = glm::vec3(0, 0, 1);

		// normalize relative vector:
		glm::vec3 nrel = rel / cdistance;
	
		// get distance between bodies (never be negative)
		float distance = glm::max(cdistance - o.scale - n.scale, 0.f);

		// skip if not in my field of view
		float similarity = glm::dot(uf, nrel);
		if (similarity < field_of_view) continue;

		// copy song (params)
		o.params = glm::mix(o.params, n.params, creature_song_copy_factor*dt);
		
		// if too close, stop moving:
		if (distance > 0.0000001f 
			&& distance < agent_personal_space
			&& (o.type != Creature::TYPE_PREDATOR_HEAD || n.type == Creature::TYPE_PREDATOR_HEAD)) {
			// add an avoidance force:
			//float mag = 1. - (distance / agent_personal_space);
			//float avoid_factor = -0.5;

			// factor is 0 at agent_personal_space
			// factor is massive when distance is near zero
			float avoid_factor = ((agent_personal_space/distance) - 1.f);
			avoid -= rel  * avoid_factor;
			avoidcount++;
			//avoid += rel * (mag * avoid_factor);
			//o.velocity *= glm::vec3(0.5f);
			//o.velocity = limit(o.velocity, distance);
		} else {
			// not too near, so use centering force

			// accumulate centres:
			center += rel;
			centrecount++;

			copy += n.velocity;
			copycount++;
		}
	}

	// adjust velocity according to neighbourhood
	// actually velocity is just a function of speed & uf
	// so need to adjust direction
	// (though, could throttle speed to avoid collision)

	//glm::vec3(0,1,0);
	// turn away from lowlands:
	
	if (o.location.y < coastline_height) {
		// instant death
		o.health = -1.;
	} else if (flatness_ahead < creature_flatness_min) {
		// turn away to the nearest flat axis
		// get a very smooth normal:
		float smooth = 0.5f;
		
		// the normal tells us which way we don't want to go
		auto ln = land_normal_ahead;
		
		auto desired_dir = safe_normalize(glm::vec3(-ln.x, 0.f, -ln.z));
		auto q = get_forward_rotation_to(o.orientation, desired_dir);
		o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.5f);


	} else if (o.location.y < coastline_height*2.) {

		// get a very smooth normal:
		float smooth = 0.5f;
		glm::vec3 ln = sdf_field_normal4(sdf_dim, distance, norm, 0.125f/SDF_DIM);

		auto desired_dir = safe_normalize(glm::vec3(-ln.x, 0.f, -ln.z));
		auto q = get_forward_rotation_to(o.orientation, desired_dir);
		o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.5f);
		//o.color = glm::vec3(1,0,0);

	} else if (avoidcount > 0) {
		auto desired_dir = safe_normalize(avoid);
		//desired_dir = quat_unrotate(o.orientation, desired_dir);

		auto q = get_forward_rotation_to(o.orientation, desired_dir);
		o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.25f);
		
	} else {

		switch (o.type) {
		case Creature::TYPE_ANT: {
			
			// TODO:
			// burrow down near nest:
			//h += 0.2*AL_MIN(0, nestdist);
			{
				float range = M_PI * steepness * M_PI;
				glm::quat wander = glm::angleAxis(glm::linearRand(-range, range), up);
				float wander_factor = 0.5f * dt;
				o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, wander, wander_factor));
			}

		} break;
		case Creature::TYPE_BOID: {

			
			if (copycount > 0) {
				// average neighbour velocity
				copy /= float(copycount);

				// split into speed & direction
				float desired_speed = glm::length(copy);

				auto desired_dir = safe_normalize(copy);

				auto q = get_forward_rotation_to(o.orientation, desired_dir);
				o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.125f);
				
				// // get disparity
				// auto influence_diff = desired_dir - uf;
				// float diff_mag = glm::length(influence_diff);
				// if (diff_mag > 0.001f && desired_speed > 0.001f) {
				// 	// try to match speed:
				// 	speed = glm::min(speed, desired_speed);
				// 	// try to match orientation:

				// }
			}
			if (centrecount > 0) {

				center /= float(centrecount);

				auto desired_dir = safe_normalize(center);

				// if centre is too close, rotate away from it?
				// centre is relative to self, but not rotated
				auto q = get_forward_rotation_to(o.orientation, desired_dir);
				o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.125f);
				
			} else {
				float range = M_PI * steepness * M_PI;
				glm::quat wander = glm::angleAxis(glm::linearRand(-range, range), up);
				float wander_factor = 0.15f * dt;
				o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, wander, wander_factor));
			}
		} break;
		case Creature::TYPE_BUG: {

		} break;
		case Creature::TYPE_PREDATOR_HEAD: {

			// speed varies with hunger
			// can we eat?
			for (int j=0; j<nres; j++) {
				auto& n = creatures[neighbours[j]];
				auto rel = n.location - o.location;
				float dist = glm::length(rel) - o.scale - n.scale;
				if (dist < o.scale * predator_eat_range) {


					// jump:
					o.location += rel * 0.5f;
					float e = n.health;
					o.health = AL_MIN(2.f, o.health + e);
					n.health = -0.5f;

					o.pred_head.victim = -1;
					break;
				}
			}

			if (rnd::uni() * 0.2f < dt) {
				// change target:
				o.pred_head.victim = -1;

				if (nres) {
					auto j = neighbours[0];
					Creature& n = creatures[j];
					if (n.type != Creature::TYPE_PREDATOR_HEAD
						&& n.type != Creature::TYPE_PREDATOR_BODY) {
						o.pred_head.victim = j;
						}
				}
			}

			// follow target:
			if (o.pred_head.victim >= 0) {
				Creature& n = creatures[o.pred_head.victim];
				auto rel = n.location - o.location;

				auto desired_dir = safe_normalize(rel);
				// if centre is too close, rotate away from it?
				// centre is relative to self, but not rotated
				auto q = get_forward_rotation_to(o.orientation, desired_dir);
				//o.orientation = glm::slerp(o.orientation, q * o.orientation, .125f);
				o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, q, 0.25f));
			} else {
				// wander
				float range = M_PI;
				glm::quat wander = glm::angleAxis(glm::linearRand(-range, range), up);
				float wander_factor = dt;
				//o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, wander, wander_factor));

			}


			o.color = glm::mix(glm::vec3(0.7), glm::vec3(1, 0.5, 0), o.health);

		} break;
		case Creature::TYPE_PREDATOR_BODY: {

		} break;
		} // end switch
		
	}

	// velocity
	o.velocity = speed * glm::normalize(uf);

	// let song evolve:
	o.params = wrap(o.params + glm::linearRand(glm::vec4(-creature_song_mutate_rate*dt), glm::vec4(creature_song_mutate_rate*dt)), 1.f);
	//o.color = glm::vec3(o.params);

	// float gravity = 2.0f;
	// o.accel.y -= gravity; //glm::mix(o.accel.y, newrise, 0.04f);
	// if (sdist < (o.scale * 0.025f)) { //(o.scale * rnd::uni(2.f))) {
	// 	// jump!
	// 	float jump = rnd::uni();
	// 	o.accel.y = jump * gravity * 2.f * o.scale;

		
	// }

	
	// add my direction to the fluid current
	//glm::vec3 push = quat_uf(o.orientation) * (creature_fluid_push * (float)dt);
	glm::vec3 push = o.velocity * (creature_fluid_push * (float)dt);
	//fluid.velocities.front().addnorm(norm, &push.x);
	al_field3d_addnorm_interp(field_dim, fluid_velocities.front(), norm, push);

	
}

void onUnloadGPU() {
	// free resources:
	landShader.dest_closing();
	landMeshShader.dest_closing();
	humanMeshShader.dest_closing();
	particleShader.dest_closing();
	objectShader.dest_closing();
	creatureShader.dest_closing();
	deferShader.dest_closing();
	deferShader2.dest_closing();
	simpleShader.dest_closing();
	debugShader.dest_closing();
	flowShader.dest_closing();
	blendShader.dest_closing();

	quadMesh.dest_closing();
	cubeVBO.dest_closing();
	creaturePartsVBO.dest_closing();
	creatureVAO.dest_closing();
	particlesVAO.dest_closing();
	debugVAO.dest_closing();

	fluidTex.dest_closing();
	emissionTex.dest_closing();
	distanceTex.dest_closing();
	fungusTex.dest_closing();
	noiseTex.dest_closing();
	landTex.dest_closing();
	flowTex.dest_closing();
	humanTex.dest_closing();

	texDraw.dest_closing();

	projectors[0].fbo.dest_closing();
	projectors[1].fbo.dest_closing();
	projectors[2].fbo.dest_closing();

	gBufferVR.dest_closing();
	gBufferProj.dest_closing();
	Alice::Instance().hmd->dest_closing();
	slowFbo.dest_closing();

	if (colorTex) {
		glDeleteTextures(1, &colorTex);
		colorTex = 0;
	}

	
}

void onReloadGPU() {

	onUnloadGPU();

	simpleShader.readFiles("simple.vert.glsl", "simple.frag.glsl");
	objectShader.readFiles("object.vert.glsl", "object.frag.glsl");
	//segmentShader.readFiles("segment.vert.glsl", "segment.frag.glsl");
	creatureShader.readFiles("creature.vert.glsl", "creature.frag.glsl");
	particleShader.readFiles("particle.vert.glsl", "particle.frag.glsl");
	landShader.readFiles("land.vert.glsl", "land.frag.glsl");
	humanMeshShader.readFiles("hmesh.vert.glsl", "hmesh.frag.glsl");
	landMeshShader.readFiles("lmesh.vert.glsl", "lmesh.frag.glsl");
	deferShader.readFiles("defer.vert.glsl", "defer.frag.glsl");
	deferShader2.readFiles("defer2.vert.glsl", "defer2.frag.glsl");
	debugShader.readFiles("debug.vert.glsl", "debug.frag.glsl");
	flowShader.readFiles("flow.vert.glsl", "flow.frag.glsl");
	blendShader.readFiles("blend.vert.glsl", "blend.frag.glsl");
	
	quadMesh.dest_changed();
	

	for (int i=0; i<NUM_PROJECTORS; i++) {
		if (!projectors[i].fbo.dest_changed()) {
			console.error("FBO attachement error for projector %d", i);
		} else {
			projectors[i].fbo.begin(); 
			glClearColor(0.f, 0.f, 0.f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			projectors[i].fbo.end(); 
		}
	}
	{
		if (!slowFbo.dest_changed()) {
			console.error("FBO attachement error for slowFbo");
		} else {
			slowFbo.begin(); 
			glClearColor(0.f, 0.f, 0.f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			slowFbo.end(); 
		}
	}

	tableVAO.bind();
	tableVBO.bind();
	tableVBO.submit(&tableObj.vertices[0], sizeof(Vertex) * tableObj.vertices.size());
	tableEBO.submit(&tableObj.indices[0], tableObj.indices.size());
	tableEBO.bind();
	tableVAO.attr(0, &Vertex::position);
	tableVAO.attr(1, &Vertex::normal);
	tableVAO.attr(2, &Vertex::texcoord);

	gridVAO.bind();
	{
		const int dim = LAND_DIM+1;
		gridVertices.resize(dim*dim);
		
		const glm::vec3 normalizer = 1.f/glm::vec3(dim, 1.f, dim);
		const glm::vec2 pixelcenter = glm::vec2(0.5f / (dim-1.f), 0.5f  / (dim-1.f));

		for (int i=0, y=0; y<dim; y++) {
			for (int x=0; x<dim; x++) {
				Vertex& v = gridVertices[i++];
				v.position = glm::vec3(x, 0, y) * normalizer;
				v.normal = glm::vec3(0, 1, 0);
				// depends whether wrapping or not, divide dim or dim+1?
				v.texcoord = glm::vec2(v.position.x, v.position.z) + pixelcenter;
			}
		}
		gridVBO.submit((void *)&gridVertices[0], sizeof(Vertex) * gridVertices.size());

		/*
			e.g.: 2x2 squares:
			0 1 2
			3 4 5
			6 7 8

			dim = 3x3 vertices
			elements = 2x2 squares * 2 tris * 3 verts:
			034 410, 145 521
			367 743, 478 854
		*/
		grid_elements = (dim-1) * (dim-1) * 6;
		gridElements.resize(grid_elements);
		int i=0;
		for (unsigned int y=0; y<dim-1; y++) {
			for (unsigned int x=0; x<dim-1; x++) {
				unsigned int p00 = (y*dim) + (x);
				unsigned int p01 = p00 + 1;
				unsigned int p10 = p00 + dim;
				unsigned int p11 = p00 + dim + 1;
				// tri 1:
				gridElements[i++] = p00;
				gridElements[i++] = p10;
				gridElements[i++] = p11;
				// tri 2:
				gridElements[i++] = p11;
				gridElements[i++] = p01;
				gridElements[i++] = p00;
			}
		}
		gridEBO.submit(&gridElements[0], gridElements.size());
	}

	gridVBO.bind();
	gridEBO.bind();
	gridVAO.attr(0, &Vertex::position);
	gridVAO.attr(1, &Vertex::normal);
	gridVAO.attr(2, &Vertex::texcoord);

	creatureVAO.bind();
	cubeVBO.bind();
	creatureVAO.attr(0, 3, GL_FLOAT, sizeof(glm::vec3), 0);
	creaturePartsVBO.bind();
	creatureVAO.attr(2, &CreaturePart::location, true);
	creatureVAO.attr(3, &CreaturePart::orientation, true);
	creatureVAO.attr(4, &CreaturePart::scale, true);
	creatureVAO.attr(5, &CreaturePart::phase, true);
	creatureVAO.attr(6, &CreaturePart::color, true);
	creatureVAO.attr(7, &CreaturePart::params, true);
	creatureVAO.attr(8, &CreaturePart::id, true);

	particlesVAO.bind();
	particlesVBO.bind();
	particlesVAO.attr(0, &Particle::location);
	particlesVAO.attr(1, &Particle::color);

	debugVAO.bind();
	debugVBO.bind();
	debugVAO.attr(0, &DebugDot::location);
	debugVAO.attr(1, &DebugDot::size);
	debugVAO.attr(2, &DebugDot::color);

	landTex.wrap = GL_CLAMP_TO_EDGE;
	flowTex.wrap = GL_CLAMP_TO_EDGE;
	humanTex.wrap = GL_CLAMP_TO_EDGE;
	distanceTex.wrap = GL_CLAMP_TO_EDGE;
	fungusTex.wrap = GL_CLAMP_TO_EDGE;

	{
		glGenTextures(1, &colorTex);
		glBindTexture(GL_TEXTURE_2D, colorTex);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);	
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);  
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);  
		//glTexParameteri( GL_TEXTURE_3D, GL_GENERATE_MIPMAP, GL_TRUE ); 
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	gBufferVR.dest_changed();
	gBufferProj.dest_changed();
	
	Alice::Instance().hmd->dest_changed();

}

void draw_scene(int width, int height, Projector& projector) {
	double t = Alice::Instance().simTime;
	//console.log("%f", t);

	flowTex.bind(1);
	humanTex.bind(2);
	noiseTex.bind(3);
	distanceTex.bind(4);
	fungusTex.bind(5);
	landTex.bind(6);
	fluidTex.bind(7);

	if (0) {
		simpleShader.use();
		simpleShader.uniform("uViewProjectionMatrix", viewProjMat);
		tableVAO.drawElements(tableObj.indices.size());
	}

	if (enablers[SHOW_LANDMESH]) {
		landMeshShader.use();
		landMeshShader.uniform("uViewProjectionMatrix", viewProjMat);
		landMeshShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		landMeshShader.uniform("uLandMatrix", state->world2field);
		landMeshShader.uniform("uLandMatrixInverse", state->field2world);
		landMeshShader.uniform("uWorld2Map", glm::mat4(1.f));
		landMeshShader.uniform("uLandLoD", 1.5f);
		landMeshShader.uniform("uMapScale", 1.f);
		landMeshShader.uniform("uHumanTex", 2);
		landMeshShader.uniform("uNoiseTex", 3);
		landMeshShader.uniform("uDistanceTex", 4);
		landMeshShader.uniform("uFungusTex", 5);
		landMeshShader.uniform("uLandTex", 6);
		landMeshShader.uniform("uTime", (float)t);

		if (enablers[SHOW_AS_GRID]) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		gridVAO.drawElements(grid_elements);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	if (enablers[SHOW_HUMANMESH]) {
		humanMeshShader.use();
		humanMeshShader.uniform("uViewProjectionMatrix", viewProjMat);
		humanMeshShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		humanMeshShader.uniform("uLandMatrix", state->world2field);
		humanMeshShader.uniform("uLandMatrixInverse", state->field2world);
		humanMeshShader.uniform("uWorld2Map", glm::mat4(1.f));
		humanMeshShader.uniform("uLandLoD", 1.5f);
		humanMeshShader.uniform("uMapScale", 1.f);
		humanMeshShader.uniform("uNoiseTex", 3);
		humanMeshShader.uniform("uDistanceTex", 4);
		humanMeshShader.uniform("uLandTex", 6);
		humanMeshShader.uniform("uHumanTex", 2);

		if (enablers[SHOW_AS_GRID]) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		gridVAO.drawElements(grid_elements);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	//Mini
	if (enablers[SHOW_MINIMAP]) {
		landMeshShader.use();
		landMeshShader.uniform("uViewProjectionMatrix", viewProjMat);
		landMeshShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		landMeshShader.uniform("uLandMatrix", state->world2field);
		landMeshShader.uniform("uLandMatrixInverse", state->field2world);
		landMeshShader.uniform("uWorld2Map", state->world2minimap);
		landMeshShader.uniform("uMapScale", state->minimapScale);
		landMeshShader.uniform("uDistanceTex", 4);
		landMeshShader.uniform("uFungusTex", 5);
		landMeshShader.uniform("uLandTex", 6);

		if (enablers[SHOW_AS_GRID]) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		gridVAO.drawElements(grid_elements);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	if (0) {
		landShader.use();
		landShader.uniform("time", t);
		landShader.uniform("uViewProjectionMatrix", viewProjMat);
		landShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		landShader.uniform("uNearClip", projector.near_clip);
		landShader.uniform("uFarClip", projector.far_clip);
		landShader.uniform("uDistanceTex", 4);
		landShader.uniform("uFungusTex", 5);
		landShader.uniform("uLandTex", 6);
		landShader.uniform("uLandMatrix", state->world2field);
		quadMesh.draw();
	}

	noiseTex.unbind(3);
	distanceTex.unbind(4);
	fungusTex.unbind(5);
	landTex.unbind(6);

	if (enablers[SHOW_OBJECTS]) {

		if (enablers[USE_OBJECT_SHADER]) {
			objectShader.use();
			objectShader.uniform("time", t);
			objectShader.uniform("uViewMatrix", viewMat);
			objectShader.uniform("uViewProjectionMatrix", viewProjMat);
			objectShader.uniform("uFluidTex", 7);
			objectShader.uniform("uFluidMatrix", state->world2field);
		} else {
			creatureShader.use();
			creatureShader.uniform("time", t);
			creatureShader.uniform("uViewMatrix", viewMat);
			creatureShader.uniform("uViewProjectionMatrix", viewProjMat);
			creatureShader.uniform("uFluidTex", 7);
			creatureShader.uniform("uFluidMatrix", state->world2field);
		}
		creatureVAO.drawInstanced(sizeof(positions_cube) / sizeof(glm::vec3), rendercreaturecount);
	}

	if (enablers[SHOW_PARTICLES]) {
		particleShader.use(); 
		particleShader.uniform("time", t);
		particleShader.uniform("uViewMatrix", viewMat);
		particleShader.uniform("uViewMatrixInverse", viewMatInverse);
		particleShader.uniform("uProjectionMatrix", projMat);
		particleShader.uniform("uViewProjectionMatrix", viewProjMat);
		particleShader.uniform("uViewPortHeight", (float)height);
		particleShader.uniform("uPointSize", state->particleSize);
		particleShader.uniform("uColorTex", 0);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, colorTex);
		glEnable( GL_PROGRAM_POINT_SIZE );
		glEnable(GL_POINT_SPRITE);
		glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
		particlesVAO.draw(NUM_PARTICLES, GL_POINTS);
		glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glDisable(GL_POINT_SPRITE);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	if (enablers[SHOW_DEBUGDOTS]) {

		auto dot = state->debugdots[NUM_DEBUGDOTS/3];
		//console.log("%f %f %f", dot.location.x, dot.location.y, dot.location.z);

		debugShader.use(); 
		debugShader.uniform("uViewMatrix", viewMat);
		debugShader.uniform("uViewMatrixInverse", viewMatInverse);
		debugShader.uniform("uProjectionMatrix", projMat);
		debugShader.uniform("uViewProjectionMatrix", viewProjMat);
		debugShader.uniform("uViewPortHeight", (float)height);
		debugShader.uniform("uColorTex", 0);


		glBindTexture(GL_TEXTURE_2D, colorTex);
		glEnable( GL_PROGRAM_POINT_SIZE );
		glEnable(GL_POINT_SPRITE);
		glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
		debugVAO.draw(NUM_DEBUGDOTS, GL_POINTS);
		glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glDisable(GL_POINT_SPRITE);
		glBindTexture(GL_TEXTURE_2D, 0);

		glDisable(GL_CULL_FACE);
	}

	glDisable(GL_CULL_FACE);
}

void draw_gbuffer(SimpleFBO& fbo, GBuffer& gbuffer, Projector& projector, bool isVR, glm::vec2 viewport_scale=glm::vec2(1.f), glm::vec2 viewport_offset=glm::vec2(0.f), bool blend=false) {

	fbo.begin();
	glScissor(
		fbo.dim.x*viewport_offset.x, 
		fbo.dim.y*viewport_offset.y, 
		fbo.dim.x*viewport_scale.x, 
		fbo.dim.y*viewport_scale.y);
	glViewport(
		fbo.dim.x*viewport_offset.x, 
		fbo.dim.y*viewport_offset.y, 
		fbo.dim.x*viewport_scale.x, 
		fbo.dim.y*viewport_scale.y);

	if (blend) {
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendEquation(GL_FUNC_ADD);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	} else {
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.f, 0.f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	{
		gbuffer.bindTextures(); // 0,1,2
		distanceTex.bind(4);
		fungusTex.bind(5);
		emissionTex.bind(6);
		fluidTex.bind(7);

		// altshader == true for the top-down projectors
		Shader& shader = projector.altShader ? deferShader2 : deferShader;

		shader.use();
		shader.uniform("gColor", 0);
		shader.uniform("gNormal", 1);
		shader.uniform("gPosition", 2);
		shader.uniform("gTexCoord", 3);
		shader.uniform("uDistanceTex", 4);
		shader.uniform("uFungusTex", 5);
		shader.uniform("uEmissionTex", 6);
		shader.uniform("uFluidTex", 7);

		shader.uniform("uViewMatrix", viewMat);
		shader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		shader.uniform("uFluidMatrix", state->world2field);
		shader.uniform("uNearClip", projector.near_clip);
		shader.uniform("uFarClip", projector.far_clip);
		
		shader.uniform("time", Alice::Instance().simTime);
		shader.uniform("uDim", glm::vec2(gbuffer.dim.x, gbuffer.dim.y));
		shader.uniform("uTexScale", viewport_scale);
		shader.uniform("uTexOffset", viewport_offset);

		shader.uniform("uFade", isVR ? state->vrFade : 0.f);
		
		quadMesh.draw();

		shader.unuse();
		
		distanceTex.unbind(4);
		fungusTex.unbind(5);
		emissionTex.unbind(6);
		fluidTex.unbind(7);
		gbuffer.unbindTextures();
	}
	if (blend) {
		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
	} else {
		
	}
	fbo.end();
}

void State::animate(float dt) {
	// keep the computation in here to absolute minimum
	// since it detracts from frame rate
	// here we should only be extrapolating raw visible features
	// such as location, orientation
	// that would otherwise look jerky when animated from the simulation thread

	// reset how many creature parts we will render:
	rendercreaturecount = 0;
	livingcreaturecount = 0;
	numants = 0;
	numboids = 0;

	for (int i=0; i<NUM_PARTICLES; i++) {
		Particle &o = particles[i];
		o.location = o.location + o.velocity * dt;
		//o.location = glm::clamp(o.location, world_min, world_max);
	}

	for (int i=0; i<NUM_CREATURES; i++) {
		auto &o = creatures[i];
		
		if (o.state == Creature::STATE_ALIVE) {
			livingcreaturecount++;

			switch(o.type) {
				case Creature::TYPE_ANT: numants++; break;
				case Creature::TYPE_BOID: numboids++; break;
			}

			// update location
			glm::vec3 p1 = wrap(o.location + o.velocity * dt, world_min, world_max);

			// get norm'd coordinate:
			glm::vec3 norm = transform(world2field, p1);
			glm::vec2 norm2 = glm::vec2(norm.x, norm.z);

			// stick to land surface:
			auto landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), land, norm2);
			p1 = transform(field2world, glm::vec3(norm.x, landpt.w, norm.z));

			float distance = glm::length(p1 - o.location);
			o.location = p1;
			
			// apply change of orientation here too
			// slerping by dt is a close approximation to rotation in radians per second
			o.orientation = safe_normalize(glm::slerp(o.orientation, o.rot_vel * o.orientation, dt * 1.f));

			// re-align the creature to the surface normal:
			glm::vec3 land_normal = glm::vec3(landpt);
			{
				// re-orient relative to ground:
				float creature_land_orient_factor = 1.f;//0.25f;
				o.orientation = safe_normalize(glm::slerp(o.orientation, align_up_to(o.orientation, land_normal), creature_land_orient_factor));
			}

			o.phase += distance;
		}
		if (o.state != Creature::STATE_BARDO) {
			// copy data into the creatureparts:
			CreaturePart& part = creatureparts[rendercreaturecount];
			part.id = o.type;//i;
			part.location = o.location;
			part.scale = o.scale;
			part.orientation = o.orientation;
			part.color = o.color;
			part.phase = o.phase;
			part.params = o.params;
			rendercreaturecount++;
		}
	}
}

void onFrame(uint32_t width, uint32_t height) {
	profiler.reset();
	


	Alice& alice = Alice::Instance();
	double t = alice.simTime;
	float dt = alice.fps.dt;
	float aspect = gBufferVR.dim.x / (float)gBufferVR.dim.y;
	CloudDevice& kinect0 = alice.cloudDeviceManager.devices[0];
	CloudDevice& kinect1 = alice.cloudDeviceManager.devices[1];

	if (alice.simTime > 10.) {
		state->land_rise_rate = 0.03f;
	}

	#ifdef AL_WIN
	if (alice.fps.count == 30 && !alice.window.isFullScreen) {
		//alice.window.fullScreen(true);
		//alice.goFullScreen = true;
	}
	#endif


	if (1) {

		// keep vr location above ground
		auto norm = transform(state->world2field, vrLocation);
		auto norm2 = glm::vec2(norm.x, norm.z);
		auto landpt = al_field2d_readnorm_interp(land_dim2, state->land, norm2);
		float newy = state->field2world_scale * landpt.w;
		vrLocation.y = glm::mix(vrLocation.y, newy, 0.25f);
		
		//console.log("vrLocation %f %f %f == %f", vrLocation.x, vrLocation.y, vrLocation.z, landpt.w);

		timeToVrJump -= dt; 
		

		//Teleport Fade
		if (fadeState == -1) {
			state->vrFade += dt * 2.f;
			if (state->vrFade >= 1) {
				state->vrFade = 1;
				vrLocation = nextVrLocation;
				cameraLoc = nextVrLocation;
				fadeState = 1;
			}
		} else if (fadeState == 1) {
			state->vrFade -= dt * 2.f;
			if (state->vrFade <= 0) {
				fadeState = 0;
				state->vrFade = 0;
			}
		}
	}

	if (1) {
		// later, figure out how to place teleport points in viable locations
		

		for (int i=0; i<NUM_TELEPORT_POINTS; i++ ) {
			state->debugdots[i].location = transform(state->world2minimap, state->teleport_points[i]);
			state->debugdots[i].color = glm::vec3(0,1,0);
		}
		
		if (alice.leap->isConnected) {
			//console.log("leap connected!");
			// copy bones into debugdots
			glm::mat4 trans = viewMatInverse * state->leap2view;

			int num_ray_dots = 64;
			int num_hand_dots = 5*4;

			glm::vec3 mapPos = vrLocation + glm::vec3(0., 1., 0.);
			glm::vec3 head2map = mapPos - headPos;

			glm::vec2 head2map_horiz = glm::vec2(head2map.x, head2map.z);
			float dist2map_squared = glm::dot(head2map_horiz, head2map_horiz);

			float m = 0.005f / dist2map_squared;
			//console.log("vr location y %f", vrLocation.y);

			state->minimapScale = glm::mix(state->minimapScale, m, dt*3.f);

			glm::vec3 midPoint = (state->world_min + state->world_max)/2.f;
				midPoint.y = 0;
			state->world2minimap = 
					glm::translate(glm::vec3(mapPos)) * 
					glm::scale(glm::vec3(state->minimapScale)) *
					glm::translate(-midPoint);

			for (int h=0; h<2; h++) {
		
				int d = NUM_TELEPORT_POINTS + h * (num_hand_dots + num_ray_dots);
				auto& hand = alice.leap->hands[h];

				//glm::vec3 col = (hand.id % 2) ?  glm::vec3(1, 0, hand.pinch) :  glm::vec3(0, 1, hand.pinch);
				float cf = fmod(hand.id / 6.f, 1.f);
				glm::vec3 col = glm::vec3(cf, 1.-cf, h);
				if (!hand.isVisible) {
					col = glm::vec3(0.2);
				};

				for (int f=0; f<5; f++) {
					auto& finger = hand.fingers[f];


					for (int b=0; b<4; b++) {
						auto& bone = finger.bones[b];
						auto boneloc = transform(trans, bone.center);
						if (hand.isVisible) state->debugdots[d].location = boneloc;
						state->debugdots[d].color = col;

						if (b == 3) {
							// finger tip:

							for (int i=0; i<NUM_TELEPORT_POINTS; i++ ) {
								auto maploc = transform(state->world2minimap, state->teleport_points[i]);
								float lengthToDot = glm::length(boneloc - maploc);
								if (lengthToDot < 0.02f) {
									// TELEPORT!
									nextVrLocation = state->teleport_points[i];
									fadeState = -1;
									//state->vrFade = sin(t) * 0.5 + 0.5;
								}
							}
						}
						d++;
					}
				}

				//get hand position and direction and cast ray forward until it hits land
				glm::vec3 handPos = hand.palmPos;
				glm::vec3 handDir = hand.direction;

				//glm::vec3 handNormall = glm::vec3(0.,1.,0.);

				// these are in 'leap' space, need to convert to world space
				handPos = transform(trans, handPos);
				handDir = transform(trans, handDir);

				auto a = hand.fingers[3].bones[0].center;
				auto b = hand.fingers[3].bones[1].center;

				a = transform(trans, a);
				b = transform(trans, b);


				handDir = safe_normalize(b - a);
				//handDir = safe_normalize(glm::mix(handDir, glm::vec3(0,1,0), 0.25));
				handPos = a;

				auto rotaxis = safe_normalize(glm::cross(handDir, glm::vec3(0, 1, 0)));
				auto warp = glm::rotate(-0.1f, rotaxis);

				glm::vec3 p = a;
			}
		}
	}

	{
		// update teleport points
		int i = alice.fps.count % NUM_TELEPORT_POINTS;
		glm::vec3 pt = state->teleport_points[i];
		// get slope at pt:
		glm::vec3 norm = transform(state->world2field, pt);
		glm::vec4 landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), state->land, glm::vec2(norm.x, norm.z));
		glm::vec3 normal = glm::vec3(landpt);
		normal = safe_normalize(normal);
		// move uphill:
		pt -= normal * 0.1f;
		// put on land again
		norm = transform(state->world2field, pt);
		landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), state->land, glm::vec2(norm.x, norm.z));
		pt = transform(state->field2world, glm::vec3(norm.x, landpt.w, norm.z)) ;

		state->teleport_points[i] = pt;

		//console.log("point %d %f %f %f", i, pt.x, pt.y, pt.z);
	}

	// navigation:
	{
		glm::vec3& navloc = projectors[2].location;
		glm::quat& navquat = projectors[2].orientation;	
		switch (camMode % 3){
			case 0: {
				// WASD mode:
				float camera_speed = camFast ? camera_speed_default * 3.f : camera_speed_default;
				float camera_turnangle = camFast ? camera_turn_default * 3.f : camera_turn_default;

				// move camera:
				glm::vec3 newloc = navloc + quat_rotate(navquat, camVel) * (camera_speed * dt);
				// wrap to world:
				newloc = wrap(newloc, state->world_min, state->world_max);
				navloc = glm::mix(navloc, newloc, 0.25f);

				// stick to floor:
				glm::vec3 norm = transform(state->world2field, navloc);
				glm::vec4 landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), state->land, glm::vec2(norm.x, norm.z));
				navloc = transform(state->field2world, glm::vec3(norm.x, landpt.w, norm.z)) ;
				navloc.y += 1.6f;
		
				// rotate camera:
				glm::quat newori = safe_normalize(navquat * glm::angleAxis(camera_turnangle * dt, camTurn));
				
				// now orient to floor:
				glm::vec3 up = glm::vec3(landpt);
				up = glm::mix(up, glm::vec3(0,1,0), 0.5);
				newori = align_up_to(newori, glm::normalize(up));


				navquat = glm::slerp(navquat, newori, 0.25f);
			
			} break;
			case 1: {
				// follow a creature mode:
				auto& o = state->creatures[objectSel % NUM_CREATURES];
				
				auto boom = glm::vec3(0., 1.6f, 2.f);
				
				glm::vec3 loc = o.location + quat_rotate(navquat, boom);
				loc = glm::mix(navloc, loc, 0.1f);

				// keep this above ground:
				glm::vec3 norm = transform(state->world2field, loc);
				auto landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), state->land, glm::vec2(norm.x, norm.z));
				navloc = transform(state->field2world, glm::vec3(norm.x, glm::max(norm.y, landpt.w+0.01f), norm.z));

				navloc = glm::mix(navloc, loc, 0.1f);
				navquat = glm::slerp(navquat, o.orientation, 0.03f);
			}
			default: {
				// orbit around
				float a = M_PI * t / 30.;

				glm::quat newori = glm::angleAxis(a, glm::vec3(0,1,0));
				glm::vec3 newloc = vrLocation + glm::vec3(0., 2., 0.) + (quat_uz(navquat))*30.f;
				
				navloc = glm::mix(navloc, newloc, 0.01f);
				
				// keep this above ground:
				glm::vec3 norm = transform(state->world2field, navloc);
				auto landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), state->land, glm::vec2(norm.x, norm.z));
				navloc = transform(state->field2world, glm::vec3(norm.x, glm::max(norm.y, landpt.w+0.01f), norm.z));

				navloc = glm::mix(navloc, newloc, 0.1f);
				
				navquat = glm::slerp(navquat, newori, 0.05f);
			}
		}

			
	}

	// animation
	if (alice.isSimulating && isRunning) {
		state->animate(dt);
		profiler.log("animation", alice.fps.dt);
	}

	// upload data to GPU:
	if (alice.isSimulating && isRunning) {

		
		// upload VBO data to GPU:
		creaturePartsVBO.submit(&state->creatureparts[0], sizeof(state->creatureparts));
		particlesVBO.submit(&state->particles[0], sizeof(state->particles));
		debugVBO.submit(&state->debugdots[0], sizeof(state->debugdots));
		
		// upload texture data to GPU:
		//fluidTex.submit(fluid.velocities.dim(), (glm::vec3 *)fluid.velocities.front()[0]);
		fluidTex.submit(field_dim, state->fluid_velocities.front());
		emissionTex.submit(field_dim, state->emission_field.front());
		//fungusTex.submit(glm::ivec2(FUNGUS_DIM, FUNGUS_DIM), state->fungus_field.front());
		fungusTex.submit(glm::ivec2(FUNGUS_DIM, FUNGUS_DIM), &state->field_texture[0]);
		noiseTex.submit(glm::ivec2(FUNGUS_DIM, FUNGUS_DIM), &state->noise_texture[0]);
		landTex.submit(glm::ivec2(LAND_DIM, LAND_DIM), &state->land[0]);
		humanTex.submit(glm::ivec2(LAND_DIM, LAND_DIM), &state->human.front()[0]);
		distanceTex.submit(sdf_dim, (float *)&state->distance[0]);
		flowTex.submit(glm::ivec2(512,424), &state->flow[0]);
		
		//if (alice.cloudDevice->use_colour) {
			const CloudDevice& cd = alice.cloudDeviceManager.devices[flip];
			const ColourFrame& image = cd.colourFrame();
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, colorTex);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cColorWidth, cColorHeight, 0, GL_RGB, 
			GL_UNSIGNED_BYTE, image.color);
		//}
		profiler.log("gpu upload", alice.fps.dt);
	}

	glEnable(GL_MULTISAMPLE);  

	// render the projectors:
	
	#ifdef AL_WIN
	{
		int i = alice.fps.count % 3;
	#else
	for (int i=0; i<NUM_PROJECTORS; i++) {
	#endif
		Projector& proj = projectors[i];
		SimpleFBO& fbo = proj.fbo;

		if (i == 2 && enablers[SHOW_TIMELAPSE]) {
			
			// timelapse wall projector:
			int slices = gBufferProj.dim.x;//((int(t) % 3) + 3);
			int slice = (sliceIndex++ % slices);

			if (alice.fps.count == 0 || slice == (gBufferProj.dim.x - 10)) { //timeToVrJump < 0.f) {
				vrIsland = (vrIsland + 1) % NUM_ISLANDS;
				nextVrLocation = state->teleport_points[vrIsland];
				fadeState = -1;

				timeToVrJump = 30.;
			}

			float centredslice = (slice - ((-1.f+slices)/2.f))*2.f;
			float slicewidth = 1.f/slices;
			float sliceangle = M_PI * 2./slices; 
			// 0..1
			float sliceoffset = slice / float(slices);

			// want to put the forward direction in the middle
			auto sliceRot = glm::angleAxis((centredslice/float(slices))* float(M_PI * -1.f), quat_uy(proj.orientation));
			viewMat = glm::inverse(glm::translate(proj.location) * glm::mat4_cast(sliceRot * proj.orientation));

			// slice frustum x depends on sliceangle:
			float fw = proj.near_clip * tanf(sliceangle * 0.5f);
			float f = proj.near_clip * 1.f;
			projMat = glm::frustum(-fw, fw, -f, f, proj.near_clip, proj.far_clip);

			viewProjMat = projMat * viewMat;
			projMatInverse = glm::inverse(projMat);
			viewMatInverse = glm::inverse(viewMat);
			viewProjMatInverse = glm::inverse(viewProjMat);

			// draw the scene into the GBuffer:
			glEnable(GL_SCISSOR_TEST);
			{
				glm::vec2 viewport_scale = glm::vec2(slicewidth * 2.f, 1.f);
				glm::vec2 viewport_offset = glm::vec2(sliceoffset, 0.f);

				gBufferProj.begin();
					viewport.pos = glm::ivec2(gBufferProj.dim.x * viewport_offset.x, gBufferProj.dim.y * viewport_offset.y);
					viewport.dim = glm::ivec2(gBufferProj.dim.x * viewport_scale.x + 2., gBufferProj.dim.y * viewport_scale.y);

					glScissor(
						viewport.pos.x, 
						viewport.pos.y, 
						viewport.dim.x, 
						viewport.dim.y);
					glViewport(
						viewport.pos.x, 
						viewport.pos.y, 
						viewport.dim.x, 
						viewport.dim.y);
					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
					draw_scene(viewport.dim.x, viewport.dim.y, projectors[2]);
				gBufferProj.end();

				draw_gbuffer(fbo, gBufferProj, projectors[2], false, viewport_scale, viewport_offset);

				// //glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this
				// bool useblend = true;
				// draw_gbuffer(fbo, gBufferProj, projectors[2], false, viewport_scale, viewport_offset, useblend);

				// // blend in:

				// slowFbo.begin();
				// 	glScissor(0, 0, slowFbo.dim.x, slowFbo.dim.y);
				// 	glViewport(0, 0, slowFbo.dim.x, slowFbo.dim.y);
			
				// 	glDisable(GL_DEPTH_TEST);
				// 	glEnable(GL_BLEND);
				// 	glBlendEquation(GL_FUNC_ADD);
				// 	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					
				// 	glActiveTexture(GL_TEXTURE0);
				// 	glBindTexture(GL_TEXTURE_2D, fbo.tex);
				// 	blendShader.use();
				// 	blendShader.uniform("uSourceTex", 0);
				// 	quadMesh.draw();
				// 	blendShader.unuse();
				// 	glBindTexture(GL_TEXTURE_2D, 0);
				// 	glDisable(GL_BLEND);
				// 	glEnable(GL_DEPTH_TEST);
				// slowFbo.end();
			}
			glDisable(GL_SCISSOR_TEST);
		} else {
			// regular projection

			viewMat = proj.view();
			projMat = proj.projection();

			viewProjMat = projMat * viewMat;
			projMatInverse = glm::inverse(projMat);
			viewMatInverse = glm::inverse(viewMat);
			viewProjMatInverse = glm::inverse(viewProjMat);
			
			if (0) {
				// don't use gbuffer:
				fbo.begin();

				glScissor(0, 0, fbo.dim.x, fbo.dim.y);
				glViewport(0, 0, fbo.dim.x, fbo.dim.y);
				glEnable(GL_DEPTH_TEST);
				glClearColor(0.f, 0.f, 0.f, 1.0f);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				draw_scene(fbo.dim.x, fbo.dim.y, proj);
				fbo.end();
			} else {

				// draw the scene into the GBuffer:
				glEnable(GL_SCISSOR_TEST);
				gBufferProj.begin();
					glScissor(0, 0, gBufferProj.dim.x, gBufferProj.dim.y);
					glViewport(0, 0, gBufferProj.dim.x, gBufferProj.dim.y);
					glEnable(GL_DEPTH_TEST);
					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
					draw_scene(gBufferProj.dim.x, gBufferProj.dim.y, proj);
				gBufferProj.end();
				//glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this
				draw_gbuffer(fbo, gBufferProj, proj, false, glm::vec2(1.f), glm::vec2(0.f));
				glDisable(GL_SCISSOR_TEST);
			}
		}
	}
	profiler.log("render projectors", alice.fps.dt);
	
	// render the VR viewpoint:
	Hmd& vive = *alice.hmd;
	//SimpleFBO& fbo = vive.fbo;
	if (width && height) {
		if (vive.connected) {	
				
			vive.near_clip = projectors[2].near_clip;
			vive.far_clip = projectors[2].far_clip;	
			vive.update();
			glEnable(GL_SCISSOR_TEST);

			//vrLocation = state->objects[1].location + glm::vec3(0., 1., 0.);

			// get head position in world space:
			headPos = vive.mTrackedPosition + vrLocation;

			for (int eye = 0; eye < 2; eye++) {
				// update nav
				viewMat = glm::inverse(vive.m_mat4viewEye[eye]) * glm::mat4_cast(glm::inverse(vive.mTrackedQuat)) * glm::translate(glm::mat4(1.f), -vive.mTrackedPosition) * glm::translate(-vrLocation);
				/*
				projMat = glm::frustum(vive.frustum[eye].l, vive.frustum[eye].r, vive.frustum[eye].b, vive.frustum[eye].t, vive.frustum[eye].n, vive.frustum[eye].f);
				*/

				projMat = vive.mProjMatEye[eye];

				viewProjMat = projMat * viewMat;
				projMatInverse = glm::inverse(projMat);
				viewMatInverse = glm::inverse(viewMat);
				viewProjMatInverse = glm::inverse(viewProjMat);

				glm::vec2 viewport_scale = glm::vec2(0.5f, 1.f);
				glm::vec2 viewport_offset = glm::vec2(eye*0.5f, 0.f);

				gBufferVR.begin();

					viewport.pos = glm::ivec2(gBufferVR.dim.x * viewport_offset.x, gBufferVR.dim.y * viewport_offset.y);
					viewport.dim = glm::ivec2(gBufferVR.dim.x * viewport_scale.x, gBufferVR.dim.y * viewport_scale.y);

					//console.log("eye %d vp %d %d %d %d", eye, viewport.pos.x, viewport.pos.y, viewport.dim.x, viewport.dim.y);

					glScissor(
						viewport.pos.x, 
						viewport.pos.y, 
						viewport.dim.x, 
						viewport.dim.y);
					glViewport(
						viewport.pos.x, 
						viewport.pos.y, 
						viewport.dim.x, 
						viewport.dim.y);
					glEnable(GL_DEPTH_TEST);
					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

					draw_scene(viewport.dim.x, viewport.dim.y, projectors[2]);
				gBufferVR.end();

				//glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this
				draw_gbuffer(vive.fbo, gBufferVR, projectors[2], true, viewport_scale, viewport_offset);
			}
			glDisable(GL_SCISSOR_TEST);
		} else {
			
			// regular projection
			Projector& proj = projectors[2];

			viewMat = proj.view();
			projMat = proj.projection();

			viewProjMat = projMat * viewMat;
			projMatInverse = glm::inverse(projMat);
			viewMatInverse = glm::inverse(viewMat);
			viewProjMatInverse = glm::inverse(viewProjMat);
			
			// draw the scene into the GBuffer:
			glEnable(GL_SCISSOR_TEST);
			gBufferProj.begin();
				glScissor(0, 0, gBufferProj.dim.x, gBufferProj.dim.y);
				glViewport(0, 0, gBufferProj.dim.x, gBufferProj.dim.y);
				glEnable(GL_DEPTH_TEST);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				draw_scene(gBufferProj.dim.x, gBufferProj.dim.y, proj);
			gBufferProj.end();
			//glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this
			draw_gbuffer(vive.fbo, gBufferProj, proj, false, glm::vec2(1.f), glm::vec2(0.f));
			glDisable(GL_SCISSOR_TEST);
		}
	} 
	profiler.log("render 1st person", alice.fps.dt);
		
	alice.hmd->submit();
	profiler.log("hmd submit", alice.fps.dt);

	glViewport(0, 0, width, height);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.f, 0.f, 0.f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#ifdef AL_WIN
	float third = 1.f/3.f;
	if (enablers[SHOW_TIMELAPSE]) {
		projectors[2].fbo.draw(glm::vec2(2.f*third, 0.f), glm::vec2(1.f, 1.f));
		//slowFbo.draw(glm::vec2(2.f*third, 0.f), glm::vec2(1.f, 1.f));
	} else {
		vive.fbo.draw(glm::vec2(2.f*third, 0.f), glm::vec2(1.f, 1.f));
	}
	
	projectors[1].fbo.draw(glm::vec2(third, 0.f), glm::vec2(2.f*third, 1.f));
	projectors[0].fbo.draw(glm::vec2(0.f, 0.f), glm::vec2(third, 1.f));
#else

	if (soloView) {
		switch (soloView) {
			case 1: projectors[0].fbo.draw(); break;
			case 2: projectors[1].fbo.draw(); break;
			//case 3: slowFbo.draw(); break; 
			case 3: projectors[2].fbo.draw(); break;
			case 4: {
				if (!SHOW_TIMELAPSE) {
					flowShader.use();
					flowShader.uniform("tex", 0);
					flowShader.uniform("uScale", glm::vec2(1.f));
					flowShader.uniform("uOffset", glm::vec2(0.f));
					texDraw.draw_no_shader(flowTex.id);
					flowShader.unuse();
				} else {
					vive.fbo.draw(); break;
				}
			} break;
			default: soloView = 0;
		}
	} else {
		float third = 1.f/3.f;
		float sixth = 1.f/6.f;
		projectors[0].fbo.draw(glm::vec2(0.f,  0.f),  glm::vec2(0.5f,0.5f));
		projectors[1].fbo.draw(glm::vec2(0.5f, 0.f),  glm::vec2(1.f ,0.5f));
		projectors[2].fbo.draw(glm::vec2(0.5f, 0.5f), glm::vec2(1.f ,1.f));
		//slowFbo.draw(glm::vec2(0.5f, 0.5f), glm::vec2(1.f ,1.f));
		vive.fbo.              draw(glm::vec2(0.f,  0.5f), glm::vec2(0.5f ,1.f));
	}
#endif
	profiler.log("draw to window", alice.fps.dt);

	if (showFPS) {
		console.log("fps %f(%f) at %f; fluid %f(%f) sim %f(%f) field %f(%f) land %f (%f) kinect %f %f, rendered creatures %d (alive ants %d boids %d total %d)", alice.fps.fps, alice.fps.fpsPotential, alice.simTime, fluidThread.fps.fps, fluidThread.fps.fpsPotential, simThread.fps.fps, simThread.fps.fpsPotential, fieldThread.fps.fps, fieldThread.fps.fpsPotential, landThread.fps.fps, landThread.fps.fpsPotential, kinect0.fps.fps, kinect1.fps.fps, rendercreaturecount, numants, numboids,livingcreaturecount);
		//profiler.dump();
	}
}


void onKeyEvent(int keycode, int scancode, int downup, bool shift, bool ctrl, bool alt, bool cmd){
	Alice& alice = Alice::Instance();

	switch(keycode) {
		case GLFW_KEY_0:
		case GLFW_KEY_1:
		case GLFW_KEY_2:
		case GLFW_KEY_3:
		case GLFW_KEY_4:
		case GLFW_KEY_5:
		case GLFW_KEY_6:
		case GLFW_KEY_7:
		case GLFW_KEY_8:
		case GLFW_KEY_9: {
			int num = keycode - GLFW_KEY_0;
			if (downup) {
				if (shift) {
					enablers[num] = !enablers[num];
				} else {
					soloView = (soloView != num) ? num : 0;
				}
			}
		}
		break;
		// case GLFW_KEY_ENTER: {
		// 	if (downup && alt) {
		// 		if (alice.hmd->connected) {
		// 			alice.hmd->disconnect();
		// 		} else if (alice.hmd->connect()) {
		// 			gBufferVR.dim = alice.hmd->fbo.dim;
		// 			gBufferVR.dest_changed();
		// 			alice.hmd->dest_changed();
		// 			alice.fps.setFPS(90);
		// 		}
		// 	}
		// } break;

		// ? key to switch debug modes
		case GLFW_KEY_SLASH: {
			//console.log("D was pressed");
			if (downup) debugMode++;
		} break;
		
		case GLFW_KEY_C: {
			if (downup) { 
				camMode++;
				console.log("Cam mode %d", camMode);
			}
		} break;

		case GLFW_KEY_F: {
			if(downup){
				objectSel = (objectSel + 1) % NUM_CREATURES;
			}
		} break;

		case GLFW_KEY_RIGHT_SHIFT:
		case GLFW_KEY_LEFT_SHIFT: {
			if (downup) camFast = false;
			else camFast = true;
			break;
		}

		// WASD+arrows for nav:
		case GLFW_KEY_UP:
		case GLFW_KEY_W:
			if (downup) camVel.z = -1.f; 
			else camVel.z = 0.f; 
			break;
		case GLFW_KEY_DOWN:
		case GLFW_KEY_S:
			if (downup) camVel.z =  1.f; 
			else camVel.z = 0.f; 
			break;
		case GLFW_KEY_RIGHT:
			if (downup) camTurn.y = -1.f; 
			else camTurn.y = 0.f; 
			break;
		case GLFW_KEY_LEFT:
			if (downup) camTurn.y =  1.f; 
			else camTurn.y = 0.f; 
			break;
		case GLFW_KEY_A:
			if (downup) camVel.x = -1.f; 
			else camVel.x = 0.f; 
			break;
		case GLFW_KEY_D:
			if (downup) camVel.x =  1.f; 
			else camVel.x = 0.f; 
			break;

		case GLFW_KEY_T:
			if (downup) showFPS = !showFPS;
			break;

		default:
			console.log("keycode: %d scancode: %d press: %d shift %d ctrl %d alt %d cmd %d", keycode, scancode, downup, shift, ctrl, alt, cmd);
			break;
	
	}

}


void threads_begin() {
	console.log("starting threads");
	// allow threads to run
	isRunning = true;
	simThread.begin(sim_update);
	fieldThread.begin(fields_update);
	fluidThread.begin(fluid_update);
	landThread.begin(land_update);
	console.log("started threads");
}

void threads_end() {
	// release threads:
	isRunning = false;
	console.log("ending threads");
	simThread.end();
	fieldThread.end();
	fluidThread.end();
	landThread.end();
	console.log("ended threads");
}

void onReset() {
	threads_end();
	state->reset();
	onReloadGPU();
	threads_begin();
}

// The onReset event is triggered when pressing the "Backspace" key in Alice
void State::reset() {

	// zero then invoke constructor on it:
	memset(state, 0, sizeof(State)); 
	state = new(state) State;

	// how to convert the normalized coordinates of the fluid (0..1) into positions in the world:
	// this effectively defines the bounds of the fluid in the world:
	// from transform(field2world(glm::vec3(0.)))
	// to   transform(field2world(glm::vec3(1.)))
	field2world_scale = world_max.x - world_min.x;
	world2field_scale = 1.f/field2world_scale;
	field2world = glm::scale(glm::vec3(field2world_scale));
	// how to convert world positions into normalized texture coordinates in the fluid field:
	world2field = glm::inverse(field2world);

	//vive2world = glm::rotate(float(M_PI/2), glm::vec3(0,1,0)) * glm::translate(glm::vec3(-40.f, 0.f, -30.f));
		//glm::rotate(M_PI/2., glm::vec3(0., 1., 0.));
	leap2view = glm::mat4(1.f); //glm::rotate(float(M_PI * -0.26), glm::vec3(1, 0, 0));

	/// initialize at zero so that the minimap is invisible
	world2minimap = glm::scale(glm::vec3(0.f));
	
	cameraLoc = world_centre;

	//hashspace.reset(world_min, world_max);
	hashspace.reset(glm::vec2(world_min.x, world_min.z), glm::vec2(world_max.x, world_max.z));
	dead_space.reset();

	fluid_velocities.reset();
	fluid_gradient.reset();

	creature_pool.init();

	fungus_field.reset();
	//al_field2d_add(fungus_dim, fungus_field.front(), 0.5f);
	//fungus_field.copy();


	{
		for (int i=0; i<FUNGUS_TEXELS; i++) {
			noise_texture[i] = glm::linearRand(glm::vec4(0), glm::vec4(1));
		}
	}

	for (int i=0; i<NUM_PARTICLES; i++) {
		auto& o = particles[i];
		auto randpt = glm::linearRand(world_min, world_max);
		randpt.y = coastline_height * (rnd::uni() * 3.f + 1.f);
		o.location = randpt;
		o.color = glm::vec3(rnd::uni());
	}


	{
		int i=0;
		glm::ivec2 dim = glm::ivec2(FUNGUS_DIM, FUNGUS_DIM);
		for (size_t y=0;y<dim.y;y++) {
			for (size_t x=0;x<dim.x;x++) {
				fungus_field.front()[i] = rnd::uni();
				fungus_field.back()[i] = rnd::uni();
			}
		}
	}

	emission_field.reset();

#ifdef AL_WIN
	{
		int i=0;
		glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
		for (size_t y=0;y<dim2.y;y++) {
			for (size_t x=0;x<dim2.x;x++, i++) {
				land[i] = glm::vec4(0., 1., 0., 0.);
			}
		}
	}
#else
	/*
		Create the initial landscape:
	*/
	{	
		int i=0;
		glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
		for (size_t y=0;y<dim2.y;y++) {
			for (size_t x=0;x<dim2.x;x++, i++) {
				glm::vec2 coord = glm::vec2(x, y);
				glm::vec2 norm = coord/glm::vec2(dim2);
				glm::vec2 snorm = norm*2.f-1.f;

				float w = 0.f;

				glm::vec2 p = snorm;
				//w += pow((cos(M_PI * p.x)+1.)*(cos(M_PI * p.y)+1.)*0.25, 0.5);

				p = p * 2.f;
				p += glm::vec2(0.234f, 0.567f);
				p = glm::rotate(p, 2.f);
				w += pow((cos(M_PI * p.x)+1.)*(cos(M_PI * p.y)+1.)*0.25, 0.5) * 0.5;


				p = p * 2.f;
				p += glm::vec2(0.234f, 0.567f);
				p = glm::rotate(p, 2.f);
				w += pow((cos(M_PI * p.x)+1.)*(cos(M_PI * p.y)+1.)*0.25, 0.5) * 0.25;

				p = p * 2.f;
				p += glm::vec2(0.234f, 0.567f);
				p = glm::rotate(p, 2.f);
				w += pow((cos(M_PI * p.x)+1.)*(cos(M_PI * p.y)+1.)*0.25, 0.5) * 0.125;

				p = p * glm::length(snorm);
				p += glm::vec2(0.234f, 0.567f);
				p = glm::rotate(p, 2.f);
				w += pow((cos(M_PI * p.x)+1.)*(cos(M_PI * p.y)+1.)*0.25, 0.5) * 0.125;


				w *= pow((cos(M_PI * snorm.x)+1.1)*(cos(M_PI * snorm.y)+1.1)*0.25, 0.35);


				w = glm::max(w - 0.2f, 0.f);

				land[i].w = w * 0.3 + 0.01;
			}
		}
	}
	
	
	generate_land_sdf_and_normals();
#endif

	if (1) {
		int div = sqrt(NUM_DEBUGDOTS);
		for (int i=0; i<NUM_DEBUGDOTS; i++) {
			auto& o = debugdots[i];
			
			float x = (i / div) / float(div);
			float z = (i % div) / float(div);
			
			//o.location = glm::linearRand(world_min, world_max);

			// normalized coordinate (0..1)
			glm::vec3 norm = glm::vec3(x, 0, z); //transform(world2field, o.location);

			// get land data at this point:
			// xyz is normal, w is height
			glm::vec4 landpt = al_field2d_readnorm_interp(glm::vec2(land_dim.x, land_dim.z), land, glm::vec2(norm.x, norm.z));
			
			// if flatness == 1, land is horizontal. 
			// if flatness == 0, land is vertical.
			float flatness = fabsf(landpt.y); // simplified dot product of landnorm with (0,1,0)
			// make it more extreme
			flatness = powf(flatness, 2.f);				

			// get land surface coordinate:
			glm::vec3 land_coord = transform(field2world, glm::vec3(norm.x, landpt.w, norm.z));

			// place on land
			o.location = land_coord;
			o.color = glm::vec3(flatness, 0.5, 1. - flatness) * 0.25f; //glm::vec3(0, 0, 1);
			o.size = particleSize;
		}
	}

	island_centres[0] = glm::vec3(120., 20., 70.);
	island_centres[1] = glm::vec3(70., 20., 215.);
	island_centres[2] = glm::vec3(120., 20., 345.);
	island_centres[3] = glm::vec3(285., 20., 385.);
	island_centres[4] = glm::vec3(255., 20., 295.);

	for (int i=0; i<NUM_TELEPORT_POINTS; i++ ) {
		teleport_points[i] = island_centres[i];
		teleport_points[i].y = world_centre.y;
	}
	cameraLoc = island_centres[2];
	vrLocation = nextVrLocation = cameraLoc;

	// make up some speaker locations:
	for (int i=0; i<NUM_ISLANDS; i++) {
		int id = NUM_DEBUGDOTS - NUM_ISLANDS - 1 + i;
		debugdots[id].location = island_centres[i];
		debugdots[id].color = glm::vec3(1,0,0);
		debugdots[id].size = particleSize * 500;
	}

	for (int i=0; i<NUM_CREATURES; i++) {
		Creature& a = creatures[i];
		a.idx = i;
		a.type = rnd::integer(4) + 1;

		creature_reset(i);
	}

}

void State::generate_land_sdf_and_normals() {
	{
		// generate SDF from land height:
		int i=0;
		for (size_t z=0;z<SDF_DIM;z++) {
			for (size_t y=0;y<SDF_DIM;y++) {
				for (size_t x=0;x<SDF_DIM;x++) {
					glm::vec3 coord = glm::vec3(x, y, z);
					glm::vec3 norm = coord/glm::vec3(SDF_DIM);
					glm::vec2 norm2 = glm::vec2(norm.x, norm.z);
					
					//int ii = al_field2d_index(dim2, glm::ivec2(x, z));
					//float w = land[ ii ].w;
					float w = al_field2d_readnorm_interp(land_dim2, land, norm2).w;

					distance[i] = norm.y < w ? -1. : 1.;
					distance_binary[i] = distance[i] < 0.f ? 0.f : 1.f;

					i++;
				}
			}
		}
		sdf_from_binary(sdf_dim, distance_binary, distance);
		//sdf_from_binary_deadreckoning(land_dim, distance_binary, distance);
		al_field3d_scale(sdf_dim, distance, 1.f/(SDF_DIM));
	}

	// generate land normals:
	int i=0;
	glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
	for (size_t y=0;y<dim2.y;y++) {
		for (size_t x=0;x<dim2.x;x++, i++) {
			glm::vec2 coord = glm::vec2(x, y);
			glm::vec2 norm = coord/glm::vec2(dim2);
			glm::vec2 snorm = norm*2.f-1.f;

			float w = land[i].w;

			glm::vec3 norm3 = glm::vec3(norm.x, w, norm.y);

			glm::vec3 normal = sdf_field_normal4(sdf_dim, distance, norm3, 1.f/SDF_DIM);
			land[i] = glm::vec4(normal, w);
		}
	}
}

void State::update_projector_loc() {
	Alice& alice = Alice::Instance();
	// read calibration:
	CloudDevice& kinect0 = alice.cloudDeviceManager.devices[0];
	CloudDevice& kinect1 = alice.cloudDeviceManager.devices[1];
	{
		console.log("READING JSON!!!");
		json calibjson;
		
		{
			// read a JSON file
			std::ifstream calibstr("projector_calibration/realcalib.json");
			calibstr >> calibjson;
			calibstr.close();
		}
		console.log("DIGGING INTO JSON!!!");

		glm::vec3 pos, cloud_translate;
		glm::quat orient, cloud_rotate;
		glm::vec4 frustum;

		if (calibjson.count("position")) {
			auto j = calibjson["position"];
			pos = glm::vec3(j.at(0), j.at(1), j.at(2));
		}
		if (calibjson.count("quat")) {
			auto j = calibjson["quat"];
			// jitter displays as x y z w
			// glm declares as w x y z
			orient = glm::quat(j.at(3), j.at(0), j.at(1), j.at(2));
		}
		if (calibjson.count("frustum")) {
			auto j = calibjson["frustum"];
			frustum = glm::vec4(j.at(0), j.at(1), j.at(2), j.at(3));
		}

		if (calibjson.count("cloud_translate")) {
			auto j = calibjson["cloud_translate"];
			cloud_translate = glm::vec3(j.at(0), j.at(1), j.at(2));
		}
		if (calibjson.count("cloud_rotate")) {
			auto j = calibjson["cloud_rotate"];
			// jitter displays as x y z w
			// glm declares as w x y z
			cloud_rotate = glm::quat(j.at(3), j.at(0), j.at(1), j.at(2));
		}


		console.log("pos %f %f %f", pos.x, pos.y, pos.z);
		console.log("quat %f %f %f %f", orient.x, orient.y, orient.z, orient.w);
		console.log("frustum %f %f %f %f", frustum.x, frustum.y, frustum.z, frustum.w);
		console.log("cloud_translate %f %f %f", cloud_translate.x, cloud_translate.y, cloud_translate.z);
		console.log("cloud_rotate %f %f %f %f", cloud_rotate.x, cloud_rotate.y, cloud_rotate.z, cloud_rotate.w);

		// TODO: determine the projector ground location in real-space
		auto real_loc = glm::vec3(state->projector1_location_x, 0., state->projector1_location_y);
		auto rot_mat = glm::rotate(projector1_rotation, glm::vec3(0,1,0));

		projectors[0].orientation = quat_cast(rot_mat) * orient;
		projectors[0].location = (transform(rot_mat, pos) + real_loc) * state->kinect2world_scale;
		projectors[0].far_clip = 6.f * state->kinect2world_scale;

		float nearclip = 1.f;
		projectors[0].frustum_min = glm::vec2(frustum.x, frustum.z) * nearclip;
		projectors[0].frustum_max = glm::vec2(frustum.y, frustum.w) * nearclip;
		projectors[0].near_clip = nearclip;

		// sequence is:
		// 1. apply cloud rotate (i.e. undo the kinect's rotation relative to ground)
		// 2. apply cloud translate (i.e. undo kinect's position relative to ground)
		// 3. apply projector loc (i.e. move ground center to desired location)
		// 4. apply projector rot (i.e. rotate system around ground center)
		// 5. scale to world

		kinect0.cloudTransform = glm::scale(glm::vec3(state->kinect2world_scale)) 
			* glm::translate(real_loc) 
			* rot_mat
			* glm::translate(cloud_translate) 
			* glm::mat4_cast(cloud_rotate);

		//kinect0.cloudTransform = glm::scale(glm::vec3(state->kinect2world_scale)) * glm::translate(cloud_translate) * glm::mat4_cast(cloud_rotate);
	}

	{
		console.log("READING JSON!!!");
		json calibjson;
		
		{
			// read a JSON file
			std::ifstream calibstr("projector_calibration2/realcalib.json");
			calibstr >> calibjson;
			calibstr.close();
		}
		console.log("DIGGING INTO JSON!!!");

		glm::vec3 pos, cloud_translate;
		glm::quat orient, cloud_rotate;
		glm::vec4 frustum;

		if (calibjson.count("position")) {
			auto j = calibjson["position"];
			pos = glm::vec3(j.at(0), j.at(1), j.at(2));
		}
		if (calibjson.count("quat")) {
			auto j = calibjson["quat"];
			// jitter displays as x y z w
			// glm declares as w x y z
			orient = glm::quat(j.at(3), j.at(0), j.at(1), j.at(2));
		}
		if (calibjson.count("frustum")) {
			auto j = calibjson["frustum"];
			frustum = glm::vec4(j.at(0), j.at(1), j.at(2), j.at(3));
		}

		if (calibjson.count("cloud_translate")) {
			auto j = calibjson["cloud_translate"];
			cloud_translate = glm::vec3(j.at(0), j.at(1), j.at(2));
		}
		if (calibjson.count("cloud_rotate")) {
			auto j = calibjson["cloud_rotate"];
			// jitter displays as x y z w
			// glm declares as w x y z
			cloud_rotate = glm::quat(j.at(3), j.at(0), j.at(1), j.at(2));
		}


		console.log("pos %f %f %f", pos.x, pos.y, pos.z);
		console.log("quat %f %f %f %f", orient.x, orient.y, orient.z, orient.w);
		console.log("frustum %f %f %f %f", frustum.x, frustum.y, frustum.z, frustum.w);
		console.log("cloud_translate %f %f %f", cloud_translate.x, cloud_translate.y, cloud_translate.z);
		console.log("cloud_rotate %f %f %f %f", cloud_rotate.x, cloud_rotate.y, cloud_rotate.z, cloud_rotate.w);

		
		// TODO: determine the projector ground location in real-space
		auto real_loc = glm::vec3(state->projector2_location_x, 0., state->projector2_location_y);
		auto rot_mat = glm::rotate(projector2_rotation, glm::vec3(0,1,0));

		projectors[1].orientation = quat_cast(rot_mat) * orient;
		projectors[1].location = (transform(rot_mat, pos) + real_loc) * state->kinect2world_scale;
		projectors[1].far_clip = 6.f * state->kinect2world_scale;

		float nearclip = 1.f;//0.1f * state->kinect2world_scale;
		projectors[1].frustum_min = glm::vec2(frustum.x, frustum.z) * nearclip;
		projectors[1].frustum_max = glm::vec2(frustum.y, frustum.w) * nearclip;
		projectors[1].near_clip = nearclip;

		// sequence is:
		// 1. apply cloud rotate (i.e. undo the kinect's rotation relative to ground)
		// 2. apply cloud translate (i.e. undo kinect's position relative to ground)
		// 3. apply projector loc (i.e. move ground center to desired location)
		// 4. scale to world

		kinect1.cloudTransform = glm::scale(glm::vec3(state->kinect2world_scale)) 
			* glm::translate(real_loc) 
			* rot_mat
			* glm::translate(cloud_translate) 
			* glm::mat4_cast(cloud_rotate);

		//kinect1.cloudTransform = glm::mat4(1.f);
	}

}

void test() {

}


extern "C" {
    AL_EXPORT int onload() {

		test();
    	
		Alice& alice = Alice::Instance();

		console.log("onload");
		console.log("sim alice %p", &alice);

		// import/allocate state
		state = statemap.create("state.bin", true);
		console.log("sim state %p should be size %d", state, sizeof(State));
		//state_initialize();
		console.log("onload state initialized");

		audiostate = audiostatemap.create("audio/audiostate.bin", true);

		
		#ifdef AL_WIN
		alice.goFullScreen = true;
		soloView = 0;
		ShowCursor(FALSE);
		#endif

		onReset();

		

		// set up projectors:
		state->update_projector_loc();
		{
			projectors[2].orientation = glm::angleAxis(float(-M_PI/2.), glm::vec3(1,0,0));
			projectors[2].location = 0.5f * (state->world_min + state->world_max);
			glm::vec2 aspectfactor = glm::vec2(float(projectors[2].fbo.dim.x) / projectors[2].fbo.dim.y, 1.f);
			projectors[2].frustum_min = glm::vec2(-1.f) * aspectfactor;
			projectors[2].frustum_max = glm::vec2(1.f) * aspectfactor;
			projectors[2].near_clip = 0.05;
			projectors[2].far_clip = (state->world_max.z - state->world_min.z);

			projectors[0].altShader = true;
			projectors[1].altShader = true;
		}


		landTex.generateMipMap = true;
		humanTex.generateMipMap = true;
		emissionTex.generateMipMap = true;
		flowTex.generateMipMap = true;
		

		enablers[SHOW_LANDMESH] = 1;
		enablers[SHOW_AS_GRID] = 0;
		enablers[SHOW_MINIMAP] = 0;//1;
		enablers[SHOW_OBJECTS] = 1;
		enablers[SHOW_TIMELAPSE] = 1;//1;
		enablers[SHOW_PARTICLES] = 1;//1;
		enablers[SHOW_DEBUGDOTS] = 0;//1;
		enablers[USE_OBJECT_SHADER] = 0;//1;
		enablers[SHOW_HUMANMESH] = 1;
		enablers[CALIBRATE] = 0;

		//threads_begin();



	
		console.log("onload fluid initialized");
	
		gBufferProj.dim = glm::ivec2(1920, 1200);
		for (int i=0; i<3; i++) {
			projectors[i].fbo.dim = gBufferProj.dim;
		}
		slowFbo.dim = gBufferProj.dim;
		//slowFbo.useFloatTexture = true;

		alice.hmd->connect();
		if (alice.hmd->connected) {
			gBufferVR.dim = alice.hmd->fbo.dim;
		} else {
			gBufferVR.dim = glm::ivec2(512, 512);
		}

		#ifdef AL_WIN
		alice.fps.setFPS(90);
		#endif

		// allocate on GPU:
		onReloadGPU();
		
		console.log("onload onReloadGPU ran");

		// register event handlers 
		alice.onFrame.connect(onFrame);
		alice.onReloadGPU.connect(onReloadGPU);
		alice.onReset.connect(onReset);
		alice.onKeyEvent.connect(onKeyEvent);
		#ifdef AL_WIN
		alice.window.position(4000, 200);
		#else
		alice.window.position(45, 45);
		#endif
      

		return 0;
    }
    
    AL_EXPORT int onunload() {
		Alice& alice = Alice::Instance();

		threads_end();

    	// free resources:
    	onUnloadGPU();
		console.log("unloaded GPU");
    	
    	// unregister handlers
    	alice.onFrame.disconnect(onFrame);
    	alice.onReloadGPU.disconnect(onReloadGPU);
		alice.onReset.disconnect(onReset);
		alice.onKeyEvent.disconnect(onKeyEvent);
		console.log("disconnected events");
    	
    	// export/free state
    	statemap.destroy(true);
		audiostatemap.destroy(true);

		console.log("let go of map");
	
		console.log("onunload done.");
        return 0;
    }
}

// this was just for debugging something on Windows
// through this I learned that ALL threads begun after a dll loads must 
#ifdef AL_WIN
extern "C" BOOL WINAPI DllMain(HINSTANCE hinstDLL,DWORD     fdwReason,LPVOID    lpvReserved) {
	switch(fdwReason) {
		case 0: fprintf(stderr, "DLLMAIN detach process\n"); break;
		case 1: fprintf(stderr, "DLLMAIN attach process\n"); break;
		case 2: fprintf(stderr, "DLLMAIN attach thread\n"); break;
		case 3: fprintf(stderr, "DLLMAIN detach thread\n"); break;
	}
	return TRUE;
}
#endif
