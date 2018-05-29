#include "al/al_console.h"
#include "al/al_math.h"
#include "al/al_distance.h"
#include "al/al_field3d.h"
#include "al/al_field2d.h"
#include "al/al_gl.h"
#include "al/al_kinect2.h"
#include "al/al_mmap.h"
#include "al/al_hmd.h"
#include "al/al_time.h"
#include "alice.h"
#include "state.h"


Shader objectShader;
Shader segmentShader;
Shader particleShader;
Shader landShader;
Shader heightMeshShader;
Shader deferShader; 

QuadMesh quadMesh;
GLuint colorTex;
FloatTexture3D fluidTex;
FloatTexture3D densityTex;
FloatTexture3D distanceTex;

FloatTexture2D fungusTex;
FloatTexture2D landTex;

VBO cubeVBO(sizeof(positions_cube), positions_cube);

VAO objectVAO;
VBO objectInstancesVBO(sizeof(State::objects));

VAO segmentVAO;
VBO segmentInstancesVBO(sizeof(State::segments));

VAO particlesVAO;
VBO particlesVBO(sizeof(State::particles));

float near_clip = 0.1f;
float far_clip = 12.f;
float particleSize = 1.f/196;
float camSpeed = 5.0f;
float camPitch;
float camYaw;
float camUp;
float camStrafe;
bool camForward;
bool camBackwards;
float creature_fluid_push = 0.2f;

int debugMode = 0;
int camMode = 0;

std::mutex sim_mutex;
bool accel = 0;
bool decel = 0;


glm::vec3 world_min(-4.f, 0.f, 0.f);
glm::vec3 world_max(4.f, 8.f, 8.f);
glm::vec3 world_centre(0.f, 1.8f, 4.f);
glm::vec3 prevVel = glm::vec3(0.);
glm::vec3  newCamLoc;

// how to convert world positions into fluid texture coordinates:
glm::mat4 world2fluid;
glm::mat4 fluid2world;
glm::mat4 vive2world;
glm::mat4 kinect2world; 
glm::mat4 viewMat;
glm::mat4 projMat;
glm::mat4 viewProjMat;
glm::mat4 viewMatInverse;
glm::mat4 projMatInverse;
glm::mat4 viewProjMatInverse;
glm::vec3 cameraLoc;
glm::quat cameraOri;

State * state;
Mmap<State> statemap;

Fluid3D<> fluid;
int fluid_passes = 14;
int fluid_noise_count = 32;
float fluid_decay = 0.9999f;
double fluid_viscosity = 0.00000001; //0.00001;
double fluid_boundary_damping = .2;
double fluid_noise = 8.;

float density_decay = 0.98f;
float density_diffuse = 0.01; // somwhere between 0.1 and 0.01 seems to be good
float density_scale = 50.;

MetroThread simThread(30);
MetroThread fluidThread(10);
bool isRunning = 1;

// angle of rotation for the camera direction
float angle=0.0;
// actual vector representing the camera's direction
float lx=0.0f,lz=-1.0f;
// XZ position of the camera
float x=0.0f,z=5.0f;

/*
	A helper for deferred rendering
*/
struct GBuffer {

	static const int numBuffers = 3;

	unsigned int fbo;
	unsigned int rbo;
	std::vector<unsigned int> textures;
	std::vector<unsigned int> attachments;

	//GLint min_filter = GL_LINEAR; 
	//GLint mag_filter = GL_LINEAR;
	GLint min_filter = GL_NEAREST;
	GLint mag_filter = GL_NEAREST;

	glm::ivec2 dim = glm::ivec2(1024, 1024);

	GBuffer(int numbuffers = 3) {
		textures.resize(numbuffers);
		attachments.resize(numbuffers);
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

GBuffer gBuffer;

void fluid_land_resist(glm::vec3 * velocities, const glm::ivec3 field_dim, float * distancefield, const glm::ivec3 land_dim) {

	/* 
		boundary effect of landscape: the closer we get to the landscape, 
		the more the velocities should be redirected away from the surface




	*/

	glm::vec3 field_dimf = glm::vec3(field_dim);

	int i = 0;
	for (size_t z = 0; z<field_dim.z; z++) {
		for (size_t y = 0; y<field_dim.y; y++) {
			for (size_t x = 0; x<field_dim.x; x++, i++) {
				
				// get norm'd coordinate:
				glm::vec3 norm = glm::vec3(x,y,z) / field_dimf;

				// use this to sample the landscape:
				float sdist;
				al_field3d_readnorm_interp(land_dim, state->distance, norm, &sdist );
				float dist = fabsf(sdist);

				// generate a normalized influence factor -- the closer we are to the surface, the greater this is
				float influence = glm::smoothstep(0.2f, 0.f, dist);

				
				// get a normal for the land:
				// TODO: or read from state->land xyz?
				glm::vec3 normal = sdf_field_normal4(land_dim, state->distance, norm, 2.f/LAND_DIM);

				glm::vec3& vel = velocities[i];
				//glm::vec3 veln = safe_normalize(vel);
				float speed = glm::length(vel);

				// get the projection of vel onto normal axis
				// i.e. the component of vel that points in either normal direction:
				glm::vec3 normal_component = normal * (dot(vel, normal));

				// remove this component from the original velocity:
				glm::vec3 without_normal_component = vel - normal_component;

				// and re-scale to original magnitude:
				glm::vec3 rescaled = safe_normalize(without_normal_component) * speed;

				// update:
				vel = mix(vel, rescaled, influence);	
			}
		}
	}
	
}

/*
void apply_fluid_boundary2(glm::vec3 * velocities, const glm::vec4 * landscape, const size_t dim0, const size_t dim1, const size_t dim2) {

	const float influence_offset = -(float)fluid_boundary_damping;
	const float influence_scale = 1.f / (float)fluid_boundary_damping;

	// probably don't need the triple loop here -- could do it cell by cell.
	int i = 0;
	for (size_t z = 0; z<dim2; z++) {
		for (size_t y = 0; y<dim1; y++) {
			for (size_t x = 0; x<dim0; x++, i++) {

				const glm::vec4 land = landscape[i];
				const double distance = fabs(land.w);
				//const float inside = sign(land.w);	// do we care?
				const double influence = glm::clamp((distance + influence_offset) * influence_scale, 0., 1.);
				

				glm::vec3& vel = velocities[i];
				//glm::vec3 veln = safe_normalize(vel);
				float speed = glm::length(vel);

				// already normalized.
				const glm::vec3 normal = glm::vec3(land);	

				// get the projection of vel onto normal axis
				// i.e. the component of vel that points in either normal direction:
				glm::vec3 normal_component = normal * (dot(vel, normal));

				// remove this component from the original velocity:
				glm::vec3 without_normal_component = vel - normal_component;

				// and re-scale to original magnitude:
				glm::vec3 rescaled = safe_normalize(without_normal_component) * speed;

				// update:
				vel = mix(rescaled, vel, influence);
				
			}
		}
	}
}*/

void fluid_update(double dt) {
	// update fluid
	Field3D<>& velocities = fluid.velocities;
	const size_t dim0 = velocities.dimx();
	const size_t dim1 = velocities.dimy();
	const size_t dim2 = velocities.dimz();
	glm::vec3 * data = (glm::vec3 *)velocities.front().ptr();
	//float * boundary = boundary;

	// and some turbulence:
	if (0) {
		for (int i=0; i < rnd::integer(fluid_noise_count); i++) {
			// pick a cell at random:
			glm::vec3 * cell = data + (size_t)rnd::integer(dim0*dim1*dim2);
			// add a random vector:
			*cell = glm::sphericalRand(rnd::uni((float)fluid_noise));
		}
	}

	//apply_fluid_boundary2(data, (glm::vec4 *)landscape.ptr(), dim0, dim1, dim2);
	velocities.diffuse(fluid_viscosity, fluid_passes);
	// apply boundaries:
	//apply_fluid_boundary2(data, boundary, dim0, dim1, dim2);
	//apply_fluid_boundary2(data, (glm::vec4 *)landscape.ptr(), dim0, dim1, dim2);
	// stabilize:
	fluid.project(fluid_passes / 2);
	// advect:
	velocities.advect(velocities.back(), 1.);

	fluid_land_resist((glm::vec3 *)velocities.front().data, field_dim, state->distance, land_dim);

	// apply boundaries:
	//apply_fluid_boundary2(data, boundary, dim0, dim1, dim2);
	//apply_fluid_boundary2(data, (glm::vec4 *)landscape.ptr(), dim0, dim1, dim2);
	velocities.front().scale(fluid_decay);

	// clear gradients:
	fluid.gradient.front().zero();
	fluid.gradient.back().zero();
}

void fungus_update(float dt) {
	const glm::ivec2 dim = glm::ivec2(FUNGUS_DIM, FUNGUS_DIM);
	auto & src_array = state->fungus;
	auto & dst_array = state->fungus_old;

	//float * land = state->

	for (int i=0, y=0; y<dim.y; y++) {
		for (int x=0; x<dim.x; x++, i++) {
			const glm::vec2 cell = glm::vec2(float(x),float(y));			
			const glm::vec2 norm = cell/glm::vec2(dim); // TODO convert to premultiplier
			float C = src_array[i];
			float C1 = C - 0.1;
			//float h = 20 * .1;//heightmap_array.sample(norm);
			glm::vec4 l;
			al_field2d_readnorm_interp(glm::ivec2(LAND_DIM, LAND_DIM), state->land, norm, &l);
			float h = 20.f * l.w;
			float hu = 0.;//humanmap_array.sample(norm);
			float dst = C;
			if (h <= 0 || hu > 0.1) {
				// force lowlands to be vacant
				// (note, human will also do this)
				dst = 0;
			} else if (C < -0.1) {
				// very negative values gradually drift back to zero
				dst = C * 0.999;
			} else if (C < 0) {
				// and then jump to zero when in [-0.1,0) range
				dst = 0;
			} else if (h < C1 || rnd::uni() < 0.00005*h) {
				// if land lower than vitality, decrease vitality
				// also random chance of decay for any living cell
				dst = h*rnd::uni();
			} else if (rnd::uni() < 0.06*h) {
				// migration chance increases with altitude
				// pick a neighbour cell:
				glm::vec2 tc = cell + glm::vec2(floor(rnd::uni()*3)-1, floor(rnd::uni()*3)-1);
				float tv = al_field2d_read(dim, src_array, tc); //src_array.samplepix(tc);
				// if alive, copy it
				if (tv > 0.) { dst = tv; }
			}
			dst_array[i] = dst;
		}
	}

	memcpy(src_array, dst_array, sizeof(state->fungus));

	//fungus_write = !fungus_write;
}

void sim_update(double dt) {

	// inverse dt gives rate (per second)
	float idt = 1.f/dt;

	const Alice& alice = Alice::Instance();
	if (!alice.isSimulating) return;


	if (0) {
		glm::mat4 tr = glm::translate(glm::mat4(1.f), glm::vec3(-2.f, -2.f, -2.7f));
		glm::mat4 ro = glm::rotate(glm::mat4(1.f), 1.8f, glm::vec3(1.f, 0.f, 0.f));
		kinect2world = ro * tr;
	}

	// get the most recent complete frame:
	const CloudFrame& cloudFrame = alice.cloudDevice->cloudFrame();
	const glm::vec3 * cloud_points = cloudFrame.xyz;
	const glm::vec2 * uv_points = cloudFrame.uv;
	uint64_t max_cloud_points = sizeof(cloudFrame.xyz)/sizeof(glm::vec3);

	// 
	al_field3d_scale(field_dim, state->density, glm::vec3(density_decay));
	al_field3d_diffuse(field_dim, state->density, state->density, density_diffuse);
	memcpy(state->density_back, state->density, sizeof(glm::vec3) * FIELD_VOXELS);

	fungus_update(dt);
	
	for (int i=0; i<NUM_PARTICLES; i++) {
		Particle &o = state->particles[i];

		glm::vec3 flow;
		fluid.velocities.front().readnorm(transform(world2fluid, o.location), &flow.x);
		
		//glm::vec3 noise;// = glm::sphericalRand(0.02f);
		o.velocity = flow * idt;
			// + noise;

		if (alice.cloudDevice->capturing) {
			uint64_t idx = i % max_cloud_points;
			glm::vec3 p = cloud_points[idx];

			if (p.z > 1.f) {

				p = glm::vec3(kinect2world * glm::vec4(p, 1.f));
				glm::vec2 uv = uv_points[idx];
				// this is in meters, but that seems a bit limited for our world
				glm::vec3 campos = glm::vec3(0., 1.1, 0.);
				p = p + campos;
				o.location = p;
				o.color = glm::vec3(uv, 0.5f);
			}
		} 
	}

	// simulate creatures:
	for (int i=0; i<NUM_OBJECTS; i++) {
		auto &o = state->objects[i];

		// get norm'd coordinate:
		glm::vec3 norm = transform(world2fluid, o.location);

		// get fluid flow:
		glm::vec3 flow;
		fluid.velocities.front().readnorm(norm, &flow.x);

		// get my distance from the ground:
		float sdist; // creature's distance above the ground (or negative if below)
		al_field3d_readnorm_interp(land_dim, state->distance, norm, &sdist);

		// convert to meters per second:
		flow *= idt;

		// if below ground, rise up;
		// if above ground, sink down:
		float gravity = 0.1f;
		flow.y += sdist < 0.1f ? gravity : -gravity;


		// set my velocity, in meters per second:
		o.velocity = flow;
		//if(accel == 1) o.velocity += o.velocity;
		//else if (decel == 1) o.velocity -= o.velocity * glm::vec3(2.);

/*
		// use this to sample the landscape:
		
		// get normal here too
		// get a normal for the land:
		glm::vec3 normal = sdf_field_normal4(land_dim, distancefield, norm, 0.5f/LAND_DIM);
		*/

		// add my direction to the fluid current
		glm::vec3 push = quat_uf(o.orientation) * (creature_fluid_push * (float)dt);
		fluid.velocities.front().addnorm(norm, &push.x);
		if (fmod(o.phase, 1.f) < 0.02f) {
			al_field3d_addnorm_interp(field_dim, state->density, norm, o.color * density_scale);
		}

	}


	//if(accel == 1)state->objects[0].velocity += state->objects[0].velocity;
	//else if(decel == 1)state->objects[0].velocity -= state->objects[0].velocity * glm::vec3(2.);

	for (int i=0; i<NUM_SEGMENTS; i++) {
		auto &o = state->segments[i];
		if (i % 8 == 0) {
			// a root;
			glm::vec3 fluidloc = transform(world2fluid, o.location);
			glm::vec3 flow;
			fluid.velocities.front().readnorm(fluidloc, &flow.x);
			glm::vec3 push = quat_uf(o.orientation) * (creature_fluid_push * (float)dt);
			fluid.velocities.front().addnorm(fluidloc, &push.x);
			o.velocity = flow * idt;

			al_field3d_addnorm_interp(field_dim, state->density, fluidloc, o.color * density_scale * 0.02f);
			
		} else {
			auto& p = state->segments[i-1];
			o.scale = p.scale * 0.9f;
		}
	}
}


void onUnloadGPU() {
	// free resources:
	landShader.dest_closing();
	heightMeshShader.dest_closing();
	particleShader.dest_closing();
	objectShader.dest_closing();
	segmentShader.dest_closing();
	deferShader.dest_closing();
	quadMesh.dest_closing();
	cubeVBO.dest_closing();
	objectInstancesVBO.dest_closing();
	objectVAO.dest_closing();
	segmentInstancesVBO.dest_closing();
	segmentVAO.dest_closing();
	particlesVAO.dest_closing();
	particlesVAO.dest_closing();

	fluidTex.dest_closing();
	densityTex.dest_closing();
	distanceTex.dest_closing();
	fungusTex.dest_closing();
	landTex.dest_closing();

	gBuffer.dest_closing();
	Alice::Instance().hmd->dest_closing();

	if (colorTex) {
		glDeleteTextures(1, &colorTex);
		colorTex = 0;
	}

	
}

void onReloadGPU() {

	onUnloadGPU();

	objectShader.readFiles("object.vert.glsl", "object.frag.glsl");
	segmentShader.readFiles("segment.vert.glsl", "segment.frag.glsl");
	particleShader.readFiles("particle.vert.glsl", "particle.frag.glsl");
	landShader.readFiles("land.vert.glsl", "land.frag.glsl");
	heightMeshShader.readFiles("hmesh.vert.glsl", "hmesh.frag.glsl");
	deferShader.readFiles("defer.vert.glsl", "defer.frag.glsl");
	
	quadMesh.dest_changed();

	objectVAO.bind();
	cubeVBO.bind();
	objectVAO.attr(0, 3, GL_FLOAT, sizeof(glm::vec3), 0);
	objectInstancesVBO.bind();
	objectVAO.attr(2, &Object::location, true);
	objectVAO.attr(3, &Object::orientation, true);
	objectVAO.attr(4, &Object::scale, true);
	objectVAO.attr(5, &Object::phase, true);
	objectVAO.attr(6, &Object::color, true);
		
	segmentVAO.bind();
	cubeVBO.bind();
	segmentVAO.attr(0, 3, GL_FLOAT, sizeof(glm::vec3), 0);
	segmentInstancesVBO.bind();
	segmentVAO.attr(2, &Segment::location, true);
	segmentVAO.attr(3, &Segment::orientation, true);
	segmentVAO.attr(4, &Segment::scale, true);
	segmentVAO.attr(5, &Segment::phase, true);
	segmentVAO.attr(6, &Segment::color, true);

	particlesVAO.bind();
	particlesVBO.bind();
	particlesVAO.attr(0, &Particle::location);
	particlesVAO.attr(1, &Particle::color);

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

	gBuffer.dest_changed();
	
	Alice::Instance().hmd->dest_changed();
}


void draw_scene(int width, int height) {
	double t = Alice::Instance().simTime;

	if (0) {
		heightMeshShader.use();
		heightMeshShader.uniform("uViewProjectionMatrix", viewProjMat);
		heightMeshShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);

	} else {
		landShader.use();
		landShader.uniform("time", t);
		landShader.uniform("uViewProjectionMatrix", viewProjMat);
		landShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		landShader.uniform("uNearClip", near_clip);
		landShader.uniform("uFarClip", far_clip);
		landShader.uniform("uDistanceTex", 4);
		landShader.uniform("uFungusTex", 5);
		landShader.uniform("uLandTex", 6);
		landShader.uniform("uLandMatrix", world2fluid);
		distanceTex.bind(4);
		fungusTex.bind(5);
		landTex.bind(6);
		quadMesh.draw();
		distanceTex.unbind(4);
		fungusTex.unbind(5);
		landTex.unbind(6);
	}

	objectShader.use();
	objectShader.uniform("time", t);
	objectShader.uniform("uViewMatrix", viewMat);
	objectShader.uniform("uViewProjectionMatrix", viewProjMat);
	objectVAO.drawInstanced(sizeof(positions_cube) / sizeof(glm::vec3), NUM_OBJECTS);

	segmentShader.use();
	segmentShader.uniform("time", t);
	segmentShader.uniform("uViewMatrix", viewMat);
	segmentShader.uniform("uViewProjectionMatrix", viewProjMat);
	segmentVAO.drawInstanced(sizeof(positions_cube) / sizeof(glm::vec3), NUM_SEGMENTS);

	particleShader.use(); 
	particleShader.uniform("time", t);
	particleShader.uniform("uViewMatrix", viewMat);
	particleShader.uniform("uViewMatrixInverse", viewMatInverse);
	particleShader.uniform("uProjectionMatrix", projMat);
	particleShader.uniform("uViewProjectionMatrix", viewProjMat);
	particleShader.uniform("uViewPortHeight", (float)height);
	particleShader.uniform("uPointSize", particleSize);
	particleShader.uniform("uColorTex", 0);

	glBindTexture(GL_TEXTURE_2D, colorTex);
	glEnable( GL_PROGRAM_POINT_SIZE );
	glEnable(GL_POINT_SPRITE);
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	particlesVAO.draw(NUM_PARTICLES, GL_POINTS);
	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDisable(GL_POINT_SPRITE);
	glBindTexture(GL_TEXTURE_2D, 0);

	glDisable(GL_CULL_FACE);
}

void onFrame(uint32_t width, uint32_t height) {
	const Alice& alice = Alice::Instance();
	double t = alice.simTime;
	float dt = alice.dt;
	float aspect = width/float(height);

	if (alice.framecount % 60 == 0) console.log("fps %f at %f; fluid %f(%f) sim %f(%f)", alice.fpsAvg, t, fluidThread.fps.fps, fluidThread.potentialFPS(), simThread.fps.fps, simThread.potentialFPS());

	if (alice.isSimulating && isRunning) {
		// keep the simulation in here to absolute minimum
		// since it detracts from frame rate
		// here we should only be extrapolating visible features
		// such as location (and maybe also orientation?)

		for (int i=0; i<NUM_PARTICLES; i++) {
			Particle &o = state->particles[i];
			o.location = wrap(o.location + o.velocity * dt, world_min, world_max);
		}
	
		for (int i=0; i<NUM_OBJECTS; i++) {
			auto &o = state->objects[i];
			// TODO: dt-ify this:	
			o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.015f));
			o.location = wrap(o.location + o.velocity * dt, world_min, world_max);
			o.phase += dt;
		}

		for (int i=0; i<NUM_SEGMENTS; i++) {
			auto &o = state->segments[i];
			if (i % 8 == 0) {
				// a root;
				// TODO: dt-ify
				o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.015f));
				o.location = wrap(o.location + o.velocity * dt, world_min, world_max);
				o.phase += dt;
			} else {
				auto& p = state->segments[i-1];
				o.orientation = safe_normalize(glm::slerp(o.orientation, p.orientation, 0.015f));
				glm::vec3 uz = quat_uz(p.orientation);
				o.location = p.location + uz*o.scale;
				o.phase = p.phase + 0.1f;
			}
		}

		//change mode to have object[0], segement[0], or nothing in focus
		if(debugMode % 3 == 1){
			state->objects[0].location = world_centre;
			state->objects[0].scale = 1;
			state->segments[0].scale = 0.25;
		}else if(debugMode % 3 == 2){
			state->segments[0].location = world_centre;
			state->segments[0].scale = 1;
			state->objects[0].scale = 0.25;
		}else{
			state->segments[0].scale = 0.25;
			state->objects[0].scale = 0.25;
		}

		

		// upload VBO data to GPU:
		objectInstancesVBO.submit(&state->objects[0], sizeof(state->objects));
		segmentInstancesVBO.submit(&state->segments[0], sizeof(state->segments));
		particlesVBO.submit(&state->particles[0], sizeof(state->particles));
		
		// upload texture data to GPU:
		fluidTex.submit(fluid.velocities.dim(), (glm::vec3 *)fluid.velocities.front()[0]);
		densityTex.submit(field_dim, state->density_back);
		fungusTex.submit(glm::ivec2(FUNGUS_DIM, FUNGUS_DIM), &state->fungus[0]);
		landTex.submit(glm::ivec2(LAND_DIM, LAND_DIM), &state->land[0]);
		distanceTex.submit(land_dim, (float *)&state->distance[0]);

		const ColourFrame& image = alice.cloudDevice->colourFrame();
		glBindTexture(GL_TEXTURE_2D, colorTex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cColorWidth, cColorHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image.color);
	}

	
	Hmd& vive = *alice.hmd;
	SimpleFBO& fbo = vive.fbo;
	if (vive.connected) {	
		vive.near_clip = near_clip;
		vive.far_clip = far_clip;	
		vive.update();
		glEnable(GL_SCISSOR_TEST);

		for (int eye = 0; eye < 2; eye++) {
			gBuffer.begin();

			glScissor(eye * gBuffer.dim.x / 2, 0, gBuffer.dim.x / 2, gBuffer.dim.y);
			glViewport(eye * gBuffer.dim.x / 2, 0, gBuffer.dim.x / 2, gBuffer.dim.y);
			glEnable(GL_DEPTH_TEST);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			// update nav
			viewMat = glm::inverse(vive.m_mat4viewEye[eye]) * glm::mat4_cast(glm::inverse(vive.mTrackedQuat)) * glm::translate(glm::mat4(1.f), -vive.mTrackedPosition) * vive2world;
			projMat = glm::frustum(vive.frustum[eye].l, vive.frustum[eye].r, vive.frustum[eye].b, vive.frustum[eye].t, vive.frustum[eye].n, vive.frustum[eye].f);

			viewProjMat = projMat * viewMat;
			projMatInverse = glm::inverse(projMat);
			viewMatInverse = glm::inverse(viewMat);
			viewProjMatInverse = glm::inverse(viewProjMat);

			draw_scene(gBuffer.dim.x / 2, gBuffer.dim.y);
			gBuffer.end();

			//glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this

			// now process the GBuffer and render the result into the fbo
			fbo.begin();
			//glScissor(0, 0, fbo.dim.x, fbo.dim.y);
			//glViewport(0, 0, fbo.dim.x, fbo.dim.y);
			glEnable(GL_DEPTH_TEST);
			glClearColor(0.f, 0.f, 0.f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			{
				deferShader.use();
				deferShader.uniform("gColor", 0);
				deferShader.uniform("gNormal", 1);
				deferShader.uniform("gPosition", 2);
				deferShader.uniform("uDensityTex", 6);
				deferShader.uniform("uFluidTex", 7);

				deferShader.uniform("uViewMatrix", viewMat);
				deferShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
				deferShader.uniform("uNearClip", near_clip);
				deferShader.uniform("uFarClip", far_clip);
				deferShader.uniform("uFluidMatrix", world2fluid);
				deferShader.uniform("time", Alice::Instance().simTime);
				deferShader.uniform("uDim", glm::vec2(gBuffer.dim.x, gBuffer.dim.y));
				deferShader.uniform("uTexTransform", glm::vec2(0.5, eye * 0.5));
				densityTex.bind(6);
				fluidTex.bind(7);
				gBuffer.bindTextures();
				quadMesh.draw();
				densityTex.unbind(6);
				fluidTex.unbind(7);
				gBuffer.unbindTextures();
				deferShader.unuse();
			}
			fbo.end();
		}
		glDisable(GL_SCISSOR_TEST);
	} else {
		// draw the scene into the GBuffer:
		gBuffer.begin();
		glEnable(GL_SCISSOR_TEST);
	
		// No HMD:
		glScissor(0, 0, gBuffer.dim.x, gBuffer.dim.y);
		glViewport(0, 0, gBuffer.dim.x, gBuffer.dim.y);
		glEnable(GL_DEPTH_TEST);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// update nav

		int camModeMax = 5;
		double a = M_PI * t / 30.;
		//when c is pressed, swap between normal camera, objects[0] camera, segments[0] camera, and a sine wave movement
		if(camMode % camModeMax == 1){
			/*viewMat = glm::lookAt(
				glm::vec3(state->objects[0].location), 
				state->objects[0].location + (state->objects[0].velocity + prevVel)/glm::vec3(2.), 
				glm::vec3(0., 1., 0.));*/
			auto& o = state->objects[0];

			glm::vec3 fluidloc = transform(world2fluid, o.location);
			glm::vec3 flow;
			fluid.velocities.front().readnorm(fluidloc, &flow.x);
			flow = flow * dt * 100.0f;
			
			cameraLoc = glm::mix(cameraLoc, o.location + flow, 0.1f);
			cameraOri = glm::slerp(cameraOri, o.orientation, 0.01f);
			//TODO: Once creatures follow the ground, fix boom going into the earth

			viewMat = glm::inverse(glm::translate(cameraLoc) * glm::mat4_cast(cameraOri) * glm::translate(glm::vec3(0., 0.3, 0.75)));
			projMat = glm::perspective(glm::radians(75.0f), aspect, near_clip, far_clip);
			prevVel = glm::vec3(o.velocity);

		}else if(camMode % camModeMax == 2){
			double a = M_PI * t / 30.;
			/*viewMat = glm::lookAt(
				glm::vec3(state->segments[0].location), 
				state->segments[0].location + (state->segments[0].velocity + prevVel)/glm::vec3(2.), 
				glm::vec3(0., 1., 0.));*/

			auto& o = state->segments[0];

			glm::vec3 fluidloc = transform(world2fluid, o.location);
			glm::vec3 flow;
			fluid.velocities.front().readnorm(fluidloc, &flow.x);
			flow = flow * dt * 100.0f;
			
			cameraLoc = glm::mix(cameraLoc, o.location + flow, 0.1f);
			cameraOri = glm::slerp(cameraOri, o.orientation, 0.01f);
			//TODO: Once creatures follow the ground, fix boom going into the earth

			viewMat = glm::inverse(glm::translate(cameraLoc) * glm::mat4_cast(cameraOri) * glm::translate(glm::vec3(0., 0.4, 1.)));
			projMat = glm::perspective(glm::radians(75.0f), aspect, near_clip, far_clip);
			prevVel = glm::vec3(o.velocity);

		}else if(camMode % camModeMax == 3){
			viewMat = glm::lookAt(
				world_centre + 
				glm::vec3(0.5*sin(t), 0.85*sin(0.5*a), 4.*sin(a)), 
				world_centre, 
				glm::vec3(0., 1., 0.));
		}else if(camMode % camModeMax == 4){
			
			if(camForward){
				newCamLoc = cameraLoc + quat_uf(cameraOri)* (camSpeed * 0.01f);}
			else if(camBackwards){
				 newCamLoc = cameraLoc + quat_uf(cameraOri)* -(camSpeed * 0.01f);}

			newCamLoc = glm::vec3 (newCamLoc.x, camUp*0.01f, newCamLoc.z);
			cameraLoc = glm::mix(cameraLoc,newCamLoc, 0.5f);

			
			glm::quat newCamRot = glm::angleAxis(camYaw*0.01f, glm::vec3(0, 1, 0)) * glm::angleAxis(camPitch*0.01f, glm::vec3(1, 0, 0));
			newCamRot = glm::normalize(newCamRot);
			//glm::quat (0.f, camYaw*0.01f, 0.f , camPitch*0.01f);
			cameraOri = glm::mix(cameraOri,newCamRot, 0.5f);

			viewMat = glm::inverse(glm::translate(cameraLoc) * glm::mat4_cast(cameraOri) * glm::translate(glm::vec3(0., 0.3, 0.75)));
			projMat = glm::perspective(glm::radians(75.0f), aspect, near_clip, far_clip);

			camForward = false;
			camBackwards = false;
		}else{
			double a = M_PI * t / 30.;
			viewMat = glm::lookAt(
				world_centre + 
				glm::vec3(3.*cos(a), 0.85*sin(0.5*a), 4.*sin(a)), 
				world_centre, 
				glm::vec3(0., 1., 0.));
			projMat = glm::perspective(glm::radians(75.0f), aspect, near_clip, far_clip);
		}
		
		
		viewProjMat = projMat * viewMat;

		projMatInverse = glm::inverse(projMat);
		viewMatInverse = glm::inverse(viewMat);

		viewProjMatInverse = glm::inverse(viewProjMat);

		draw_scene(gBuffer.dim.x, gBuffer.dim.y);
	
		glDisable(GL_SCISSOR_TEST);
		gBuffer.end();
		
		//glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this

		// now process the GBuffer and render the result into the fbo
		SimpleFBO& fbo = alice.hmd->fbo;

		fbo.begin();
		glEnable(GL_SCISSOR_TEST);
		glScissor(0, 0, fbo.dim.x, fbo.dim.y);
		glViewport(0, 0, fbo.dim.x, fbo.dim.y);
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.f, 0.f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		{
			deferShader.use();
			deferShader.uniform("gColor", 0);
			deferShader.uniform("gNormal", 1);
			deferShader.uniform("gPosition", 2);
			deferShader.uniform("uDistanceTex", 4);
			//deferShader.uniform("uFungusTex", 5);
			deferShader.uniform("uDensityTex", 6);
			deferShader.uniform("uFluidTex", 7);

			deferShader.uniform("uViewMatrix", viewMat);
			deferShader.uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
			deferShader.uniform("uNearClip", near_clip);
			deferShader.uniform("uFarClip", far_clip);
			deferShader.uniform("uFluidMatrix", world2fluid);
			deferShader.uniform("time", Alice::Instance().simTime);
			deferShader.uniform("uDim", glm::vec2(gBuffer.dim.x, gBuffer.dim.y));
			deferShader.uniform("uTexTransform", glm::vec2(1., 0.));
			distanceTex.bind(4);
			//fungusTex.bind(5);
			densityTex.bind(6);
			fluidTex.bind(7);
			gBuffer.bindTextures();
			quadMesh.draw();

			distanceTex.unbind(4);
			//fungusTex.unbind(5);
			densityTex.unbind(6);
			fluidTex.unbind(7);
			gBuffer.unbindTextures();
			deferShader.unuse();
		}
		glDisable(GL_SCISSOR_TEST);
		fbo.end();
	}
	
	glViewport(0, 0, width, height);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.f, 0.f, 0.f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	fbo.draw();

	alice.hmd->submit();
	
	// openvr header recommends this after submit:
	//glFlush();
	//glFinish();

}

void onKeyEvent(int keycode, int scancode, int downup, bool shift, bool ctrl, bool alt, bool cmd){
	Alice& alice = Alice::Instance();
	switch(keycode) {
		case GLFW_KEY_ENTER: {
			if (downup && alt) {
				if (alice.hmd->connected) {
					alice.hmd->disconnect();
				} else if (alice.hmd->connect()) {
					gBuffer.dim = alice.hmd->fbo.dim;
					gBuffer.dest_changed();
					alice.hmd->dest_changed();
					alice.desiredFrameRate = 90;
				}
			}
		} break;
		case GLFW_KEY_D: {
			//console.log("D was pressed");
			if (downup) debugMode++;
		} break;
		case GLFW_KEY_C: {
			//console.log("C was pressed");
			if (downup) camMode++;
		} break;
		//forward
		case GLFW_KEY_UP: {
			//state->objects[0].velocity++;
			accel = 1;
			float aclTst = accel;
			console.log("Accel = %f", accel);//aclTst);
		} break;
		//backward
		case GLFW_KEY_DOWN: {
			//state->objects[0].velocity -= state->objects[0].velocity * glm::vec3(2.);
			decel = 1;
		} break;
		
		case GLFW_KEY_LEFT: {
			
		} break;
		
		case GLFW_KEY_RIGHT: {
			
		} break;
		//pitch up
		case GLFW_KEY_KP_8: {
			camPitch += camSpeed;
		} break;
		//pitch down
		case GLFW_KEY_KP_2: {
			camPitch -= camSpeed;
		} break;
		//yaw left
		case GLFW_KEY_KP_4: {
			camYaw += camSpeed;
		} break;
		//yaw right
		case GLFW_KEY_KP_6: {
			camYaw -= camSpeed;
		} break;
		//Go left
		case GLFW_KEY_KP_7: {
			camStrafe -= camSpeed;
		} break;
		//Go right
		case GLFW_KEY_KP_9: {
			camStrafe += camSpeed;
		} break;
		//Go Up
		case GLFW_KEY_KP_ADD: {
			camUp += camSpeed;
		} break;
		//Go down
		case GLFW_KEY_KP_SUBTRACT: { 
			camUp -= camSpeed;
		} break;
		//Go Forward
		case GLFW_KEY_KP_1: {
			camForward = true;
		} break;
		//Go Back
		case GLFW_KEY_KP_3: {
			camBackwards = true;
		} break;
		// default:
		//state->objects[0].velocity = glm::vec3(0.);
		accel = 0;
		decel = 0;
	}

}


void threads_begin() {
	console.log("starting threads");
	// allow threads to run
	isRunning = true;
	simThread.begin(sim_update);
	fluidThread.begin(fluid_update);
	console.log("started threads");
}

void threads_end() {
	// release threads:
	isRunning = false;
	console.log("ending threads");
	simThread.end();
	fluidThread.end();
	console.log("ended threads");
}

// The onReset event is triggered when pressing the "Backspace" key in Alice
void onReset() {

	threads_end();

	// zero by default:
	memset(state, 0, sizeof(State));

	for (int i=0; i<NUM_OBJECTS; i++) {
		auto& o = state->objects[i];
		o.location = world_centre+glm::ballRand(1.f);
		o.color = glm::ballRand(1.f)*0.5f+0.5f;
		o.phase = rnd::uni();
		o.scale = 0.25;
	}
	for (int i=0; i<NUM_SEGMENTS; i++) {
		auto& o = state->segments[i];
		o.location = world_centre+glm::ballRand(1.f);
		o.color = glm::ballRand(1.f)*0.5f+0.5f;
		o.phase = rnd::uni();
		o.scale = 0.25;
	}
	for (int i=0; i<NUM_PARTICLES; i++) {
		auto& o = state->particles[i];
		o.location = world_centre+glm::ballRand(1.f);
		o.color = glm::vec3(1.f);
	}


	{
		int i=0;
		glm::ivec2 dim = glm::ivec2(FUNGUS_DIM, FUNGUS_DIM);
		for (size_t y=0;y<dim.y;y++) {
			for (size_t x=0;x<dim.x;x++) {
				state->fungus[i] = rnd::uni();
				state->fungus_old[i] = rnd::uni();
			}
		}
	}

	//al_field3d_zero(field_dim, state->density);
	{
		int i=0;
		glm::ivec3 dim = field_dim;
		for (size_t z=0;z<dim.z;z++) {
			for (size_t y=0;y<dim.y;y++) {
				for (size_t x=0;x<dim.x;x++) {
					glm::vec3 coord = glm::vec3(x, y, z);
					glm::vec3 norm = coord/glm::vec3(dim);
					//glm::vec3 snorm = norm*2.f-1.f;
					//glm::vec3 snormhalf = snorm * 0.5f;
					state->density[i] = glm::vec3(0.f);//norm * 0.000001f;
					state->density_back[i] = state->density[i];

					i++;
				}
			}
		}
	}

	{
		int i=0;
		glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
		for (size_t y=0;y<dim2.y;y++) {
			for (size_t x=0;x<dim2.x;x++, i++) {
				glm::vec2 coord = glm::vec2(x, y);
				glm::vec2 norm = coord/glm::vec2(dim2);
				glm::vec2 snorm = norm*2.f-1.f;

				// plane height (in normed 0..1 to LAND_DIM):
				float w = glm::abs(sin(M_PI * snorm.x)*snorm.y*0.15) + 0.01;
				state->land[i].w = w;
			}
		}
	}
	{
		// generate SDF from state->land height:
		int i=0;
		glm::ivec3 dim = glm::ivec3(LAND_DIM, LAND_DIM, LAND_DIM);
		glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
		for (size_t z=0;z<dim.z;z++) {
			for (size_t y=0;y<dim.y;y++) {
				for (size_t x=0;x<dim.x;x++) {
					glm::vec3 coord = glm::vec3(x, y, z);
					glm::vec3 norm = coord/glm::vec3(dim);
					
					int ii = al_field2d_index(dim2, glm::ivec2(x, z));
					float w = state->land[ ii ].w;

					state->distance[i] = norm.y < w ? -1. : 1.;
					state->distance_binary[i] = state->distance[i] < 0.f ? 0.f : 1.f;

					i++;
				}
			}
		}
		sdf_from_binary(land_dim, state->distance_binary, state->distance);
		//sdf_from_binary_deadreckoning(land_dim, state->distance_binary, state->distance);
		al_field3d_scale(land_dim, state->distance, 1.f/land_dim.x);
	}
	{
		// generate state->land normals:
		int i=0;
		glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
		for (size_t y=0;y<dim2.y;y++) {
			for (size_t x=0;x<dim2.x;x++, i++) {
				glm::vec2 coord = glm::vec2(x, y);
				glm::vec2 norm = coord/glm::vec2(dim2);
				glm::vec2 snorm = norm*2.f-1.f;

				float w = state->land[i].w;

				glm::vec3 norm3 = glm::vec3(norm.x, w, norm.y);

				glm::vec3 normal = sdf_field_normal4(land_dim, state->distance, norm3, 2.f/LAND_DIM);
				state->land[i] = glm::vec4(normal, w);
			}
		}
	}

	
	

	onReloadGPU();

	threads_begin();
}

void test() {
	Timer timer;
	double t = 0.1;

	al_sleep(t);
	console.log("slept, elapsed %f %f", t, timer.measure());
	t *= 0.1;

	al_sleep(t);
	console.log("slept, elapsed %f %f", t, timer.measure());
	t *= 0.1;

	al_sleep(t);
	console.log("slept, elapsed %f %f", t, timer.measure());
	t *= 0.1;

	al_sleep(t);
	console.log("slept, elapsed %f %f", t, timer.measure());
	t *= 0.1;
}


extern "C" {
    AL_EXPORT int onload() {

		//test();
    	
		Alice& alice = Alice::Instance();

		console.log("onload");
		console.log("sim alice %p", &alice);

		// import/allocate state
		state = statemap.create("state.bin", true);
		console.log("sim state %p should be size %d", state, sizeof(State));
		//state_initialize();
		console.log("onload state initialized");


		// TODO currently this is a memory leak on unload:
		fluid.initialize(FIELD_DIM, FIELD_DIM, FIELD_DIM);
		

		threads_begin();

		world2fluid = glm::scale(glm::vec3(0.1f));
		// how to convert the normalized coordinates of the fluid (0..1) into positions in the world:
		// this effectively defines the bounds of the fluid in the world:
		// from transform(fluid2world(glm::vec3(0.)))
		// to   transform(fluid2world(glm::vec3(1.)))
		fluid2world = glm::scale(glm::vec3(world_max.x - world_min.x));
		// how to convert world positions into normalized texture coordinates in the fluid field:
		world2fluid = glm::inverse(fluid2world);

		vive2world = glm::rotate(float(M_PI/2), glm::vec3(0,1,0)) * glm::translate(glm::vec3(0.f, 0.f, -3.f));
			//glm::rotate(M_PI/2., glm::vec3(0., 1., 0.));

		
		console.log("onload fluid initialized");
	
		gBuffer.dim = glm::ivec2(512, 512);
		alice.hmd->connect();
		if (alice.hmd->connected) {
			alice.desiredFrameRate = 90;
			gBuffer.dim = alice.hmd->fbo.dim;
		} else if (isPlatformWindows()) {
			gBuffer.dim.x = 1920;
			gBuffer.dim.y = 1080;
			//alice.streamer->init(gBuffer.dim);
		}
		console.log("gBuffer dim %d x %d", gBuffer.dim.x, gBuffer.dim.y);

		// allocate on GPU:
		onReloadGPU();
		
		console.log("onload onReloadGPU ran");

		// register event handlers 
		alice.onFrame.connect(onFrame);
		alice.onReloadGPU.connect(onReloadGPU);
		alice.onReset.connect(onReset);

		alice.onKeyEvent.connect(onKeyEvent);
		alice.window.position(45, 45);

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