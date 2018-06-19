#include "al/al_console.h"
#include "al/al_math.h"
#include "al/al_field3d.h"
#include "al/al_gl.h"
#include "al/al_kinect2.h"
#include "al/al_mmap.h"
#include "al/al_hmd.h"
#include "al/al_time.h"
#include "alice.h"
#include "state.h"

VBO cubeVBO(sizeof(positions_cube), positions_cube);

Shader * objectShader;
VAO objectVAO;
VBO objectInstancesVBO(sizeof(State::objects));

Shader * segmentShader;
VAO segmentVAO;
VBO segmentInstancesVBO(sizeof(State::segments));

VAO particlesVAO;
VBO particlesVBO(sizeof(State::particles));

float particleSize = 1.f/160;
float near_clip = 0.1f;
float far_clip = 12.f;

glm::vec3 world_min(-4.f, 0.f, 0.f);
glm::vec3 world_max(4.f, 4.f, 8.f);
glm::vec3 world_centre(0.f, 1.8f, 2.f);
float world2fluid = 8.f;

glm::mat4 kinect2world; 

Shader * particleShader;
Shader * landShader;
Shader * deferShader;
QuadMesh quadMesh;
GLuint colorTex;

glm::mat4 viewMat;
glm::mat4 projMat;
glm::mat4 viewProjMat;
glm::mat4 viewMatInverse;
glm::mat4 projMatInverse;
glm::mat4 viewProjMatInverse;

double fluid_viscosity, fluid_diffusion, fluid_boundary_damping, fluid_noise;
Fluid3D<> fluid;
int fluid_passes = 14;
int fluid_noise_count = 32;
double fluid_sleep_s = 0.01;
float fluid_decay = 0.9999f;

glm::vec4 boundary[FIELD_VOXELS];

std::thread fluidThread;
bool isRunning = 1;

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
				glm::vec3 veln = safe_normalize(vel);
				float speed = glm::length(vel);


				const glm::vec3 normal = glm::vec3(land);	// already normalized.

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
}

void fluid_update() {
	// update fluid
	Field3D<>& velocities = fluid.velocities;
	const size_t stride0 = velocities.stride(0);
	const size_t stride1 = velocities.stride(1);
	const size_t stride2 = velocities.stride(2);
	const size_t dim0 = velocities.dimx();
	const size_t dim1 = velocities.dimy();
	const size_t dim2 = velocities.dimz();
	const size_t dimwrap0 = dim0 - 1;
	const size_t dimwrap1 = dim1 - 1;
	const size_t dimwrap2 = dim2 - 1;
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
	// apply boundaries:
	//apply_fluid_boundary2(data, boundary, dim0, dim1, dim2);
	//apply_fluid_boundary2(data, (glm::vec4 *)landscape.ptr(), dim0, dim1, dim2);
	velocities.front().scale(fluid_decay);

	// clear gradients:
	fluid.gradient.front().zero();
	fluid.gradient.back().zero();

}

void fluid_run() {
	static FPS fps;
	console.log("fluid thread started");
	while(isRunning) {
		fluid_update();
		al_sleep(fluid_sleep_s);

		if (fps.measure()) {
			console.log("sim fps %f", fps.fps);
		}
	}
	console.log("fluid thread ending");
}

State * state;
Mmap<State> statemap;


void onUnloadGPU() {
	// free resources:
	if (landShader) {
		delete landShader;
		landShader = 0;
	}	
	if (particleShader) {
		delete particleShader;
		particleShader = 0;
	}	
	if (objectShader) {
		delete objectShader;
		objectShader = 0;
	}
	if (segmentShader) {
		delete segmentShader;
		segmentShader = 0;
	}
	if (deferShader) {
		delete deferShader;
		deferShader = 0;
	}	
	
	quadMesh.dest_closing();
	cubeVBO.dest_closing();
	objectInstancesVBO.dest_closing();
	objectVAO.dest_closing();
	segmentInstancesVBO.dest_closing();
	segmentVAO.dest_closing();
	particlesVAO.dest_closing();
	particlesVAO.dest_closing();
	gBuffer.dest_closing();
	Alice::Instance().hmd->dest_closing();

	if (colorTex) {
		glDeleteTextures(1, &colorTex);
		colorTex = 0;
	}

	/*if (particlesVAO) {
		glDeleteVertexArrays(1, &particlesVAO);
		particlesVAO = 0;
	}
	if (particlesVBO) {
		glDeleteBuffers(1, &particlesVBO);
		particlesVBO = 0;
	}*/
}

void onReloadGPU() {

	onUnloadGPU();
	
	landShader = Shader::fromFiles("land.vert.glsl", "land.frag.glsl");
	particleShader = Shader::fromFiles("particle.vert.glsl", "particle.frag.glsl");
	objectShader = Shader::fromFiles("object.vert.glsl", "object.frag.glsl");
	segmentShader = Shader::fromFiles("segment.vert.glsl", "segment.frag.glsl");
	deferShader = Shader::fromFiles("defer.vert.glsl", "defer.frag.glsl");
	
	quadMesh.dest_changed();

	objectVAO.bind();
	cubeVBO.bind();
	objectVAO.attr(0, 3, GL_FLOAT, sizeof(glm::vec3), 0);
	objectInstancesVBO.bind();
	objectVAO.attr(2, &Object::location, true);
	objectVAO.attr(3, &Object::orientation, true);
	objectVAO.attr(4, &Object::scale, true);
	objectVAO.attr(5, &Object::phase, true);
		
	segmentVAO.bind();
	cubeVBO.bind();
	segmentVAO.attr(0, 3, GL_FLOAT, sizeof(glm::vec3), 0);
	segmentInstancesVBO.bind();
	segmentVAO.attr(2, &Segment::location, true);
	segmentVAO.attr(3, &Segment::orientation, true);
	segmentVAO.attr(4, &Segment::scale, true);
	segmentVAO.attr(5, &Segment::phase, true);

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
	
	landShader->use();
	landShader->uniform("time", t);
	landShader->uniform("uViewProjectionMatrix", viewProjMat);
	landShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
	landShader->uniform("uNearClip", near_clip);
	landShader->uniform("uFarClip", far_clip);
	quadMesh.draw();

	objectShader->use();
	objectShader->uniform("time", t);
	objectShader->uniform("uViewMatrix", viewMat);
	objectShader->uniform("uViewProjectionMatrix", viewProjMat);
	objectVAO.drawInstanced(sizeof(positions_cube) / sizeof(glm::vec3), NUM_OBJECTS);

	segmentShader->use();
	segmentShader->uniform("time", t);
	segmentShader->uniform("uViewMatrix", viewMat);
	segmentShader->uniform("uViewProjectionMatrix", viewProjMat);
	segmentVAO.drawInstanced(sizeof(positions_cube) / sizeof(glm::vec3), NUM_SEGMENTS);

	particleShader->use(); 
	particleShader->uniform("time", t);
	particleShader->uniform("uViewMatrix", viewMat);
	particleShader->uniform("uViewMatrixInverse", viewMatInverse);
	particleShader->uniform("uProjectionMatrix", projMat);
	particleShader->uniform("uViewProjectionMatrix", viewProjMat);
	particleShader->uniform("uViewPortHeight", (float)height);
	particleShader->uniform("uPointSize", particleSize);
	particleShader->uniform("uColorTex", 0);

	glBindTexture(GL_TEXTURE_2D, colorTex);
	glEnable( GL_PROGRAM_POINT_SIZE );
	glEnable(GL_POINT_SPRITE);
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	particlesVAO.draw(NUM_PARTICLES, GL_POINTS);
	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDisable(GL_POINT_SPRITE);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void onFrame(uint32_t width, uint32_t height) {
	const Alice& alice = Alice::Instance();
	double t = alice.simTime;
	double dt = alice.dt;
	float aspect = width/float(height);

	if (alice.framecount % 60 == 0) console.log("fps %f at %f", alice.fpsAvg, t);

	if (alice.isSimulating) {

		if (0) {
			glm::mat4 tr = glm::translate(glm::mat4(1.f), glm::vec3(-2.f, -2.f, -2.7f));
			float a = t;
			glm::mat4 ro = glm::rotate(glm::mat4(1.f), 1.8f, glm::vec3(1.f, 0.f, 0.f));
			kinect2world = ro * tr;
		}

		// get the most recent complete frame:
		const CloudFrame& cloudFrame = alice.cloudDevice->cloudFrame();
		//console.log("cloud frame %d", alice.cloudDevice->lastCloudFrame);
	
		// updates from simulation:
		const uint16_t * depth_points = cloudFrame.depth;
		const glm::vec3 * camera_points = cloudFrame.xyz;
		const glm::vec2 * uv_points = cloudFrame.uv;
		uint64_t max_camera_points = sizeof(cloudFrame.xyz)/sizeof(glm::vec3);

		const int midpoint = cDepthWidth*cDepthHeight * rnd::uni();
		//console.log("midpoint depth %d z %f", (int)(depth_points[midpoint]), camera_points[midpoint].z);
	
		{
			for (int i=0; i<NUM_PARTICLES; i++) {
				Particle &o = state->particles[i];

				glm::vec3 flow;
				fluid.velocities.front().read_interp(world2fluid * o.location, &flow.x);
				
				glm::vec3 noise;// = glm::sphericalRand(0.02f);

				o.location = wrap(
					o.location + world2fluid * flow + noise, world_min, world_max);

				o.phase += dt;

				if (alice.cloudDevice->capturing && rnd::uni() < 0.05f) {
					uint64_t idx = i % max_camera_points;
					glm::vec3 p = camera_points[idx];

					if (p.z > 0.9f) {

						p = glm::vec3(kinect2world * glm::vec4(p, 1.f));
						glm::vec2 uv = uv_points[idx];
						// this is in meters, but that seems a bit limited for our world
						glm::vec3 campos = glm::vec3(0., 0.8, 0.);
						p = p + campos;
						o.location = p;
						o.color = glm::vec3(uv, 0.5f);
						o.phase -= 1.f;
					}
				}
			}
		}
		
		for (int i=0; i<NUM_OBJECTS; i++) {
			auto &o = state->objects[i];

			//o.location = wrap(o.location + quat_uf(o.orientation)*0.05f, glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
			//o.location = glm::clamp(o.location + glm::ballRand(0.1f), glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
			o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.015f));
			
			glm::vec3 flow;
			fluid.velocities.front().read_interp(world2fluid * o.location, &flow.x);
			
			float creature_speed = 0.02f*(float)alice.dt;
			glm::vec3 push = quat_uf(o.orientation) * creature_speed;
			fluid.velocities.front().add(world2fluid * o.location, &push.x);

			o.location = wrap(o.location + world2fluid * flow, world_min, world_max);

			//state->particles[i].location = o.location;
		}

		for (int i=0; i<NUM_SEGMENTS; i++) {
			auto &o = state->segments[i];

			if (i % 8 == 0) {
				// a root;
				//o.location = wrap(o.location + quat_uf(o.orientation)*0.05f, glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
				//o.location = glm::clamp(o.location + glm::ballRand(0.1f), glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
				o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.015f));
				
				glm::vec3 flow;
				fluid.velocities.front().read_interp(world2fluid * o.location, &flow.x);
				
				float creature_speed = 0.02f*(float)alice.dt;
				glm::vec3 push = quat_uf(o.orientation) * creature_speed;
				fluid.velocities.front().add(world2fluid * o.location, &push.x);

				o.location = wrap(o.location + world2fluid * flow, world_min, world_max);

				//state->particles[i].location = o.location;
			} else {
				auto& p = state->segments[i-1];

				o.orientation = safe_normalize(glm::slerp(o.orientation, p.orientation, 0.015f));
				glm::vec3 uz = quat_uz(p.orientation);
				o.location = p.location + uz*o.scale;
				o.phase = p.phase + 0.1f;
				o.scale = p.scale * 0.9f;
			}

			
		}

		// upload VBO data to GPU:
		objectInstancesVBO.submit(&state->objects[0], sizeof(state->objects));
		segmentInstancesVBO.submit(&state->segments[0], sizeof(state->segments));
		particlesVBO.submit(&state->particles[0], sizeof(state->particles));
		// upload texture data to GPU:
		const ColourFrame& image = alice.cloudDevice->colourFrame();
		glBindTexture(GL_TEXTURE_2D, colorTex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cColorWidth, cColorHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image.color);
	}

	alice.hmd->update();

	{
		// draw the scene into the GBuffer:
		gBuffer.begin();
		glEnable(GL_SCISSOR_TEST);

		if (alice.hmd->connected) {		
			Hmd& vive = *alice.hmd;	
			vive.near_clip = near_clip;
			vive.far_clip = far_clip;
			for (int eye = 0; eye < 2; eye++) {
				glScissor(eye * gBuffer.dim.x / 2, 0, gBuffer.dim.x / 2, gBuffer.dim.y);
				glViewport(eye * gBuffer.dim.x / 2, 0, gBuffer.dim.x / 2, gBuffer.dim.y);
				glEnable(GL_DEPTH_TEST);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				// update nav
				viewMat = glm::inverse(vive.m_mat4viewEye[eye]) * glm::mat4_cast(glm::inverse(vive.mTrackedQuat)) * glm::translate(glm::mat4(1.f), -vive.mTrackedPosition);
				projMat = glm::frustum(vive.frustum[eye].l, vive.frustum[eye].r, vive.frustum[eye].b, vive.frustum[eye].t, vive.frustum[eye].n, vive.frustum[eye].f);

				viewProjMat = projMat * viewMat;
				projMatInverse = glm::inverse(projMat);
				viewMatInverse = glm::inverse(viewMat);
				viewProjMatInverse = glm::inverse(viewProjMat);

				draw_scene(gBuffer.dim.x / 2, gBuffer.dim.y);
			}
		} else {
			// No HMD:
			glScissor(0, 0, gBuffer.dim.x, gBuffer.dim.y);
			glViewport(0, 0, gBuffer.dim.x, gBuffer.dim.y);
			glEnable(GL_DEPTH_TEST);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// update nav
			double a = M_PI * t / 30.;
			viewMat = glm::lookAt(
				world_centre 
				+ glm::vec3(0.f, 0.f, -.5f)
				+  glm::vec3(.03*cos(a), .03*sin(0.5*a), .1*sin(a))
				, 
				world_centre, 
				glm::vec3(0., 1., 0.));
			projMat = glm::perspective(45.0f, aspect, near_clip, far_clip);
			
			viewProjMat = projMat * viewMat;
			projMatInverse = glm::inverse(projMat);
			viewMatInverse = glm::inverse(viewMat);
			viewProjMatInverse = glm::inverse(viewProjMat);

			draw_scene(gBuffer.dim.x, gBuffer.dim.y);
		}
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
			deferShader->use();
			deferShader->uniform("uViewMatrix", viewMat);
			deferShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
			deferShader->uniform("gColor", 0);
			deferShader->uniform("gNormal", 1);
			deferShader->uniform("gPosition", 2);
			deferShader->uniform("uNearClip", near_clip);
			deferShader->uniform("uFarClip", far_clip);
			deferShader->uniform("time", Alice::Instance().simTime);
			deferShader->uniform("uDim", glm::vec2(gBuffer.dim.x, gBuffer.dim.y));
			gBuffer.bindTextures();
			quadMesh.draw();
			gBuffer.unbindTextures();
			deferShader->unuse();
		}
		glDisable(GL_SCISSOR_TEST);
		fbo.end();

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
}

void onReset() {
	for (int i=0; i<NUM_OBJECTS; i++) {
		auto& o = state->objects[i];
		o.location = world_centre+glm::ballRand(1.f);
		o.phase = rnd::uni();
		o.scale = 0.25;
	}
	for (int i=0; i<NUM_SEGMENTS; i++) {
		auto& o = state->segments[i];
		o.location = world_centre+glm::ballRand(1.f);
		o.phase = rnd::uni();
		o.scale = 0.25;
	}
	for (int i=0; i<NUM_PARTICLES; i++) {
		auto& o = state->particles[i];
		o.location = world_centre+glm::ballRand(1.f);
		o.color = glm::vec3(1.f);
		o.phase = rnd::uni();
	}
}

template <typename T>
void printRatio(){ 
    std::cout << "  precision: " << T::num << "/" << T::den << " second " << std::endl;
    typedef typename std::ratio_multiply<T,std::kilo>::type MillSec;
    typedef typename std::ratio_multiply<T,std::mega>::type MicroSec;
    std::cout << std::fixed;
    std::cout << "             " << static_cast<double>(MillSec::num)/MillSec::den << " milliseconds " << std::endl;
    std::cout << "             " << static_cast<double>(MicroSec::num)/MicroSec::den << " microseconds " << std::endl;
}

void test() {

}

extern "C" {
    AL_EXPORT int onload() {
    	
		Alice& alice = Alice::Instance();

		console.log("onload");
		console.log("sim alice %p", &alice);

		// import/allocate state
		state = statemap.create("state.bin", true);
		console.log("sim state %p should be size %d", state, sizeof(State));
		//state_initialize();
		console.log("onload state initialized");

		// allow threads to run
		isRunning = true;

		// TODO currently this is a memory leak on unload:
		fluid.initialize(FIELD_DIM, FIELD_DIM, FIELD_DIM);
		for (int i = 0; i<FIELD_VOXELS; i++) {
			//glm::vec4 * n = (glm::vec4 *)noisefield[i];
			//*n = glm::linearRand(glm::vec4(0.), glm::vec4(1.));
			boundary[i] = glm::vec4(glm::sphericalRand(1.f), 1.f);
		}
		fluid_viscosity = 0.00001;
		fluid_boundary_damping = .2;
		fluid_noise_count = 32;
		fluid_noise = 8.;

		fluidThread = std::move(std::thread(fluid_run));

		console.log("onload fluid initialized");
		
		//alice.cloudDevice->record(1);

		alice.desiredFrameRate = 30;
		gBuffer.dim = glm::ivec2(512, 512);

		// let Alice know we want to use an HMD
		//alice.hmd->connect();
		
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

		test();

		return 0;
    }
    
    AL_EXPORT int onunload() {
		Alice& alice = Alice::Instance();

		// release threads:
		isRunning = false;
		console.log("joining threads");
		if (fluidThread.joinable()) fluidThread.join();

    	// free resources:
    	onUnloadGPU();
    	
    	// unregister handlers
    	alice.onFrame.disconnect(onFrame);
    	alice.onReloadGPU.disconnect(onReloadGPU);
		alice.onReset.disconnect(onReset);
    	
    	// export/free state
    	statemap.destroy(true);
    
        return 0;
    }
}