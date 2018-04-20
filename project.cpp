#include "al/al_console.h"
#include "al/al_math.h"
#include "al/al_field3d.h"
#include "al/al_gl.h"
#include "al/al_kinect2.h"
#include "al/al_mmap.h"
#include "alice.h"
#include "state.h"

struct GBuffer {

	static const int numBuffers = 3;

	unsigned int fbo;
	unsigned int rbo;
	//unsigned int gColor, gNormal, gPosition;
	unsigned int textures[numBuffers];
	unsigned int attachments[numBuffers];

	glm::ivec2 dim = glm::ivec2(1920, 1080);
	

	void dest_changed() {
		dest_closing();

		GLint min_filter = GL_LINEAR; //GL_NEAREST;
		GLint mag_filter = GL_LINEAR; //GL_NEAREST;

		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);

		glGenTextures(numBuffers, textures);

		// Buffer 0: color buffer
		glBindTexture(GL_TEXTURE_2D, textures[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dim.x, dim.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textures[0], 0);
		attachments[0] = GL_COLOR_ATTACHMENT0;
		
		// Buffer 1: normal
		glBindTexture(GL_TEXTURE_2D, textures[1]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, dim.x, dim.y, 0, GL_RGB, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, textures[1], 0);
		attachments[1] = GL_COLOR_ATTACHMENT1;

		// Buffer 2: position
		glBindTexture(GL_TEXTURE_2D, textures[2]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, dim.x, dim.y, 0, GL_RGB, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);  
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, textures[2], 0);
		attachments[2] = GL_COLOR_ATTACHMENT2;
		
		// set up the draw buffer attachments:
		glDrawBuffers(numBuffers, attachments);
		// create & attach depth buffer
		glGenRenderbuffers(1, &rbo);
		glBindRenderbuffer(GL_RENDERBUFFER, rbo);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, dim.x, dim.y);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);
		// finally check if framebuffer is complete
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
			std::cout << "Framebuffer not complete!" << std::endl;
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
			glDeleteTextures(3, textures);
			textures[0] = 0;
		}
	}

	void bindTextures() {
		for (int i=numBuffers-1; i>=0; i--) {
        	glActiveTexture(GL_TEXTURE0+i);
        	glBindTexture(GL_TEXTURE_2D, textures[i]);
		}
	}

	void unbindTextures() {
        for (int i=numBuffers-1; i>=0; i--) {
        	glActiveTexture(GL_TEXTURE0+i);
        	glBindTexture(GL_TEXTURE_2D, 0);
		}
	}
};

Shader * objectShader;
unsigned int objectVAO;
unsigned int objectVBO;
unsigned int objectInstanceVBO;

unsigned int particlesVAO;
unsigned int particlesVBO;
float particleSize = 1.f/64;
float near_clip = 0.1f;
float far_clip = 64.f;

Shader * particleShader;
Shader * landShader;
Shader * deferShader;
QuadMesh quadMesh;
SimpleFBO fbo;
GBuffer gBuffer;


glm::mat4 viewMat;
glm::mat4 projMat;
glm::mat4 viewProjMat;
glm::mat4 viewMatInverse;
glm::mat4 projMatInverse;
glm::mat4 viewProjMatInverse;

double fluid_viscosity, fluid_diffusion, fluid_decay, fluid_boundary_damping, fluid_noise;
Fluid3D<> fluid;
int fluid_passes = 14;
int fluid_noise_count = 32;

glm::vec4 boundary[FIELD_VOXELS];

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

	// clear gradients:
	fluid.gradient.front().zero();
	fluid.gradient.back().zero();

}

float vertices[] = {
    -1.0f,-1.0f,-1.0f, 
    -1.0f,-1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f, 
    
    1.0f, 1.0f,-1.0f, 
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f, 
    
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    
    -1.0f, 1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    
    1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f
};

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
	if (deferShader) {
		delete deferShader;
		deferShader = 0;
	}	
	
	quadMesh.dest_closing();
	fbo.dest_closing();
	gBuffer.dest_closing();
	
	if (objectVAO) {
		glDeleteVertexArrays(1, &objectVAO);
		objectVAO = 0;
	}
	if (objectVBO) {
		glDeleteBuffers(1, &objectVBO);
		objectVBO = 0;
	}
	if (objectInstanceVBO) {	
		glDeleteBuffers(1, &objectInstanceVBO);
		objectInstanceVBO = 0;
	}

	if (particlesVAO) {
		glDeleteVertexArrays(1, &particlesVAO);
		particlesVAO = 0;
	}
	if (particlesVBO) {
		glDeleteBuffers(1, &particlesVBO);
		particlesVBO = 0;
	}

}

void onReloadGPU() {

	onUnloadGPU();
	
	landShader = Shader::fromFiles("land.vert.glsl", "land.frag.glsl");
	particleShader = Shader::fromFiles("particle.vert.glsl", "particle.frag.glsl");
	objectShader = Shader::fromFiles("object.vert.glsl", "object.frag.glsl");
	deferShader = Shader::fromFiles("defer.vert.glsl", "defer.frag.glsl");
	
	quadMesh.dest_changed();
	fbo.dest_changed();
	gBuffer.dest_changed();

	{
		// define the VAO 
		// (a VAO stores attrib & buffer mappings in a re-usable way)
		glGenVertexArrays(1, &objectVAO); 
		glBindVertexArray(objectVAO);
		// define the VBO while VAO is bound:
		glGenBuffers(1, &objectVBO); 
		glBindBuffer(GL_ARRAY_BUFFER, objectVBO);  
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		// attr location 
		glEnableVertexAttribArray(0); 
		// set the data layout
		// attr location, element size & type, normalize?, source stride & offset
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); 

		glGenBuffers(1, &objectInstanceVBO);
		glBindBuffer(GL_ARRAY_BUFFER, objectInstanceVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Object) * NUM_OBJECTS, &state->objects[0], GL_STATIC_DRAW);

		glEnableVertexAttribArray(2);
		// attr location, element size & type, normalize?, source stride & offset
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Object), (void*)offsetof(Object, location));
		// mark this attrib as being per-instance	
		glVertexAttribDivisor(2, 1);  
		
		glEnableVertexAttribArray(3);
		// attr location, element size & type, normalize?, source stride & offset
		glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Object), (void*)offsetof(Object, orientation));
		// mark this attrib as being per-instance	
		glVertexAttribDivisor(3, 1);  
		
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	{
		// define the VAO
		// (a VAO stores attrib & buffer mappings in a re-usable way)
		glGenVertexArrays(1, &particlesVAO);
		glBindVertexArray(particlesVAO);

		// define the VBO while VAO is bound:
		glGenBuffers(1, &particlesVBO);
		glBindBuffer(GL_ARRAY_BUFFER, particlesVBO);
		//glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Particle) * NUM_PARTICLES, &state->particles[0], GL_STATIC_DRAW);

		// attr location 
		glEnableVertexAttribArray(0);
		// attr location, element size & type, normalize?, source stride & offset
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (void*)(offsetof(Particle, location)));

		// attr location 
		glEnableVertexAttribArray(1);
		// attr location, element size & type, normalize?, source stride & offset
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (void*)(offsetof(Particle, color)));
		
		//glEnableVertexAttribArray(1);
		// attr location, element size & type, normalize?, source stride & offset
		//glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Particle), (void*)(offsetof(Particle, color)));

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}


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

	glBindVertexArray(objectVAO);
	// draw instances:
	glDrawArraysInstanced(GL_TRIANGLES, 0, sizeof(vertices) / (sizeof(float) * 3), NUM_OBJECTS);  

	particleShader->use(); 
	particleShader->uniform("time", t);
	particleShader->uniform("uViewMatrix", viewMat);
	particleShader->uniform("uViewMatrixInverse", viewMatInverse);
	particleShader->uniform("uProjectionMatrix", projMat);
	particleShader->uniform("uViewProjectionMatrix", viewProjMat);
	particleShader->uniform("uViewPortHeight", (float)height);
	particleShader->uniform("uPointSize", particleSize);

	glBindVertexArray(particlesVAO);
	// draw instances:
	glEnable( GL_PROGRAM_POINT_SIZE );
	glEnable(GL_POINT_SPRITE);
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glDrawArrays(GL_POINTS, 0, NUM_PARTICLES);	
	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDisable(GL_POINT_SPRITE);
	glBindVertexArray(0);
}

void onFrame(uint32_t width, uint32_t height) {
	const Alice& alice = Alice::Instance();
	double t = alice.simTime;
	float aspect = width/float(height);

	if (alice.framecount % 60 == 0) {
        console.log("fps %f", alice.fpsAvg);
    }

	if (Alice::Instance().isSimulating) {

		// update simulation:
		fluid_update();

		const glm::vec3 * camera_points = alice.cloudDevice.captureFrame.xyz;
		uint64_t max_camera_points = sizeof(alice.cloudDevice.captureFrame.xyz)/sizeof(glm::vec3);
		
		for (int i=0; i<NUM_PARTICLES; i++) {
			Particle &o = state->particles[i];

			glm::vec3 flow;
			fluid.velocities.front().read_interp(o.location, &flow.x);
			
			glm::vec3 noise;// = glm::sphericalRand(0.02f);

			o.location = wrap(
				o.location + flow + noise, 
				glm::vec3(-20.f, 0.f, -20.f), 
				glm::vec3(20.f, 10.f, 20.f));

			glm::vec3 p = camera_points[i % max_camera_points];
			// this is in meters, but that seems a bit limited for our world
			glm::vec3 campos = glm::vec3(0., 2., 0.);
			p = p + campos;
			p *= 2.;
			o.location = p;
			o.color = o.location;
			//camera_points[i % max_camera_points];
		}

		for (int i=0; i<NUM_OBJECTS; i++) {
			Object &o = state->objects[i];

			
			//o.location = wrap(o.location + quat_uf(o.orientation)*0.05f, glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
			//o.location = glm::clamp(o.location + glm::ballRand(0.1f), glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
			o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.05f));
			


			glm::vec3 flow;
			fluid.velocities.front().read_interp(o.location, &flow.x);
			
			float creature_speed = 4.f*(float)alice.dt;
			glm::vec3 push = quat_uf(o.orientation) * creature_speed;
			fluid.velocities.front().add(o.location, &push.x);

			o.location = wrap(o.location + flow, glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));



			//state->particles[i].location = o.location;

		}
	}

	// upload GPU;
	glBindBuffer(GL_ARRAY_BUFFER, objectInstanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Object) * NUM_OBJECTS, &state->objects[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, particlesVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Particle) * NUM_PARTICLES, &state->particles[0], GL_STATIC_DRAW);

	// update nav
	double a = M_PI * t / 30.;
	viewMat = glm::lookAt(
		glm::vec3(16.*sin(a), 10.*(1.2+cos(a)), 32.*cos(a)), 
		glm::vec3(0., 0., 0.), 
		glm::vec3(0., 1., 0.));
	projMat = glm::perspective(45.0f, aspect, near_clip, far_clip);
	viewProjMat = projMat * viewMat;

	projMatInverse = glm::inverse(projMat);
	viewMatInverse = glm::inverse(viewMat);
	viewProjMatInverse = glm::inverse(viewProjMat);

	// start rendering:
	if (1) {
		glBindFramebuffer(GL_FRAMEBUFFER, gBuffer.fbo);
		glEnable(GL_SCISSOR_TEST);
		glScissor(0, 0, gBuffer.dim.x, gBuffer.dim.y);
		glViewport(0, 0, gBuffer.dim.x, gBuffer.dim.y);
		glEnable(GL_DEPTH_TEST);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		draw_scene(gBuffer.dim.x, gBuffer.dim.y);
		glDisable(GL_SCISSOR_TEST);
		glBindFramebuffer(GL_FRAMEBUFFER, 0); // end
		//glGenerateMipmap(GL_TEXTURE_2D); // not sure if we need this

		glViewport(0, 0, width, height);
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.f, 0.f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		deferShader->use();
		deferShader->uniform("uViewMatrix", viewMat);
		deferShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
		deferShader->uniform("gColor", 0);
		deferShader->uniform("gNormal", 1);
		deferShader->uniform("gPosition", 2);
		deferShader->uniform("uNearClip", near_clip);
		deferShader->uniform("uFarClip", far_clip);
		deferShader->uniform("uDim", glm::vec2(gBuffer.dim.x, gBuffer.dim.y));
		gBuffer.bindTextures();
		quadMesh.draw();
		gBuffer.unbindTextures();
		deferShader->unuse();

		/*
		// copy gBuffer depth the main depth buffer, 
        glBindFramebuffer(GL_READ_FRAMEBUFFER, gBuffer.fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); // write to default framebuffer
        // blit to default framebuffer. Note that this may or may not work as the internal formats of both the FBO and default framebuffer have to match.
        // the internal formats are implementation defined. This works on all of my systems, but if it doesn't on yours you'll likely have to write to the 		
        // depth buffer in another shader stage (or somehow see to match the default framebuffer's internal format with the FBO's internal format).
        glBlitFramebuffer(0, 0, gBuffer.dim.x, gBuffer.dim.y, 0, 0, width, height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
		*/
	}
}


void onReset() {
	for (int i=0; i<NUM_OBJECTS; i++) {
		state->objects[i].location = glm::ballRand(1.f);
	}
	for (int i=0; i<NUM_PARTICLES; i++) {
		state->particles[i].location = glm::ballRand(1.f);
		state->particles[i].color = glm::vec3(1.f);
	}
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
		console.log("initialized");


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

		fbo.dim.x = 1920;
		fbo.dim.y = 1080;

		// allocate on GPU:
		onReloadGPU();

		// register event handlers 
		alice.onFrame.connect(onFrame);
		alice.onReloadGPU.connect(onReloadGPU);
		alice.onReset.connect(onReset);

		//test();

		return 0;
    }
    
    AL_EXPORT int onunload() {
		Alice& alice = Alice::Instance();

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