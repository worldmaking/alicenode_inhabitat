#include "al/al_console.h"
#include "al/al_math.h"
#include "al/al_gl.h"
#include "al/al_mmap.h"
#include "alice.h"
#include "state.h"

Shader * objectShader;
unsigned int VAO;
unsigned int VBO;
unsigned int instanceVBO;

Shader * landShader;
QuadMesh quadMesh;

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
	if (objectShader) {
		delete objectShader;
		objectShader = 0;
	}	
	
	quadMesh.dest_closing();
	
	if (VAO) {
		glDeleteVertexArrays(1, &VAO);
		VAO = 0;
	}
	if (VBO) {
		glDeleteBuffers(1, &VBO);
		VBO = 0;
	}
	if (instanceVBO) {	
		glDeleteBuffers(1, &instanceVBO);
		instanceVBO = 0;
	}
}

void onReloadGPU() {

	onUnloadGPU();
	
	landShader = Shader::fromFiles("land.vert.glsl", "land.frag.glsl");
	
	quadMesh.dest_changed();
	
	objectShader = Shader::fromFiles("object.vert.glsl", "object.frag.glsl");
	if (!objectShader) return;
	console.log("shader loaded %p = %d", objectShader, (int)objectShader->program);
	
	// define the VAO 
	// (a VAO stores attrib & buffer mappings in a re-usable way)
	glGenVertexArrays(1, &VAO); 
	glBindVertexArray(VAO);
	// define the VBO while VAO is bound:
	glGenBuffers(1, &VBO); 
	glBindBuffer(GL_ARRAY_BUFFER, VBO);  
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	// attr location 
	glEnableVertexAttribArray(0); 
	// set the data layout
	// attr location, element size & type, normalize?, source stride & offset
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); 

	glGenBuffers(1, &instanceVBO);
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
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

void onFrame(uint32_t width, uint32_t height) {

	double t = Alice::Instance().t;
	float aspect = width/float(height);

	// update simulation:
	for (int i=0; i<NUM_OBJECTS; i++) {
		Object &o = state->objects[i];
		o.location = wrap(o.location + quat_uf(o.orientation)*0.05f, glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
		//o.location = glm::clamp(o.location + glm::ballRand(0.1f), glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
		o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.05f));

	}

	// upload GPU;
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Object) * NUM_OBJECTS, &state->objects[0], GL_STATIC_DRAW);

	// update nav
	double a = M_PI * t / 30.;
	glm::mat4 viewMat = glm::lookAt(
	glm::vec3(16.*sin(a), 10.*(1.2+cos(a)), 32.*cos(a)), 
	glm::vec3(0., 0., 0.), 
	glm::vec3(0., 1., 0.));
	glm::mat4 projMat = glm::perspective(45.0f, aspect, 0.1f, 100.0f);
	glm::mat4 viewProjMat = projMat * viewMat;
	glm::mat4 viewProjMatInverse = glm::inverse(viewProjMat);

	// start rendering:

	landShader->use();
	landShader->uniform("time", t);
	landShader->uniform("uViewProjectionMatrix", viewProjMat);
	landShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
	quadMesh.draw();

	objectShader->use();
	objectShader->uniform("time", t);
	objectShader->uniform("uViewMatrix", viewMat);
	objectShader->uniform("uViewProjectionMatrix", viewProjMat);
	objectShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);

	glBindVertexArray(VAO);
	// offset, vertex count
	//glDrawArrays(GL_TRIANGLES, 0, 3);
	// draw instances:
	glDrawArraysInstanced(GL_TRIANGLES, 0, sizeof(vertices) / (sizeof(float) * 3), NUM_OBJECTS);   
}


void state_initialize() {
	for (int i=0; i<NUM_OBJECTS; i++) {
		state->objects[i].location = glm::ballRand(1.f);
	}
}

typedef double t_sample;

t_sample tapcubic(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z) {
	t_sample f0 = 1. + a; 
	t_sample f1 = 1. - a; 
	t_sample f2 = 2. - a; 
	t_sample f3 = f1 * f2; 
	t_sample f4 = f0 * a;
	t_sample fw = -.1666667 * f3 * a; 
	t_sample fx = .5 * f0 * f3; 
	t_sample fy = .5 * f4 * f2; 
	t_sample fz = -.1666667 * f4 * f1;
	return w * fw + x * fx + y * fy + z * fz;
}

// worse performance :-(
t_sample mspcubic(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z) {
    t_sample t0 = (a - a*a) * t_sample(0.5);
    t_sample t1 = 1. + t0;
    t_sample t2 = t0 * t_sample(-0.333333333333333);

    t_sample fw = (t_sample (2.) - a) * t2; 
    t_sample fx = (t_sample (1.) - a) * t1; 
    t_sample fy = (                a) * t1; 
    t_sample fz = (t_sample (1.) + a) * t2;
    return w * fw + x * fx + y * fy + z * fz;
}

t_sample mspcubic1(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z) {
    t_sample t0 = (a - a*a) * t_sample(0.5);
    t_sample t1 = t_sample(1.) + t0;
    t_sample t2 = t0 * t_sample(-0.333333333333333);
	return (x + a*(y-x))*t1 + 
		   (t_sample(2.)*w + z + a*(z-w))*t2;
}

void test() {
	int rounds = 1000000000;
	int size = 512;
	int wrap = size-1;
	double buf[size];
	for (int i=0; i<size; i++) {
		buf[i]= glm::linearRand(0., 1.);
	}

	{
		auto t1 = std::chrono::system_clock::now();
		double r=0;
		for (int i=0; i<rounds; i++) {
			r += tapcubic(buf[(i) & wrap], buf[(i+1) & wrap], buf[(i+2) & wrap], buf[(i+3) & wrap], buf[(i+4) & wrap]);
		}
		auto t2 = std::chrono::system_clock::now();

		auto duration = (double)std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
		printf("tapcubic  %f (%f)\n", duration, r);
	}

	{
		auto t1 = std::chrono::system_clock::now();
		double r=0;
		for (int i=0; i<rounds; i++) {
			r += mspcubic1(buf[(i) & wrap], buf[(i+1) & wrap], buf[(i+2) & wrap], buf[(i+3) & wrap], buf[(i+4) & wrap]);
		}
		auto t2 = std::chrono::system_clock::now();

		auto duration = (double)std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
		printf("tapcubic1 %f (%f)\n", duration, r);
	}
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

		// allocate on GPU:
		onReloadGPU();

		// register event handlers 
		alice.onFrame.connect(onFrame);
		alice.onReloadGPU.connect(onReloadGPU);

		//test();

		return 0;
    }
    
    AL_EXPORT int onunload() {
    	// free resources:
    	onUnloadGPU();
    	
    	// unregister handlers
    	Alice::Instance().onFrame.disconnect(onFrame);
    	Alice::Instance().onReloadGPU.disconnect(onReloadGPU);
    	
    	// export/free state
    	statemap.destroy(true);
    
        return 0;
    }
}