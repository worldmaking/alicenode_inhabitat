<<<<<<< HEAD
   #include "al/al_console.h"
=======
#include "al/al_console.h"
>>>>>>> 2542a2e5c4d6ab3d46a831f1193c87189d62b297
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
<<<<<<< HEAD
    -0.1f, -0.4f, 0.1f,
     0.5f, -0.5f, 0.2f,
     0.0f,  0.2f, 0.3f
=======
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
>>>>>>> 2542a2e5c4d6ab3d46a831f1193c87189d62b297
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
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * NUM_OBJECTS, &state->translations[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(2);
	// attr location, element size & type, normalize?, source stride & offset
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	// mark this attrib as being per-instance	
	glVertexAttribDivisor(2, 1);  

}

void onFrame() {

	double t = Alice::Instance().t;
	
	// update simulation:
	for (int i=0; i<NUM_OBJECTS; i++) {
		state->translations[i] = glm::clamp(state->translations[i] + glm::ballRand(0.1f), -10.f, 10.f);
	}
	
	// upload GPU;
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * NUM_OBJECTS, &state->translations[0], GL_STATIC_DRAW);
	
	// update nav
	double a = M_PI * t / 30.;
	glm::mat4 viewMat = glm::lookAt(
		glm::vec3(16.*sin(a), 10.*(1.2+cos(a)), 32.*cos(a)), 
		glm::vec3(0., 0., 0.), 
		glm::vec3(0., 1., 0.));
	glm::mat4 projMat = glm::perspective(45.0f, 4.f/3.f, 0.1f, 100.0f);
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
		state->translations[i] = glm::ballRand(1.f);
	}
}


void test() {

	for (int i=0; i<100000000; i++) {
		float a = glm::linearRand(-100000.f, 100000.f);
		float N = glm::linearRand(0.f, 1000.f);
		float b = wrap(a, -N, N);
		if (b < -N || b >= N) {
			fprintf(stderr, "FAIL %f %f %f\n", a, N, b);
		}
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