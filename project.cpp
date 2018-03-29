#include "al/al_console.h"
#include "al/al_math.h"
#include "al/al_gl.h"
#include "al/al_mmap.h"
#include "alice.h"

#include "state.h"


Shader * shader_test;
unsigned int VAO;
unsigned int VBO;
unsigned int instanceVBO;

Shader * landShader;
QuadMesh quadMesh;

float vertices[] = {
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
     0.0f,  0.9f, 0.0f
};

State * state;
Mmap<State> statemap;


void onUnloadGPU() {
	// free resources:
	if (shader_test) {
		delete shader_test;
		shader_test = 0;
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
	
	shader_test = Shader::fromFiles("test.vert.glsl", "test.frag.glsl");
	if (!shader_test) return;
	console.log("shader loaded %p = %d", shader_test, (int)shader_test->program);
	
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
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * NUM_TRIS, &state->translations[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(2);
	// attr location, element size & type, normalize?, source stride & offset
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	// mark this attrib as being per-instance	
	glVertexAttribDivisor(2, 1);  

}

void onFrame() {
	
	// update simulation:
	{
		int i = rnd::integer(NUM_TRIS);
		float y = state->translations[i].y;
		y = y - 0.1f;
		if (y > 1.) y -= 2.;
		if (y < -1.) y += 2.;
		state->translations[i].y = y;
	}
	for (int i=0; i<NUM_TRIS; i++) {
		float y = state->translations[i].y;
		y = y * 0.99f;
		if (y > 1.) y -= 2.;
		if (y < -1.) y += 2.;
		state->translations[i].y = y;
	}
	
	// upload GPU;
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * NUM_TRIS, &state->translations[0], GL_STATIC_DRAW);
	
	// update nav
	glm::mat4 viewMat = glm::lookAt(glm::vec3(0., 20., 24.), glm::vec3(0., 0., 0.), glm::vec3(0., 1., 0.));
	glm::mat4 projMat = glm::perspective(45.0f, 4.f/3.f, 0.1f, 100.0f);
	glm::mat4 viewMatInverse = glm::inverse(viewMat);
	glm::mat4 projMatInverse = glm::inverse(projMat);
	// start rendering:
	
	landShader->use();
	//quadMesh.draw();
	
	shader_test->use();
    shader_test->uniform("time", Alice::Instance().t);
    shader_test->uniform("uViewMatrix", viewMat);
    shader_test->uniform("uProjectionMatrix", projMat);
    
    glBindVertexArray(VAO);
    // offset, vertex count
    //glDrawArrays(GL_TRIANGLES, 0, 3);
    // draw instances:
    glDrawArraysInstanced(GL_TRIANGLES, 0, 3, NUM_TRIS);  
}


void state_initialize() {
	for (int i=0; i<NUM_TRIS; i++) {
		state->translations[i] = glm::diskRand(1.f);
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