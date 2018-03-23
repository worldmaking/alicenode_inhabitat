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

float vertices[] = {
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
     0.0f,  0.9f, 0.0f
};

State * state;
Mmap<State> statemap;

State * state1;
Mmap<State> statemap1;

void onFrame() {
	int i = rnd::integer(NUM_TRIS);
	float y = state1->translations[i].y;
	y = y + 0.002f;
	if (y > 1.) y -= 2.;
	if (y < -1.) y += 2.;
	state1->translations[i].y = y;
	
	// update GPU;
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * NUM_TRIS, &state->translations[0], GL_STATIC_DRAW);
	
	shader_test->use();
    shader_test->uniform("time", Alice::Instance().t);
    
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
    	state1 = statemap1.create("state.bin", true);
    	console.log("sim state %p should be size %d", state, sizeof(State));
    	//state_initialize();
    	console.log("initialized");
    	
    	shader_test = Shader::fromFiles("test.vert.glsl", "test.frag.glsl");
    	console.log("ok");
		console.log("shader loaded %p = %d", shader_test, (int)shader_test->program);
		if (!shader_test) return 0;
		{
			
		
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
		
		
		// register event handlers 
		alice.onFrame.connect(onFrame);
		
		console.log("helloo!oooo\n");
    
        return 0;
    }
    
    AL_EXPORT int onunload() {
    
    	// free resources:
    	delete shader_test;
    	shader_test = 0;
    	
    	
    	glDeleteVertexArrays(1, &VAO);
    	VAO = 0;
    	
    	glDeleteBuffers(1, &VBO);
    	VBO = 0;
    	
    	glDeleteBuffers(1, &instanceVBO);
    	instanceVBO = 0;
    	
    	
    	// unregister handlers
    	Alice::Instance().onFrame.disconnect(onFrame);
    	
    	// export/free state
    	statemap.destroy(true);
    	statemap1.destroy();
    
        return 0;
    }
}