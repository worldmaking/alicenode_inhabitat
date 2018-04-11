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

	double t = Alice::Instance().t;
	
	for (int i=0; i<NUM_OBJECTS; i++) {
		Object &o = state->objects[i];
		o.location = wrap(o.location + quat_uf(o.orientation)*0.05f, glm::vec3(-64.f, 0.f, -64.f), glm::vec3(64.f, 10.f, 64.f));	
		//o.location = glm::clamp(o.location + glm::ballRand(0.1f), glm::vec3(-20.f, 0.f, -20.f), glm::vec3(20.f, 10.f, 20.f));	
		o.orientation = safe_normalize(glm::slerp(o.orientation, o.orientation * quat_random(), 0.025f));
		
	}
	
	// upload GPU;
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * NUM_TRIS, &state->translations[0], GL_STATIC_DRAW);
	
	// update nav
	double a = M_PI * t / 3.;
	glm::mat4 viewMat = glm::lookAt(
		glm::vec3(16.*sin(a), 10.*(1.+cos(a)), 32.*cos(a)), 
		glm::vec3(0., 0., 0.), 
		glm::vec3(0., 1., 0.));
	glm::mat4 projMat = glm::perspective(45.0f, 4.f/3.f, 0.1f, 100.0f);
	glm::mat4 viewProjMat = projMat * viewMat;
	glm::mat4 viewProjMatInverse = glm::inverse(viewProjMat);
	
	// start rendering:
	glDisable(GL_DEPTH_TEST);
	
	landShader->use();
    landShader->uniform("time", t);
    landShader->uniform("uViewMatrix", viewMat);
    landShader->uniform("uViewProjectionMatrix", viewProjMat);
    landShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
	quadMesh.draw();

	glEnable(GL_DEPTH_TEST);
	
	objectShader->use();
    objectShader->uniform("time", t);
    objectShader->uniform("uViewMatrix", viewMat);
    objectShader->uniform("uViewProjectionMatrix", viewProjMat);
    objectShader->uniform("uViewProjectionMatrixInverse", viewProjMatInverse);
    
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

<<<<<<< HEAD
=======
typedef double t_sample;

inline t_sample cubic_interp(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z) {
	const t_sample a2 = a*a;
	const t_sample f0 = z - y - w + x;
	const t_sample f1 = w - x - f0;
	const t_sample f2 = y - w;
	const t_sample f3 = x;
	return (f0*a*a2 + f1*a2 + f2*a + f3);
}

inline t_sample altcubic(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z) {
	const t_sample f0 = 1. + a; 
	
	const t_sample f1 = 1. - a; 
	
	const t_sample f2 = 2. - a; 
	
	const t_sample f3 = f1 * f2; 
	
	const t_sample f4 = f0 * a;
	
	const t_sample fw = -.1666667 * f3 * a; 
	
	const t_sample fx = .5 * f0 * f3; 
	
	const t_sample fy = .5 * f4 * f2; 
	
	const t_sample fz = -.1666667 * f4 * f1;
	
	return w * fw + x * fx + y * fy + z * fz;
}

// Breeuwsma catmull-rom spline interpolation
inline t_sample spline_interp(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z) {
	const t_sample a2 = a*a;
	const t_sample f0 = t_sample(-0.5)*w + t_sample(1.5)*x - t_sample(1.5)*y + t_sample(0.5)*z;
	const t_sample f1 = w - t_sample(2.5)*x + t_sample(2)*y - t_sample(0.5)*z;
	const t_sample f2 = t_sample(-0.5)*w + t_sample(0.5)*y;
	return (f0*a*a2 + f1*a2 + f2*a + x);
}

inline t_sample spline_harker(t_sample a, t_sample w, t_sample x, t_sample y, t_sample z)
{
    // N.B. - this is currently a high-quality cubic hermite
    const t_sample f0 = t_sample(0.5) * (z - w) + t_sample(1.5) * (x - y);
    const t_sample f1 = w - t_sample(2.5) * x + y + y - t_sample(0.5) * z;
    const t_sample f2 = t_sample(0.5) * (y - w);
    
    return (((f0 * a + f1) * a + f2) * a + x);
    //const t_sample a2 = a*a;
	//return (f0*a*a2 + f1*a2 + f2*a + x);
}

void test() {

	int count = 10000000000;
	int bufsize = 512;
	int bufwrap = bufsize-1;
	double buf[bufsize];
	for (int i=0; i<bufsize; i++) {
		buf[i] = glm::linearRand(0.f, 100000.f);
	}
	
	/*
	{
		double r = 0.;
		auto steady_start = std::chrono::steady_clock::now(); 
		for (int i=0; i<count; i++) {
			r += cubic_interp(buf[(i+67456) & bufwrap], buf[i & bufwrap], buf[(i+1) & bufwrap], buf[(i+2) & bufwrap], buf[(i+3) & bufwrap]);
		
		}
		auto steady_end = std::chrono::steady_clock::now();
		printf("cubic: %f %fs\n", r, std::chrono::duration<double>(steady_end - steady_start).count());
	}

	{
		double r = 0.;
		auto steady_start = std::chrono::steady_clock::now(); 
		for (int i=0; i<count; i++) {
			r += altcubic(buf[(i+67456) & bufwrap], buf[i & bufwrap], buf[(i+1) & bufwrap], buf[(i+2) & bufwrap], buf[(i+3) & bufwrap]);
		
		}
		auto steady_end = std::chrono::steady_clock::now();
		printf("altcubic: %f %fs\n", r, std::chrono::duration<double>(steady_end - steady_start).count());
	}

	{
		double r = 0.;
		auto steady_start = std::chrono::steady_clock::now(); 
		for (int i=0; i<count; i++) {
			r += spline_harker(buf[(i+67456) & bufwrap], buf[i & bufwrap], buf[(i+1) & bufwrap], buf[(i+2) & bufwrap], buf[(i+3) & bufwrap]);
		
		}
		auto steady_end = std::chrono::steady_clock::now();
		printf("spline_interp: %f %fs\n", r, std::chrono::duration<double>(steady_end - steady_start).count());
	}

	{
		double r = 0.;
		auto steady_start = std::chrono::steady_clock::now(); 
		for (int i=0; i<count; i++) {
			r += spline_interp(buf[(i+67456) & bufwrap], buf[i & bufwrap], buf[(i+1) & bufwrap], buf[(i+2) & bufwrap], buf[(i+3) & bufwrap]);
		
		}
		auto steady_end = std::chrono::steady_clock::now();
		printf("spline harker: %f %fs\n", r, std::chrono::duration<double>(steady_end - steady_start).count());
	}
	*/
	
	
	{
		double r = 0.;
		auto steady_start = std::chrono::steady_clock::now(); 
		for (int i=0; i<count; i++) {
			float a = spline_interp(buf[(i+67456) & bufwrap], buf[i & bufwrap], buf[(i+1) & bufwrap], buf[(i+2) & bufwrap], buf[(i+3) & bufwrap]);
			float b = spline_harker(buf[(i+67456) & bufwrap], buf[i & bufwrap], buf[(i+1) & bufwrap], buf[(i+2) & bufwrap], buf[(i+3) & bufwrap]);
			float err = fabs(a-b);
			if (err > 0.) {
				fprintf(stderr, "FAIL %f %f %f\n", a, err, b);
			}
		}
		auto steady_end = std::chrono::steady_clock::now();
		printf("spline harker: %f %fs\n", r, std::chrono::duration<double>(steady_end - steady_start).count());
	}

	/*for (int i=0; i<100000000; i++) {
		float a = glm::linearRand(-100000.f, 100000.f);
		float N = glm::linearRand(0.f, 1000.f);
		float b = wrap(a, -N, N);
		if (b < -N || b >= N) {
			fprintf(stderr, "FAIL %f %f %f\n", a, N, b);
		}
	}*/
}	

>>>>>>> client updated project
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
		
<<<<<<< HEAD
=======
		test();
		
>>>>>>> client updated project
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