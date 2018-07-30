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

void fields_update(double dt) { 
	if (Alice::Instance().isSimulating) state->fields_update(dt); 
}

void land_update(double dt) { 
	if (Alice::Instance().isSimulating) state->land_update(dt); 
	state->generate_land_sdf_and_normals();
}

void sim_update(double dt) { 
	if (Alice::Instance().isSimulating) state->sim_update(dt); 
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
