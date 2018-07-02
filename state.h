#ifndef STATE_H
#define STATE_H

#ifndef ALICE_H
// for the use of Clang-Index:
#include <stddef.h>
namespace glm {

	struct vec2 { float x, y; vec2(float, float); };
	struct vec3 { float x, y, z; vec3(float, float, float); };
	struct vec4 { float x, y, z, w; vec4(float, float, float, float); };
	struct quat { float x, y, z, w; quat(float, float, float, float); };

	struct ivec2 { int x, y; ivec2(int, int); };
	struct ivec3 { int x, y, z; ivec3(int, int, int); };
	struct ivec4 { int x, y, z, w; ivec4(int, int, int, int); };
}
#endif

#define NUM_PREDATORS 124
#define NUM_CREATURES NUM_PREDATORS

#define PREDATOR_SEGMENTS_EACH 1
#define NUM_SEGMENTS (NUM_PREDATORS*PREDATOR_SEGMENTS_EACH)

#define NUM_OBJECTS	32
#define NUM_PARTICLES 1024*256

#define NUM_DEBUGDOTS 2*5*4
//2*5*4

#define NUM_TELEPORT_POINTS 10

#define FIELD_DIM 32
#define FIELD_TEXELS FIELD_DIM*FIELD_DIM
#define FIELD_VOXELS FIELD_DIM*FIELD_DIM*FIELD_DIM

#define LAND_DIM 200
#define LAND_TEXELS LAND_DIM*LAND_DIM
#define LAND_VOXELS LAND_DIM*LAND_DIM*LAND_DIM

#define FUNGUS_DIM 512
#define FUNGUS_TEXELS FUNGUS_DIM*FUNGUS_DIM

static const glm::ivec3 field_dim = glm::ivec3(FIELD_DIM, FIELD_DIM, FIELD_DIM);
static const glm::ivec3 land_dim = glm::ivec3(LAND_DIM, LAND_DIM, LAND_DIM);
static const glm::ivec2 fungus_dim = glm::ivec2(FUNGUS_DIM, FUNGUS_DIM);

template<int DIM=32, typename T=float>
struct Field3DPod {

	Field3DPod() { reset(); }

	void reset() {
		memset(data0, 0, sizeof(data0));
		memset(data1, 0, sizeof(data1));
	}

	T * data(bool back=false) { return (!isSwapped != !back) ? data1 : data0; }
	const T * data(bool back=false) const { return (!isSwapped != !back) ? data1 : data0; }

	T * front() { return data(0); }
	T * back() { return data(1); }

	size_t length() const { return DIM*DIM*DIM; }
	glm::ivec3 dim() const { return glm::ivec3(DIM, DIM, DIM); }

	void swap() { isSwapped = !isSwapped; }

	T data0[DIM*DIM*DIM];
	T data1[DIM*DIM*DIM];
	int isSwapped = 0;
};

template<int DIM=32>
struct Fluid3DPod {

	Fluid3DPod() { reset(); }

	void reset() {
		velocities.reset();
		gradient.reset();
	}

	size_t length() const { return DIM*DIM*DIM; }
	glm::ivec3 dim() const { return glm::ivec3(DIM, DIM, DIM); }

	// TODO: I guess a clever thing would be to use vec4, xyz=velocity, w=gradient... 
	Field3DPod<DIM, glm::vec3> velocities;
	Field3DPod<DIM, float> gradient;

	bool isFlipped = 0;
};

struct Object {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	float phase;
	glm::vec3 velocity;
	glm::vec3 color;
	
	glm::vec3 accel;
};

struct Segment {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	float phase;
	glm::vec3 velocity;
	glm::vec3 color;
};

struct Particle {
	glm::vec3 location;
	glm::vec3 color;
	glm::vec3 velocity;
	float phase, unused;
};

struct DebugDot {
	glm::vec3 location;
	glm::vec3 color;
};

struct State {

	float dummy = 10;
	float test = 34;
	
	// for simulation:
	Object objects[NUM_OBJECTS];

	// for rendering:
	Particle particles[NUM_PARTICLES];
	Segment segments[NUM_SEGMENTS];
	DebugDot debugdots[NUM_DEBUGDOTS];

	// the density field is currently being used to store emissive light (as a form of smell)
	glm::vec3 density[FIELD_VOXELS];
	glm::vec3 density_back[FIELD_VOXELS];

	// the basic height field
	// .xyz represents the normal
	// .w represents the height
	glm::vec4 land[LAND_TEXELS];

	// signed distance field representing the landscape
	// the distance to the nearest land surface, as a 3D SDF
	// scaled such that the distance across the entire space == 1
	// distances are normalized over the LAND_DIM as 0..1
	float distance[LAND_VOXELS];
	// the boolean field that is used to generate the distance field
	// surface edges are marked by unequal neighbour values
	float distance_binary[LAND_VOXELS];

	// the state of the lichen CA over the world
	float fungus[FUNGUS_TEXELS];
	float fungus_old[FUNGUS_TEXELS];

	// the fluid simulation:
	Fluid3DPod<> fluidpod;

	// transforms:
	glm::vec3 world_min = glm::vec3(0.f, 0.f, 0.f);
	glm::vec3 world_max = glm::vec3(80.f, 80.f, 80.f);
	glm::vec3 world_centre = glm::vec3(40.f, 18.f, 40.f);
	float field2world_scale;
	glm::mat4 world2field;
	glm::mat4 field2world;
	glm::mat4 vive2world;
	glm::mat4 kinect2world; 
	glm::mat4 leap2view;
	glm::mat4 world2minimap;
	float minimapScale = 0.005f;
	float kinect2world_scale = 10.f;
	float near_clip = 0.02f;
	float far_clip = 1200.f;

	// parameters:

	int fluid_passes = 14;
	int fluid_noise_count = 32;
	float fluid_decay = 0.9999f;
	double fluid_viscosity = 0.00000001; //0.00001;
	double fluid_boundary_damping = .2;
	double fluid_noise = 8.;

	float density_decay = 0.98f;
	float density_diffuse = 0.01; // somwhere between 0.1 and 0.01 seems to be good
	float density_scale = 0.5;

	float particleSize = 0.005;
	float creature_fluid_push = 0.75f;
	
	void fluid_update(float dt);
};

#endif