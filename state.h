#ifndef STATE_H
#define STATE_H

#ifndef ALICE_H

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

#define NUM_DEBUGDOTS 512*424*2
//2*5*4

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
	glm::vec3 accel;
	glm::vec3 color;
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
	

	Particle particles[NUM_PARTICLES];
	Object objects[NUM_OBJECTS];
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
	
	void fluid_update(float dt);
};

#endif