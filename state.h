#ifndef STATE_H
#define STATE_H

#define NUM_SEGMENTS 64
#define NUM_OBJECTS	32
#define NUM_PARTICLES 1024*256
#define FIELD_DIM 32
#define FIELD_TEXELS FIELD_DIM*FIELD_DIM
#define FIELD_VOXELS FIELD_DIM*FIELD_DIM*FIELD_DIM

#define LAND_DIM 128
#define LAND_TEXELS LAND_DIM*LAND_DIM
#define LAND_VOXELS LAND_DIM*LAND_DIM*LAND_DIM

static const glm::ivec3 field_dim = glm::vec3(FIELD_DIM, FIELD_DIM, FIELD_DIM);
static const glm::ivec3 land_dim = glm::vec3(LAND_DIM, LAND_DIM, LAND_DIM);

struct Object {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	float phase;
	glm::vec3 velocity;
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
};

struct State {
	Particle particles[NUM_PARTICLES];
	Object objects[NUM_OBJECTS];
	Segment segments[NUM_SEGMENTS];

	// the density field is currently being used to store emissive light (as a form of smell)
	glm::vec3 density[FIELD_VOXELS];
	glm::vec3 density_back[FIELD_VOXELS];

	// the basic height field
	// .w represents the height
	// .xyz represents the normal
	//glm::vec4 land[FIELD_TEXELS];


	//float landscape[FIELD_VOXELS];
	//float landscape_back[FIELD_VOXELS];

	// signed distance field representing the landscape
	// scaled such that the distance across the entire space == 1
	float distance[LAND_VOXELS];
	// the boolean field that is used to generate the distance field
	// surface edges are marked by unequal neighbour values
	float distance_binary[LAND_VOXELS];
};

#endif