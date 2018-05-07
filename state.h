#ifndef STATE_H
#define STATE_H

#define NUM_SEGMENTS 64
#define NUM_OBJECTS	32
#define NUM_PARTICLES 1024*256
#define FIELD_DIM 32
#define FIELD_VOXELS FIELD_DIM*FIELD_DIM*FIELD_DIM

struct Object {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	float phase;
	glm::vec3 velocity;
};

struct Segment {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	float phase;
	glm::vec3 velocity;
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
};

#endif