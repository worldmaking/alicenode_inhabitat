#ifndef STATE_H
#define STATE_H

#define NUM_OBJECTS	64
#define NUM_PARTICLES 1024*256
#define FIELD_DIM 32
#define FIELD_VOXELS FIELD_DIM*FIELD_DIM*FIELD_DIM

struct Object {
	glm::vec3 location;
	glm::quat orientation;
};

struct Particle {
	glm::vec3 location;
};

struct State {
	Particle particles[NUM_PARTICLES];
	Object objects[NUM_OBJECTS];
	

};

#endif