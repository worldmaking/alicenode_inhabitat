#ifndef STATE_H
#define STATE_H

#define NUM_OBJECTS	24
#define NUM_PARTICLES 1024*64
#define FIELD_DIM 32
#define FIELD_VOXELS FIELD_DIM*FIELD_DIM*FIELD_DIM

struct Object {
	glm::vec3 location;
	glm::quat orientation;
};

struct Particle {
	glm::vec3 location;
	glm::vec3 color;
};

struct State {
	Particle particles[NUM_PARTICLES];
	Object objects[NUM_OBJECTS];
	

};

#endif