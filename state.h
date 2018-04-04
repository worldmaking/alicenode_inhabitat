#ifndef STATE_H
#define STATE_H

#define NUM_OBJECTS 5000

struct Object {
	glm::vec3 location;
	glm::quat orientation;
};

struct State {

	Object objects[NUM_OBJECTS];
	
};

#endif