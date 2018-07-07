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

#define PREDATOR_SEGMENTS_EACH 1
#define NUM_SEGMENTS (NUM_PREDATORS*PREDATOR_SEGMENTS_EACH)

#define NUM_CREATURES 512
#define NUM_CREATURE_PARTS NUM_CREATURES

#define NUM_PARTICLES 1024*256
#define NUM_DEBUGDOTS 2*5*4
//2*5*4

#define NUM_TELEPORT_POINTS 10

#define NUM_AUDIO_FRAMES 1024

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

struct Creature {
	enum {
		STATE_BARDO = 0,
		STATE_ALIVE = 1,
		STATE_DECAYING = 2
	};
	enum {
		TYPE_NOTHING = 0,
		TYPE_ANT = 1,
		TYPE_BOID = 2,
		TYPE_BUG = 3,
		TYPE_PREDATOR_HEAD = 4,
		TYPE_PREDATOR_BODY = 5
	};

	// identity:
	int32_t idx = 0;
	int32_t type = TYPE_NOTHING;
	int32_t state = STATE_BARDO;
	// above zero means alive
	// above -1 means decaying
	// below -1 means recycle it
	float health = 1.;

	// properties:
	glm::vec3 location;
	float scale = 1.;
	glm::quat orientation;
	glm::vec3 color;
	float phase = 0.;
	glm::vec4 params;
	
	// non-render:
	glm::vec3 velocity;
	glm::quat rot_vel;
	glm::vec3 accel;

	// species-specific:
	union {
		struct {
			float food;
			float nestness;
			int64_t nest_idx;
		} ant;
		struct {
			glm::vec2 influence = glm::vec2(0);
			glm::vec2 copy = glm::vec2(0);
			glm::vec2 avoid = glm::vec2(0);
			glm::vec2 center = glm::vec2(0);
			glm::vec3 song = glm::vec3(0);
			float speed;
			int64_t eating;
		} boid;
		struct {
			float rate;
			float itchy;
			float healthshow;
			float eating;
		} bug;
		struct {
			float full_size;
			glm::vec2 vel;
			int64_t carried = 0;
		} pred_head;
		struct {
			int32_t root;
		} pred_body;
	};

	// provide a default constructor to suppress compile error due to union member
	Creature() {}
};

struct Segment {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	float phase;
	glm::vec3 velocity;
	glm::vec3 color;
};

struct CreaturePart {
	glm::vec3 location;
	float scale;
	glm::quat orientation;
	glm::vec3 color;
	float phase;
	glm::vec4 params;
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

struct AudioState {
	struct Frame {
		float state;
		float health;
		glm::vec2 norm2;
		glm::vec4 params;
	};

	Frame frames[NUM_AUDIO_FRAMES];
};

struct State {

	float dummy = 10;
	float test = 34;
	
	// for simulation:
	Lifo<NUM_CREATURES> creature_pool;
	CellSpace<LAND_DIM> dead_space;
	Creature creatures[NUM_CREATURES];

	Segment segments[NUM_SEGMENTS];

	// for rendering:
	Particle particles[NUM_PARTICLES];
	CreaturePart creatureparts[NUM_CREATURE_PARTS];
	DebugDot debugdots[NUM_DEBUGDOTS];

	Hashspace2D<NUM_CREATURES, 6> hashspace;

	// the emission field is currently being used to store emissive light (as a form of smell)
	Field3DPod<FIELD_DIM, glm::vec3> emission_field;

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
	Field2DPod<FUNGUS_DIM> fungus_field;
	// RGB corresponds to blood, food, nest
	Field2DPod<FUNGUS_DIM, glm::vec3> chemical_field;
	// the fungus + chemicals combined into a temporally-smoothed texture
	glm::vec4 field_texture[FUNGUS_TEXELS];
	
	// a baked grid of randomness over the landscape:
	glm::vec4 noise_texture[FUNGUS_TEXELS];

	// the fluid simulation:
	Field3DPod<FIELD_DIM, glm::vec3> fluid_velocities;
	Field3DPod<FIELD_DIM, float> fluid_gradient;

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

	// parameters:

	int fluid_passes = 14;
	int fluid_noise_count = 32;
	float fluid_decay = 0.9999f;
	double fluid_viscosity = 0.00000001; //0.00001;
	double fluid_boundary_damping = .2;
	double fluid_noise = 8.;

	float emission_decay = 0.98f;
	glm::vec3 emission_diffuse = glm::vec3(0.01); // somwhere between 0.1 and 0.01 seems to be good
	float emission_scale = 0.5;

	glm::vec3 chemical_decay = glm::vec3(0.999f);
	glm::vec3 chemical_diffuse = glm::vec3(0.001);

	glm::vec3 blood_color = glm::vec3(1., 0.461, 0.272) * 4.f; 
	glm::vec3 food_color = glm::vec3(1., 0.43, 0.64); 
	glm::vec3 nest_color = glm::vec3(0.75, 1., 0.75); 


	float particleSize = 0.005;
	float creature_fluid_push = 0.25f;

	float alive_lifespan_decay = 0.125;
	float dead_lifespan_decay = 0.25;

	// main thread:
	void animate(float dt);
	void reset();
	
	// background threads:
	void fluid_update(float dt);
	void fields_update(float dt);
	void sim_update(float dt);

	void creature_reset(int i) {
		Creature& a = creatures[i];
		a.idx = i;
		a.type = rnd::integer(4) + 1;
		a.state = Creature::STATE_ALIVE;
		a.health = rnd::uni();

		a.location = glm::linearRand(world_min,world_max);
		a.scale = rnd::uni(0.5f) + 0.75f;
		a.orientation = quat_random();
		a.color = glm::ballRand(1.f)*0.5f+0.5f;
		a.phase = rnd::uni();
		a.params = glm::linearRand(glm::vec4(0), glm::vec4(1));

		a.velocity = glm::vec3(0);
		a.rot_vel = glm::quat();
		a.accel = glm::vec3(0);

		switch(a.type) {
			case Creature::TYPE_ANT:
				break;
			case Creature::TYPE_BUG:
				break;
			case Creature::TYPE_BOID:
				break;
			case Creature::TYPE_PREDATOR_HEAD:
				break;
		}
	}

	void creatures_update(float dt) {

		int birthcount = 0;
		int deathcount = 0;
		int recyclecount = 0;

		
		// spawn new?
		//console.log("creature pool count %d", creature_pool.count);
		if (rnd::integer(NUM_CREATURES) < creature_pool.count/4) {
			auto i = creature_pool.pop();
			birthcount++;
			//console.log("spawn %d", i);
			creature_reset(i);
		}

		// visit each creature:
		for (int i=0; i<NUM_CREATURES; i++) {
			Creature& a = creatures[i];
			if (a.state == Creature::STATE_ALIVE) {
				if (a.health < 0) {
					//console.log("death of %d", i);
					deathcount++;
					// remove from hashspace:
					hashspace.remove(i);
					// set new state:
					a.state = Creature::STATE_DECAYING;
					
					continue;
				}

				// simulate as alive
				//... 

				// birth chance?
				if (rnd::integer(NUM_CREATURES) < creature_pool.count) {
					auto j = creature_pool.pop();
					birthcount++;
					//console.log("spawn %d", i);
					creature_reset(j);
					Creature& child = creatures[j];
					child.location = a.location;
					child.orientation = glm::slerp(child.orientation, a.orientation, 0.5f);
				}


				// TODO: make this species-dependent?
				a.health -= dt * alive_lifespan_decay;// * (1.+rnd::bi()*0.1);

			} else if (a.state == Creature::STATE_DECAYING) {
				
				glm::vec3 norm = transform(world2field, a.location);
				glm::vec2 norm2 = glm::vec2(norm.x, norm.z);
				
				// decay complete?
				if (a.health < -1) {
					//console.log("recycle of %d", i);
					recyclecount++;
					a.state = Creature::STATE_BARDO;
					dead_space.unset(norm2);
					creature_pool.push(i);
					continue;
				}

				// simulate as dead:
				
				// rate of decay:
				float decay = dt * dead_lifespan_decay;// * (1.+rnd::bi()*0.1);
				a.health -= decay;

				// blend to dull grey:
				float grey = (a.color.x + a.color.y + a.color.z)*0.25f;
				a.color += dt * (glm::vec3(grey) - a.color);

				// deposit blood:
				al_field2d_addnorm_interp(fungus_dim, chemical_field.front(), norm2, decay * blood_color);

				// retain corpse in deadspace:
				// (TODO: Is this needed, or could a hashspace query do what we need?)
				dead_space.set_safe(i, norm2);
			}
		}
		//console.log("%d deaths, %d recycles, %d births", deathcount, recyclecount, birthcount);
	}
};

#endif