#ifndef STATE_H
#define STATE_H
// michael: testing the client editor
#ifndef ALICE_H
// for the use of Clang-Index:
#include <stddef.h>
#include <stdint.h>
namespace glm {

	struct vec2 { float x, y; vec2(float, float); vec2(float); vec2(); };
	struct vec3 { float x, y, z; vec3(float, float, float); vec3(float); vec3(); };
	struct vec4 { float x, y, z, w; vec4(float, float, float, float); vec4(); };
	struct quat { float x, y, z, w; quat(float, float, float, float); quat(); };

	struct ivec2 { int x, y; ivec2(int, int); };
	struct ivec3 { int x, y, z; ivec3(int, int, int); };
	struct ivec4 { int x, y, z, w; ivec4(int, int, int, int); };

	struct mat3 { float m[9]; };
	struct mat4 { float m[16]; };

}

template<int MAX_OBJECTS = 1024, int RESOLUTION = 5>
struct Hashspace3D {
		struct Object {
			int32_t id;		///< which object ID this is
			int32_t next, prev;
			uint32_t hash;	///< which voxel ID it belongs to (or invalidHash())
			glm::vec3 pos;
		};
		
		struct Voxel {
			/// the linked list of objects in this voxel
			int32_t first;
		};
		
		struct Shell {
			uint32_t start;
			uint32_t end;
		};
	Object mObjects[MAX_OBJECTS];
	Voxel mVoxels[(1<<RESOLUTION) * (1<<RESOLUTION) * (1<<RESOLUTION)];
	Shell mShells[(1<<RESOLUTION) * (1<<RESOLUTION)]; // indices into mVoxelsByDistance
	uint32_t mVoxelsByDistance[(1<<RESOLUTION) * (1<<RESOLUTION) * (1<<RESOLUTION)];
	
	uint32_t mShift, mShift2, mDim, mDim2, mDim3, mDimHalf, mWrap, mWrap3;
	
	glm::mat4 world2voxels;
	glm::mat4 voxels2world;
	float world2voxels_scale;
};


// RESOLUTION 5 means 2^5 = 32 voxels in each axis.
template<int MAX_OBJECTS = 1024, int RESOLUTION = 5>
struct Hashspace2D {
	
	struct Object {
		int32_t id;		///< which object ID this is
		int32_t next, prev;
		uint32_t hash;	///< which voxel ID it belongs to (or invalidHash())
		glm::vec2 pos;
	};
	
	struct Voxel {
		/// the linked list of objects in this voxel
		int32_t first;
	};
	
	struct Shell {
		uint32_t start;
		uint32_t end;
	};
	
	Object mObjects[MAX_OBJECTS];
	Voxel mVoxels[(1<<RESOLUTION) * (1<<RESOLUTION) * (1<<RESOLUTION)];
	Shell mShells[(1<<RESOLUTION) * (1<<RESOLUTION)]; // indices into mVoxelsByDistance
	uint32_t mVoxelsByDistance[(1<<RESOLUTION) * (1<<RESOLUTION) * (1<<RESOLUTION)];
	
	uint32_t mShift, mShift2, mDim, mDim2, mDimHalf, mWrap, mWrap2;
	
	glm::mat3 world2voxels;
	glm::mat3 voxels2world;
	float world2voxels_scale;
	
	Hashspace2D& reset(glm::vec2 world_min, glm::vec2 world_max);

};


template<int N=128, typename T=int32_t>
struct Lifo {
	T list[N];
	int64_t count;
};
	template<int SPACE_DIM, typename T=int32_t, T invalid=T(-1)>
struct CellSpace {
	T cells[SPACE_DIM * SPACE_DIM];
};

template<int DIM=32, typename T=float>
struct Field3DPod { 
	T data0[DIM*DIM*DIM];
	T data1[DIM*DIM*DIM];
	int isSwapped = 0;
};

template<int DIM=32, typename T=float>
struct Field2DPod {
	T data0[DIM*DIM];
	T data1[DIM*DIM];
	int isSwapped = 0;
};
#endif

#define NUM_CREATURES 512
#define NUM_CREATURE_PARTS NUM_CREATURES

#define NUM_PARTICLES 1024*256

#define NUM_TELEPORT_POINTS 4

#define NUM_AUDIO_FRAMES 1024

#define FIELD_DIM 32
#define FIELD_TEXELS (FIELD_DIM*FIELD_DIM)
#define FIELD_VOXELS (FIELD_DIM*FIELD_DIM*FIELD_DIM)



#define LAND_DIM 200
#define LAND_TEXELS (LAND_DIM*LAND_DIM)
#define LAND_VOXELS (LAND_DIM*LAND_DIM*LAND_DIM)

#define SDF_DIM (64)
#define SDF_TEXELS (SDF_DIM*SDF_DIM)
#define SDF_VOXELS (SDF_DIM*SDF_DIM*SDF_DIM)

//#define FLOW_DIM 128
#define FLOW_TEXELS (512*424)

#define FUNGUS_DIM 512
#define FUNGUS_TEXELS (FUNGUS_DIM*FUNGUS_DIM)

// defined to be at least enough to visualize two kinects:
#define NUM_DEBUGDOTS (512*424*2)
//2*5*4

#define NUM_ISLANDS (5)

static const glm::ivec3 field_dim = glm::ivec3(FIELD_DIM, FIELD_DIM, FIELD_DIM);
static const glm::ivec3 land_dim = glm::ivec3(LAND_DIM, LAND_DIM, LAND_DIM);
static const glm::ivec3 sdf_dim = glm::ivec3(SDF_DIM, SDF_DIM, SDF_DIM);

static const glm::ivec2 field_dim2 = glm::ivec2(FIELD_DIM, FIELD_DIM);
static const glm::ivec2 land_dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
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
	glm::quat orientation = glm::quat();
	glm::vec3 color;
	float phase = 0.;
	glm::vec4 params = glm::vec4();
	
	glm::vec3 velocity;
	glm::quat rot_vel = glm::quat();
	//glm::vec3 accel;
	int32_t island;	// which island we are on (0-4)

	// species-specific:
	union {
		struct {
			float food;
			float nestness;
			int64_t nest_idx;
		} ant;
		struct {
			// glm::vec2 influence = glm::vec2(0);
			// glm::vec2 copy = glm::vec2(0);
			// glm::vec2 avoid = glm::vec2(0);
			// glm::vec2 center = glm::vec2(0);
			// glm::vec3 song = glm::vec3(0);
			// float speed;
			// int64_t eating;
		} boid;
		struct {
			float rate;
			float itchy;
			float healthshow;
			float eating;
		} bug;
		struct {
			float full_size;
			int32_t victim;
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

struct CreaturePart {
	glm::vec3 location; float scale;

	glm::quat orientation;

	glm::vec3 color; float phase;

	glm::vec4 params;

	float id;
};

struct Particle {
	glm::vec3 location;
	glm::vec3 color;
	glm::vec3 velocity;
	float phase, unused;
};

struct DebugDot {
	glm::vec3 location;
	float size;
	glm::vec3 color;
	float unused;
};

struct AudioState {
	struct Frame {
		// 0 = Dead, 0.1-0.4 = species type
		float state;	
		// which channel to play, 0.1-0.5
		float speaker; 		
		// < 0 means dead, typically 0..1 when alive
		float health;	
		// age of creature in seconds, 0..
		float age;
		// size of creature in meters (VR world)
		float size;
		// limited to 0..1, but tend to be close to 0.4-0.6
		// the first three params map to RGB in the debug view
		// semantics depend on species, but not all may be used; 
		// work with first channel with highest priority
		glm::vec3 params;	
	};

	Frame frames[NUM_AUDIO_FRAMES];
};

struct State {
	
	// for simulation:
	Lifo<NUM_CREATURES> creature_pool;
	CellSpace<LAND_DIM> dead_space;
	Creature creatures[NUM_CREATURES];

	// for rendering:
	Particle particles[NUM_PARTICLES];
	CreaturePart creatureparts[NUM_CREATURE_PARTS];
	DebugDot debugdots[NUM_DEBUGDOTS];

	Hashspace2D<NUM_CREATURES, 8> hashspace;

	// the emission field is currently being used to store emissive light (as a form of smell)
	Field3DPod<FIELD_DIM, glm::vec3> emission_field;

	// the basic height field
	// .xyz represents the normal
	// .w represents the height
	glm::vec4 land[LAND_TEXELS];

	Field2DPod<LAND_DIM, float> human;

	// the flow field (hopefully this isn't too high res)
	// Paris ran at 128 x 64, for example
	glm::vec2 flow[LAND_TEXELS];

	// signed distance field representing the landscape
	// the distance to the nearest land surface, as a 3D SDF
	// scaled such that the distance across the entire space == 1
	// distances are normalized over the LAND_DIM as 0..1
	float distance[SDF_VOXELS];
	// the boolean field that is used to generate the distance field
	// surface edges are marked by unequal neighbour values
	float distance_binary[SDF_VOXELS];

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

	glm::vec3 teleport_points[NUM_TELEPORT_POINTS];

	glm::vec3 island_centres[NUM_ISLANDS];

	// transforms:
	glm::vec3 world_min = glm::vec3(0.f);
	glm::vec3 world_max = glm::vec3(450.f);
	glm::vec3 world_centre = glm::vec3(225.f);
	float field2world_scale;
	float world2field_scale;
	glm::mat4 world2field;
	glm::mat4 field2world;
	glm::mat4 vive2world;
	glm::mat4 kinect2world; 
	glm::mat4 leap2view;
	glm::mat4 world2minimap;
	float minimapScale = 0.005f;
	float kinect2world_scale = 50.f; //50.f;   // or 30?

	// parameters:

	int fluid_passes = 14;
	int fluid_noise_count = 32;
	float fluid_decay = 0.9999f;
	double fluid_viscosity = 0.00000001; //0.00001;
	double fluid_boundary_damping = .2;
	double fluid_noise = 8.;

	// how much the optical flow impacts the fluid:
	float flow_scale = 1.f;

	float emission_decay = 0.9f;
	glm::vec3 emission_diffuse = glm::vec3(0.01); // somwhere between 0.1 and 0.01 seems to be good
	float emission_scale = 0.9;

	glm::vec3 chemical_decay = glm::vec3(0.98f);
	glm::vec3 chemical_diffuse = glm::vec3(0.0001);

	glm::vec3 blood_color = glm::vec3(1., 0.461, 0.272) * 4.f; 
	glm::vec3 food_color = glm::vec3(1., 0.43, 0.64); 
	glm::vec3 nest_color = glm::vec3(0.75, 1., 0.75); 

	float projector1_location_x = 4.35;
	float projector1_location_y = 6.95;
	float projector1_rotation = 0.f;
	float projector2_location_x = 2.2;
	float projector2_location_y = 3.1;
	float projector2_rotation = M_PI * 0.508;

	float land_fall_rate = 20.f;
	float land_rise_rate = 1.f;

	float vrFade = 0.f;
	float creature_speed = 2.f; // in object-size per second
	float reproduction_health_min = 0.5;
	// per-second:
	float creature_song_copy_factor = 1.25f;
	float creature_song_mutate_rate = 0.25f;

	float particleSize = 0.1;
	float particle_noise = 0.01f;

	float creature_fluid_push = 0.25f;

	float fungus_recovery_rate = 0.02;
	float fungus_seeding_chance = 0.00001;
	float fungus_migration_chance = 0.1f;
	float fungus_decay_chance = 0.1;

	float ant_nestsize = 0.04f;
	float ant_phero_decay = 0.999f;
	float ant_sensor_size = 0.5;
	float ant_food_min = 0.02;
	float ant_sniff_min = 0.001;
	float ant_follow = 0.05;

	float predator_eat_range = 0.125f;
	float predator_view_range = 8.f;

	float alive_lifespan_decay = 0.003;
	float dead_lifespan_decay = 0.25;

	float human_height_decay = 0.99;
	float coastline_height = 10.f;

	// main thread:
	void animate(float dt);
	void reset();
	
	// background threads:
	void fluid_update(float dt);
	void fields_update(float dt);
	void sim_update(float dt);
	void land_update(float dt);

	void generate_land_sdf_and_normals();


	void creature_reset(int i);
	void creatures_health_update(float dt);
	void creature_alive_update(Creature& o, float dt);

	// thread-safe:
	int nearest_island(glm::vec3 pos);
	glm::vec3 random_location_above_land(float h=0.1f);

	void update_projector_loc();

	float ant_sniff_turn(Creature& a, float p1, float p2);
};

#endif