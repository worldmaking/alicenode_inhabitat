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


// re-orient a vector `v` to the nearest direction 
// that it is orthogonal to a given `normal`
// without changing the vector's length
glm::vec3 make_orthogonal_to(glm::vec3 const v, glm::vec3 const normal) {
	// get component of v along normal:
	glm::vec3 normal_component = normal * (dot(v, normal));
	// remove this component from v:
	glm::vec3 without_normal_component = v - normal_component;
	// and re-scale to original magnitude:
	return safe_normalize(without_normal_component) * glm::length(v);
}

// re-orient a quaternion `q` to the nearest orientation 
// whose "up" vector (+Y) aligns to a given `normal`
// assumes `normal` is already normalized (length == 1)
glm::quat align_up_to(glm::quat const q, glm::vec3 const normal) {
	const float eps = 0.00001f;
	const float similar = 1.f-eps;
	// get q's up vector:
	glm::vec3 uy = quat_uy(q);
	// get similarity with normal:
	float dp = glm::dot(uy, normal);
	// if `direction` is already similar to `dir`, leave as-is
	if (fabsf(dp) > similar) return q; 
	// find an orthogonal axis to rotate around:
	glm::vec3 axis = glm::cross(uy, normal);
	// get the rotation needed around this axis:
	glm::quat diff = glm::angleAxis(acosf(dp), axis);
	// rotate the original quat to align to the normal:
	return safe_normalize(diff * q);
}

glm::quat get_forward_rotation_to(glm::quat const q, glm::vec3 const direction) {
	const float eps = 0.00001f;
	const float similar = 1.f-eps;
	float d = glm::length(direction);
	// if `direction` is too small, any direction is as good as any other... no change needed
	if (fabsf(d) < eps) return q; 
	// get similarity with direction:
	glm::vec3 desired = safe_normalize(direction);
	glm::vec3 uf = quat_uf(q);
	float dp = glm::dot(uf, desired); 
	// if `direction` is already similar to `dir`, leave as-is
	if (fabsf(dp) > similar) return q; 
	// get an orthogonal axis to rotate around:
	glm::vec3 axis = glm::cross(uf, desired);
	// get the rotation needed around this axis:
	glm::quat diff = glm::angleAxis(acosf(dp), axis);
	// rotate the original quat to align to the normal:
	return safe_normalize(diff);	
}

// re-orient a quaternion `q` to the nearest orientation 
// whose "forward" vector (-Z) aligns to a given `direction`
glm::quat align_forward_to(glm::quat const q, glm::vec3 const direction) {
	const float eps = 0.00001f;
	const float similar = 1.f-eps;
	float d = glm::length(direction);
	// if `direction` is too small, any direction is as good as any other... no change needed
	if (fabsf(d) < eps) return q; 
	// get similarity with direction:
	glm::vec3 desired = safe_normalize(direction);
	glm::vec3 uf = quat_uf(q);
	float dp = glm::dot(uf, desired); 
	// if `direction` is already similar to `dir`, leave as-is
	if (fabsf(dp) > similar) return q; 
	// get an orthogonal axis to rotate around:
	glm::vec3 axis = glm::cross(uf, desired);
	// get the rotation needed around this axis:
	glm::quat diff = glm::angleAxis(acosf(dp), axis);
	// rotate the original quat to align to the normal:
	return get_forward_rotation_to(q, direction) * q;
}

<<<<<<< HEAD
=======

static int flip = 0;
int rendercreaturecount = 0;
int livingcreaturecount = 0;
int numants = 0;
int numboids = 0;

>>>>>>> lightwork
#define NUM_ISLANDS (5)

#define NUM_CREATURES 1024
#define NUM_CREATURE_PARTS NUM_CREATURES

#define NUM_PARTICLES 1024*256

#define NUM_TELEPORT_POINTS (NUM_ISLANDS)

#define NUM_AUDIO_FRAMES 1024

#define FIELD_DIM 32
#define FIELD_TEXELS (FIELD_DIM*FIELD_DIM)
#define FIELD_VOXELS (FIELD_DIM*FIELD_DIM*FIELD_DIM)


#define LAND_DIM 256
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
	float fullsize = 1.;
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
	glm::vec2 flowsmooth[LAND_TEXELS];

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
	float fluid_decay = 0.9999; //0.999999f;
	double fluid_viscosity = 0.00001;
	double fluid_boundary_damping = .2;
	double fluid_noise = 8.;
	float fluid_advection = 0.25;
	float fluid_contour_follow =  0.001f;

	float creature_fluid_push = 1.f;

	// running averaging of the optical flow:
	float flow_smoothing = 1.f;
	// how much the optical flow impacts the fluid:
	float flow_scale = 0.2f;
	// minimum speed in optical flow image to influence fluid
	float fluid_flow_min_threshold = 1.f;
	

	float emission_decay = 0.9f;
	glm::vec3 emission_diffuse = glm::vec3(0.01); // somwhere between 0.1 and 0.01 seems to be good
	float emission_scale = 0.9;

	glm::vec3 chemical_decay = glm::vec3(0.98f);
	glm::vec3 chemical_diffuse = glm::vec3(0.0001);

	glm::vec3 blood_color = glm::vec3(1., 0.461, 0.272); 
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
	float creature_speed = 3.f; // in object-size per second
	float creature_flatness_min = 0.75; // slope at which creature won't go
	float reproduction_health_min = 0.25;
	// per-second:
	float creature_song_copy_factor = 1.25f;
	float creature_song_mutate_rate = 0.01f;
	float creature_grow_rate = 0.5f; 

	float alive_lifespan_decay = 0.05;
	float ant_alive_lifespan_decay = 0.005;
	float dead_lifespan_decay = 0.1;

	float particleSize = 0.01;
	float particle_noise = 0.0001f;
	float particle_to_egg_distance = 0.05f; // meters
	float creature_to_particle_chance = 0.001f; // chance per particle per second

	float fungus_recovery_rate = 0.002;
	float fungus_seeding_chance = 0.00001;
	float fungus_migration_chance = 0.2f;// 0.1f;
	float fungus_decay_chance = 0.002;
	float fungus_to_boid_transfer = 8.f;

	float ant_speed = 0.5;
	float ant_nestsize = 0.04f;
	float ant_phero_decay = 0.999f;
	float ant_sensor_size = 1.;
	float ant_food_min = 0.02;
	float ant_sniff_min = 0.001;
	float ant_follow = 0.05;

	float predator_eat_range = 0.125f;
	float predator_view_range = 8.f;

	float human_height_decay = 0.99;
	float coastline_height = 10.f;

	// main thread:
	inline void animate(float dt) {
		// keep the computation in here to absolute minimum
		// since it detracts from frame rate
		// here we should only be extrapolating raw visible features
		// such as location, orientation
		// that would otherwise look jerky when animated from the simulation thread

		// reset how many creature parts we will render:
		rendercreaturecount = 0;
		livingcreaturecount = 0;
		numants = 0;
		numboids = 0;

		for (int i=0; i<NUM_PARTICLES; i++) {
			Particle &o = particles[i];
			o.location = o.location + o.velocity * dt;
			//o.location = glm::clamp(o.location, world_min, world_max);
		}

		for (int i=0; i<NUM_CREATURES; i++) {
			auto &o = creatures[i];
			
			if (o.state == Creature::STATE_ALIVE) {
				livingcreaturecount++;

				switch(o.type) {
					case Creature::TYPE_ANT: numants++; break;
					case Creature::TYPE_BOID: numboids++; break;
				}

				// update location
				glm::vec3 p1 = wrap(o.location + o.velocity * dt, world_min, world_max);

				// get norm'd coordinate:
				glm::vec3 norm = transform(world2field, p1);
				glm::vec2 norm2 = glm::vec2(norm.x, norm.z);

				// stick to land surface:
				auto landpt = al_field2d_readnorm_interp(glm::vec2(land_dim), land, norm2);
				p1 = transform(field2world, glm::vec3(norm.x, landpt.w, norm.z));

				float distance = glm::length(p1 - o.location);
				o.location = p1;
				
				// apply change of orientation here too
				// slerping by dt is a close approximation to rotation in radians per second
				o.orientation = safe_normalize(glm::slerp(o.orientation, o.rot_vel * o.orientation, dt * 1.f));

				// re-align the creature to the surface normal:
				glm::vec3 land_normal = glm::vec3(landpt);
				//glm::vec3 land_normal = sdf_field_normal4(sdf_dim, state->distance, norm, 0.025f);
				{
					// re-orient relative to ground:
					float creature_land_orient_factor = 1.f;//0.25f;
					o.orientation = safe_normalize(glm::slerp(o.orientation, align_up_to(o.orientation, land_normal), creature_land_orient_factor));
				}

				o.phase += distance;
			}
			if (o.state != Creature::STATE_BARDO) {
				// copy data into the creatureparts:
				CreaturePart& part = creatureparts[rendercreaturecount];
				part.id = o.type;//i;
				part.location = o.location;
				part.scale = o.scale;
				part.orientation = o.orientation;
				part.color = o.color;
				part.phase = o.phase;
				part.params = o.params;
				rendercreaturecount++;
			}
		}
	}

	void reset();
	
	// background threads:
	void fluid_update(float dt) {

		// disturb the fluid:
		// for (int y=0, i=0; y<LAND_DIM; y++) {
		// 	for (int x=0; x<LAND_DIM; x++, i++) {
		// 		glm::vec3 norm = glm::vec3(
		// 			x/float(LAND_DIM), 
		// 			human.front()[i], 
		// 			y/float(LAND_DIM)
		// 		);

		// 		glm::vec3 push = glm::vec3(
		// 			state->flow[i].x, 
		// 			0.f, 
		// 			state->flow[i].y); 
		// 		push = push * (flow_scale * dt);

		// 		al_field3d_addnorm_interp(field_dim, fluid_velocities.front(), norm, push);
		// 	}
		// }
		
		const glm::ivec3 dim = fluid_velocities.dim();
		const glm::vec3 field_dimf = glm::vec3(field_dim);
		// diffuse the velocities (viscosity)
		fluid_velocities.swap();
		al_field3d_diffuse(dim, fluid_velocities.back(), fluid_velocities.front(), glm::vec3(fluid_viscosity), fluid_passes);

		
		// apply boundary effect to the velocity field
		// boundary effect is the landscape, forcing the fluid to align to it when near
		{
			float minspeed = 100000.f;
			float avgspeed = 0.f;
			float maxspeed = 0.f;
			
			int i = 0;
			glm::vec3 * velocities = fluid_velocities.front();
			for (size_t z = 0; z<field_dim.z; z++) {
				for (size_t y = 0; y<field_dim.y; y++) {
					for (size_t x = 0; x<field_dim.x; x++, i++) {
						
						// get norm'd coordinate:
						glm::vec3 norm = glm::vec3(x,y,z) / field_dimf;
						glm::vec2 norm2 = glm::vec2(norm.x, norm.z);

						// sample flow field:
						auto flo = al_field2d_readnorm_interp(land_dim2, flowsmooth, norm2);

						// limit magnitude?
						float flospd = glm::length(flo);
						minspeed = glm::min(flospd, minspeed);
						maxspeed = glm::max(flospd, maxspeed);
						avgspeed += flospd;

						flo = flospd > fluid_flow_min_threshold ? flo : glm::vec2(0.f);

						// use this to sample the landscape:
						float sdist;
						al_field3d_readnorm_interp(sdf_dim, distance, norm, &sdist);
						float dist = fabsf(sdist);

						// TODO: what happens 'underground'?
						// should velocities here be zeroed? or set to a slight upward motion?	

						// generate a normalized influence factor -- the closer we are to the surface, the greater this is
						//float influence = glm::smoothstep(0.05f, 0.f, dist);
						// s is the amount of dist where the influence is 50%
						float s = fluid_contour_follow;
						float influence = s / (s + dist);


						glm::vec3& vel = velocities[i];

						vel.x += flo.x * flow_scale;
						vel.z += flo.y * flow_scale;
						
						// get a normal for the land:
						// TODO: or read from state->land xyz?
						glm::vec3 normal = sdf_field_normal4(sdf_dim, distance, norm, 1.f/SDF_DIM);
						// re-orient to be orthogonal to the land normal:
						glm::vec3 rescaled = make_orthogonal_to(vel, normal);
						// update:
						vel = mix(vel, rescaled, influence);	

						// also provide an outer boundary:
						glm::vec3 central = 0.5f-norm;
						central.y = 0.f;
						auto factor = glm::dot(central, central);
						central = glm::length(vel) * safe_normalize(central);
						// update:
						//vel = mix(vel, central, factor);	
					}
				}
			}
			//console.log("flow min speed %f max speed %f avg speed %f", minspeed, maxspeed, avgspeed / FIELD_VOXELS);
		}


		// stabilize:
		// prepare new gradient data:
		al_field3d_zero(dim, fluid_gradient.back());
		al_field3d_derive_gradient(dim, fluid_velocities.back(), fluid_gradient.back()); 
		// diffuse it:
		al_field3d_diffuse(dim, fluid_gradient.back(), fluid_gradient.front(), 0.5f, fluid_passes / 2);
		// subtract from current velocities:
		al_field3d_subtract_gradient(dim, fluid_gradient.front(), fluid_velocities.front());
		
		// advect:
		fluid_velocities.swap(); 
		al_field3d_advect(dim, fluid_velocities.front(), fluid_velocities.back(), fluid_velocities.front(), fluid_advection);

		// friction:
		al_field3d_scale(dim, fluid_velocities.front(), glm::vec3(fluid_decay));
		
	}
	void fields_update(float dt) {
		const glm::ivec2 dim = glm::ivec2(FUNGUS_DIM, FUNGUS_DIM);
		const glm::vec2 invdim = 1.f/glm::vec2(dim);

		if (1) {
			// this block is quite expensive, apparently
			float * const src_array = fungus_field.front(); 
			float * dst_array = fungus_field.back(); 


			for (int i=0, y=0; y<dim.y; y++) {
				for (int x=0; x<dim.x; x++, i++) {
					const glm::vec2 cell = glm::vec2(float(x),float(y));
					const glm::vec2 norm = cell*invdim;
					float C = src_array[i];
					float C1 = C - 0.1;
					//float h = 20 * .1;//heightmap_array.sample(norm);
					glm::vec4 l;
					al_field2d_readnorm_interp(glm::ivec2(LAND_DIM, LAND_DIM), land, norm, &l);
					float hm = l.w * field2world_scale - coastline_height;
					float h = l.w;
					float hu = al_field2d_readnorm_interp(land_dim2, human.front(), norm);
					float hum = hu * field2world_scale - coastline_height;
					float hlm = hum - hm;
					float dst = C;
					if (hm <= 0 || hlm > 2.) {
						// force lowlands to be vacant
						// (note, human will also do this)
						dst = 0;
						
					} else if (C < 0.) {
						// very negative values gradually drift back to zero
						// maybe this "fertility" should also depend on height!!
						dst = C + dt*fungus_recovery_rate;
					} else if (hm < 0.) {
						dst = 0.;
					} else if (rnd::uni() < hm * fungus_seeding_chance * dt) {
						// seeding chance
						dst = rnd::uni();
					} else if (rnd::uni() < fungus_decay_chance * dt / hm) {
						// if land lower than vitality, decrease vitality
						// also random chance of decay for any living cell
						//dst = C * rnd::uni(); //- dt*fungus_recovery_rate;
						dst = glm::max(C, rnd::uni());
					} else if (rnd::uni() < hm * fungus_migration_chance * dt) {
						// migration chance increases with altitude
						// pick a neighbour cell:
						glm::vec2 tc = cell + glm::vec2(floor(rnd::uni()*3)-1, floor(rnd::uni()*3)-1);
						float tv = al_field2d_read(dim, src_array, tc); //src_array.samplepix(tc);
						// if alive, copy it
						if (tv > 0.) { dst = tv; }
					}
					dst_array[i] = glm::clamp(dst, -1.f, 1.f);
				}
			}
			fungus_field.swap();
		}

		if (1) {
			// diffuse & decay the chemical fields:
			// copy front to back, then diffuse
			// TODO: is this any different to just diffuse (front, front) ? if not, we could eliminate this copy
			// (or, would .swap() rather than .copy() work for us?)
			chemical_field.swap();	
			al_field2d_diffuse(fungus_dim, chemical_field.back(), chemical_field.front(), chemical_diffuse, 2);
			size_t elems = chemical_field.length();
			for (size_t i=0; i<elems; i++) {
				// the current simulated field values:
				glm::vec3& chem = chemical_field.front()[i];
				float f = fungus_field.front()[i];
				// the current smoothed fields as used by the renderer:
				glm::vec4& tex = field_texture[i];
				// the current smoothed fungus value:
				float f0 = tex.w; 

				// fungus increases smoothly, but death is immediate:
				float f1 = (f <= 0) ? f : glm::mix(f0, f, 0.1f);

				// other fields just clamp & decay:
				chem = glm::clamp(chem * chemical_decay, 0.f, 1.f);

				// now copy these modified results back to the field_texture:
				tex = glm::vec4(chem, f1);
			}
		}
			
		if (1) {
			// diffuse and decay the emission field:
			al_field3d_scale(field_dim, emission_field.back(), glm::vec3(emission_decay));
			al_field3d_diffuse(field_dim, emission_field.back(), emission_field.front(), emission_diffuse);
			emission_field.swap();
		}
	}
	
	void sim_update(float dt, AudioState * audiostate) {

		// inverse dt gives rate (per second)
		float idt = 1.f/dt;

		Alice& alice = Alice::Instance();


		// get the most recent complete frame:
		flip = !flip;
		const CloudDevice& cd = alice.cloudDeviceManager.devices[0];
		const CloudFrame& cloudFrame = cd.cloudFrame();
		const glm::vec3 * cloud_points = cloudFrame.xyz;
		const glm::vec2 * uv_points = cloudFrame.uv;
		const glm::vec3 * rgb_points = cloudFrame.rgb;
		uint64_t max_cloud_points = sizeof(cloudFrame.xyz)/sizeof(glm::vec3);
		glm::vec2 kaspectnorm = 1.f/glm::vec2(float(cColorHeight)/float(cColorWidth), 1.f);
		glm::vec3 kinectloc = world_centre + glm::vec3(0,0,-4);

		// deal with Kinect data:
		CloudDevice& kinect0 = alice.cloudDeviceManager.devices[0];
		CloudDevice& kinect1 = alice.cloudDeviceManager.devices[1];

		// map depth data onto land:
		if (1) {
			// first, dampen the human field:
			for (int i=0; i<LAND_TEXELS; i++) {
				human.front()[i] *= human_height_decay;
			}
			
			// for each Kinect
			for (int k=0; k<2; k++) {
				//if (k == 11) continue;

				
				const CloudFrame& cloudFrame0 = k ? kinect1.cloudFrame() : kinect0.cloudFrame();
				const CloudFrame& cloudFrame1 = k ? kinect1.cloudFramePrev() : kinect0.cloudFramePrev();
				const glm::vec3 * cloud_points0 = cloudFrame0.xyz;
				const glm::vec2 * uv_points0 = cloudFrame0.uv;
				const glm::vec3 * rgb_points0 = cloudFrame0.rgb;
				const uint16_t * depth0 = cloudFrame0.depth;

				// now update with live data:
				for (int i=0, y=0; y < cDepthHeight; y++) {
					for (int x=0; x < cDepthWidth; x++, i++) {

						// masks:
						if (k==1 && (x > cDepthWidth * 0.7 && y > cDepthWidth * 0.5)) continue;
						if (k==1 && (x > cDepthWidth * 0.85)) continue;
						if (k==0 && (y < cDepthHeight * 0.3 && x > cDepthWidth * 0.75)) continue;

						auto uv = (uv_points0[i] - glm::vec2(0.5f, 0.5f)) * kaspectnorm;
						if (k==0 && glm::length((uv_points0[i] - glm::vec2(0.17f, 0.93f))* kaspectnorm) < 0.2f) continue;

						if (k==0 && glm::length((uv_points0[i] - glm::vec2(0.57f, 0.8f))* kaspectnorm) < 0.15f) continue;	

						if (k==0 && glm::length((uv_points0[i] - glm::vec2(0.9f, 0.93f))* kaspectnorm) < 0.35f) continue;

						auto pt = cloud_points0[i];
						// filter out bad depths
						// mask outside a circular range
						// skip OOB locations:
						uint16_t min_dist = 3000;
						uint16_t max_dist = 6000;
						if (
							depth0[i] <= min_dist 
							|| depth0[i] >= max_dist
							|| pt.x < world_min.x
							|| pt.z < world_min.z
							|| pt.x > world_max.x
							|| pt.z > world_max.z
							|| pt.y > (2.0 * kinect2world_scale)
							) continue;

						// find nearest land cell for this point:
						// get norm'd coordinate:
						glm::vec3 norm = transform(world2field, pt);
						glm::vec2 norm2 = glm::vec2(norm.x, norm.z);
						// get cell index for this location:
						int landidx = al_field2d_index_norm(land_dim2, norm2);
						
						// set the land value accordingly:
						float& humanpt0 = human.front()[landidx];
						float& humanpt1 = human.back()[landidx];

						float h = world2field_scale * pt.y;
						humanpt1 = glm::mix(humanpt0, h, 0.4f);

						// in archi15 we also did spatial filtering

					}
				}
			} // end 2 kinects
			
			//al_field2d_diffuse(land_dim2, human.back(), human.front(), 0.5f, 3);
			human.swap();
			// NOW FLOW
	#ifdef AL_WIN
			if (1) {

				// copy human to char arrays for CV:
				for (int i=0; i<LAND_TEXELS; i++) {
					humanchar0[i] = humanchar1[i];
					humanchar1[i] = human.front()[i] * 255;
				}
				
				int levels = 3; // default=5;
				double pyr_scale = 0.5;
				int winsize = 13;
				int iterations = 3; // default = 10;
				int poly_n = 5;
				double poly_sigma = 1.2; // default = 1.1
				int flags = 0;
				
				// create CV mat wrapper around Jitter matrix data
				// (cv declares dim as numrows, numcols, i.e. dim1, dim0, or, height, width)
				void * src;
				cv::Mat prev(LAND_DIM, LAND_DIM, CV_8UC(1), (void *)humanchar0);
				cv::Mat next(LAND_DIM, LAND_DIM, CV_8UC(1), (void *)humanchar1);
				cv::Mat flow(LAND_DIM, LAND_DIM, CV_32FC(2), (void *)state->flow);
				cv::calcOpticalFlowFarneback(prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);

				for (int i=0; i<LAND_TEXELS; i++) {
					flowsmooth[i] += flow_smoothing * (state->flow[i] - flowsmooth[i]);
				}
				
			}
	#endif

			
		}
		if (!alice.isSimulating) return;

		if (1) {
			{
				const CloudFrame& cloudFrame1 = kinect1.cloudFrame();
				const glm::vec3 * cloud_points1 = cloudFrame1.xyz;
				const glm::vec2 * uv_points1 = cloudFrame1.uv;
				const glm::vec3 * rgb_points1 = cloudFrame1.rgb;
				const uint16_t * depth1 = cloudFrame1.depth;

				const CloudFrame& cloudFrame0 = kinect0.cloudFrame();
				const glm::vec3 * cloud_points0 = cloudFrame0.xyz;
				const glm::vec2 * uv_points0 = cloudFrame0.uv;
				const glm::vec3 * rgb_points0 = cloudFrame0.rgb;
				const uint16_t * depth0 = cloudFrame0.depth;


				int drawn = 0;
				
				for (int i=0, ki=0; i<NUM_DEBUGDOTS; i++) {
					DebugDot& o = debugdots[i];
					
					if (kinect0.capturing && i < max_cloud_points) {
						auto uv = (uv_points0[ki] - 0.5f) * kaspectnorm;
						if (depth0[ki] > 0 && glm::length(uv) < 0.5f) {
							o.location = cloud_points0[ki];
							o.color = glm::vec3(0.8, 0.5, 0.3);
							drawn++;
						}
						ki = (ki+1) % max_cloud_points;
					} else if (kinect1.capturing && i >= max_cloud_points) {
						auto uv = (uv_points1[ki] - 0.5f) * kaspectnorm;
						if (depth1[ki] > 0 && glm::length(uv) < 0.5f) {
							o.location = cloud_points1[ki];
							o.color = glm::vec3(0.5, 0.8, 0.3);
							drawn++;
						}
						ki = (ki+1) % max_cloud_points;
					}
				}
				//console.log("%d %d / %d", cDepthHeight*cDepthWidth, max_cloud_points, drawn);


			}
		}


		if (1) {
			for (int i=0; i<NUM_PARTICLES; i++) {
				Particle &o = particles[i];

				// get norm'd coordinate:
				glm::vec3 norm = transform(world2field, o.location);
				float h = al_field2d_readnorm_interp(land_dim2, land, glm::vec2(norm.x, norm.z)).w * field2world_scale;
				

				//glm::vec3 flow;
				//fluid.velocities.front().readnorm(transform(world2field, o.location), &flow.x);
				glm::vec3 flow = al_field3d_readnorm_interp(field_dim, fluid_velocities.front(), norm);

				// noise:
				flow += glm::sphericalRand(particle_noise);

				o.velocity = flow * idt;

				// chance of becoming egg?
				float hdist = fabsf(o.location.y - h);
				if (hdist < particle_to_egg_distance) {
					// spawn new?
					//console.log("creature pool count %d", creature_pool.count);
					if (creature_pool.count) {
						auto i = creature_pool.pop();
						//birthcount++;
						//console.log("spawn %d", i);
						creature_reset(i);
						Creature& a = creatures[i];
						//if (a.type != Creature::TYPE_ANT) a.location = o.location;
						//a.island = nearest_island(a.location);
					}

				} 
				
				if (rnd::uni() < (creature_to_particle_chance * dt)) {
					int idx = i % NUM_CREATURES;
					o.location = creatures[idx].location;
				} else if (
					o.location.y < h 
					|| o.location.y > world_centre.y
					|| o.location.x < world_min.x
					|| o.location.x > world_max.x
					|| o.location.z < world_min.z
					|| o.location.z > world_max.z
					) {
					
					o.location = random_location_above_land(coastline_height);
					int idx = i % NUM_CREATURES;
					//o.location = creatures[idx].location;
				} else {
					o.location.x = wrap(o.location.x, world_min.x, world_max.x);
					o.location.z = wrap(o.location.z, world_min.z, world_max.z);
				}



				
			}
		} 

		// update all creatures
		creatures_health_update(dt);

		// simulate creature pass:
		for (int i=0; i<NUM_CREATURES; i++) {
			auto &o = creatures[i];
			AudioState::Frame& audioframe = audiostate->frames[o.idx % NUM_AUDIO_FRAMES];

			if (o.state == Creature::STATE_ALIVE) {
				creature_alive_update(o, dt);

				audioframe.state = float(o.type) * 0.1f;
				audioframe.state = float(rnd::integer(4) + 1) * 0.1f;
				audioframe.speaker = o.island * 0.1f;
				audioframe.health = o.health;
				audioframe.age = o.phase;
				audioframe.size = o.scale;
				audioframe.params = glm::vec3(o.params);
				
			} else {

				audioframe.state = 0;
			}
		}
	}

	void land_update(float dt) {
		for (int y=0; y<LAND_DIM; y++) {
			for (int x=0; x<LAND_DIM; x++) {
				auto land_idx = al_field2d_index_nowrap(land_dim2, x, y);

				float h = human.front()[land_idx];
				glm::vec4& landpt = land[land_idx];
				//landpt.w = h;

				if (h == 0.f) continue;

				float h1;

				// glm::vec4& landpt = land[land_idx];
				if (h <= landpt.w) {
					// fall quickly:
					h1 = glm::mix(landpt.w, h, land_fall_rate * dt);
				} else {
					// fall quickly:
					h1 = glm::mix(landpt.w, h, land_rise_rate * dt);
				}

				landpt.w = h1;//glm::mix(landpt.w, h1, 0.2f);
			}
		}

		// maybe diffuse too to smoothen land?
		
		// next, calculate normals
		// THIS IS TOO SLOW!!!!
		//generate_land_sdf_and_normals();
	}

	void generate_land_sdf_and_normals() {
		{
			// generate SDF from land height:
			int i=0;
			for (size_t z=0;z<SDF_DIM;z++) {
				for (size_t y=0;y<SDF_DIM;y++) {
					for (size_t x=0;x<SDF_DIM;x++) {
						glm::vec3 coord = glm::vec3(x, y, z);
						glm::vec3 norm = coord/glm::vec3(SDF_DIM);
						glm::vec2 norm2 = glm::vec2(norm.x, norm.z);
						
						//int ii = al_field2d_index(dim2, glm::ivec2(x, z));
						//float w = land[ ii ].w;
						float w = al_field2d_readnorm_interp(land_dim2, land, norm2).w;

						distance[i] = norm.y < w ? -1. : 1.;
						distance_binary[i] = distance[i] < 0.f ? 0.f : 1.f;

						i++;
					}
				}
			}
			sdf_from_binary(sdf_dim, distance_binary, distance);
			//sdf_from_binary_deadreckoning(land_dim, distance_binary, distance);
			al_field3d_scale(sdf_dim, distance, 1.f/(SDF_DIM));
		}

		// generate land normals:
		int i=0;
		glm::ivec2 dim2 = glm::ivec2(LAND_DIM, LAND_DIM);
		for (size_t y=0;y<dim2.y;y++) {
			for (size_t x=0;x<dim2.x;x++, i++) {
				glm::vec2 coord = glm::vec2(x, y);
				glm::vec2 norm = coord/glm::vec2(dim2);
				glm::vec2 snorm = norm*2.f-1.f;

				float w = land[i].w;

				glm::vec3 norm3 = glm::vec3(norm.x, w, norm.y);

				glm::vec3 normal = sdf_field_normal4(sdf_dim, distance, norm3, 1.f/SDF_DIM);
				land[i] = glm::vec4(normal, w);
			}
		}
	}


	void creature_reset(int i) {
		int island = rnd::integer(NUM_ISLANDS);
		Creature& a = creatures[i];
		a.idx = i;
		//a.type = (rnd::integer(2) + 1) * 2;
		a.type = (rnd::integer(2) + 1);
		//if (rnd::uni() < 0.01) a.type = Creature::TYPE_PREDATOR_HEAD;
		a.state = Creature::STATE_ALIVE;
		a.health = rnd::uni()*0.5f+0.5f;

		a.location = random_location_above_land(coastline_height * 1.2);
		//a.location = island_centres[island];
		a.fullsize = rnd::uni(0.5f) + 0.75f;
		a.scale = 0.f;
		a.orientation = glm::angleAxis(rnd::uni(float(M_PI * 2.)), glm::vec3(0,1,0));
		a.color = glm::linearRand(glm::vec3(0.25), glm::vec3(1));
		a.phase = rnd::uni();
		a.params = glm::linearRand(glm::vec4(0), glm::vec4(1));

		a.velocity = glm::vec3(0);
		a.rot_vel = glm::quat();
		a.island = nearest_island(a.location);

		switch(a.type) {
			case Creature::TYPE_ANT:
				a.ant.nest_idx = island;
				a.location = island_centres[island];
				a.ant.food = 0;
				a.ant.nestness = 1;
				a.fullsize *= 1.25f;
				break;
			case Creature::TYPE_BUG:
				break;
			case Creature::TYPE_BOID:
				a.fullsize *= 0.75f;
				break;
			case Creature::TYPE_PREDATOR_HEAD:
				a.pred_head.victim = -1;
				break;
		}
	}

	void creatures_health_update(float dt) {

		int birthcount = 0;
		int deathcount = 0;
		int recyclecount = 0;

		// spawn a predator?
		// if (rnd::integer(NUM_CREATURES) < creature_pool.count/4
		// 	&& creature_pool.count > 7) {

		// 	auto idx = creature_pool.pop();
		// 	creature_reset(idx);
		// 	Creature& a = creatures[idx];
		// 	a.type = Creature::TYPE_PREDATOR_HEAD;
		// 	for (int i=0; i<6; i++) {
		// 		auto seg = creature_pool.pop();
		// 	}
		// }

		if (0) {
			// spawn new?
			//console.log("creature pool count %d", creature_pool.count);
			if (rnd::integer(NUM_CREATURES) < creature_pool.count) {
				auto i = creature_pool.pop();
				birthcount++;
				//console.log("spawn %d", i);
				creature_reset(i);
				Creature& a = creatures[i];
				//a.location = glm::linearRand(world_min, world_max);
				//a.island = nearest_island(a.location);
			}
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
				hashspace.move(i, glm::vec2(a.location.x, a.location.z));

				// // birth chance?
				// if (rnd::uni() < a.health * reproduction_health_min * dt
				// 	&& rnd::integer(NUM_CREATURES) < creature_pool.count
				// 	&& (a.type == Creature::TYPE_BOID 
				// 	|| a.type == Creature::TYPE_ANT)) {
				// 	auto j = creature_pool.pop();
				// 	birthcount++;
				// 	//console.log("child %d", i);
				// 	creature_reset(j);
				// 	Creature& child = creatures[j];
				// 	child.type = a.type;
				// 	if (child.type != Creature::TYPE_ANT) child.location = a.location;
				// 	child.island = nearest_island(child.location);
				// 	child.orientation = glm::slerp(child.orientation, a.orientation, 0.5f);
				// 	child.params = glm::mix(child.params, a.params, 0.9f);
				// }


				// TODO: make this species-dependent?
				a.health -= dt * alive_lifespan_decay;// * (1.+rnd::bi()*0.1);

				a.scale += creature_grow_rate * dt * (a.fullsize - a.scale);

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

				// retain corpse in deadspace:
				// (TODO: Is this needed, or could a hashspace query do what we need?)
				dead_space.set_safe(i, norm2);

				// simulate as dead:
				
				// rate of decay:
				float decay = dt * dead_lifespan_decay;// * (1.+rnd::bi()*0.1);
				a.health -= decay;
				//a.scale += creature_grow_rate * dt * (0.f-a.scale);

				// blend to dull grey:
				float grey = (a.color.x + a.color.y + a.color.z)*0.25f;
				a.color += dt * (glm::vec3(grey) - a.color);

				// deposit blood:
				al_field2d_addnorm_interp(fungus_dim, chemical_field.front(), norm2, decay * blood_color);
			}
		}
		//console.log("%d deaths, %d recycles, %d births", deathcount, recyclecount, birthcount);
	}
	void creature_alive_update(Creature& o, float dt) {
		float idt = 1.f/dt;
		// float daylight_factor = sin(daytime + 1.5 * a.pos.x) * 0.4 + 0.6; // 0.2 ... 1


		/// FOR ALL CREATURE TYPES ///


		// SENSE LOCATION & ORIENTATION
		// get orientation:
		glm::quat& oq = o.orientation;
		// derive from orientation:
		glm::vec3 up = quat_uy(oq);
		glm::vec3 uf = quat_uf(oq);
		glm::vec3 ux = quat_ux(oq);
		// go slower when moving uphill? 
		// positive value means going uphill, range is -1 to 1
		//float downhill = glm::dot(uf, glm::vec3(0.f,1.f,0.f)); 
		float uphill = uf.y;  
		float downhill = -uphill;

		// get norm'd coordinate:
		glm::vec3 norm = transform(world2field, o.location);
		glm::vec2 norm2 = glm::vec2(norm.x, norm.z);


		// look ahead to where we will be in 1 sim frame
		float lookahead_m = glm::length(o.velocity) * dt; 
		glm::vec3 ahead_pos = o.location + uf * lookahead_m;
		glm::vec3 norm_ahead = transform(world2field, ahead_pos);
		glm::vec2 norm_ahead2 = glm::vec2(norm_ahead.x, norm_ahead.z);

		

		// SENSE LAND
		// get a normal for the land:
		//glm::vec3 land_normal = sdf_field_normal4(sdf_dim, distance, norm, 0.05f/SDF_DIM);

		auto landpt = al_field2d_readnorm_interp(land_dim2, land, norm2);
		auto landpt_ahead = al_field2d_readnorm_interp(land_dim2, land, norm_ahead2);
		glm::vec3 land_normal = safe_normalize(glm::vec3(landpt));
		glm::vec3 land_normal_ahead = safe_normalize(glm::vec3(landpt_ahead));



		// 0..1 factors:
		//float flatness = fabsf(glm::dot(land_normal, glm::vec3(0.f,1.f,0.f)));
		float flatness = fabsf(land_normal.y);
		float flatness_ahead = fabsf(land_normal_ahead.y);
		float steepness = 1.f-flatness;
		// a direction along which the ground is horizontal (contour)
		//glm::vec3 flataxis = safe_normalize(glm::cross(land_normal, glm::vec3(0.f,1.f,0.f)));
		glm::vec3 flataxis = safe_normalize(glm::vec3(-land_normal.z, 0.f, land_normal.x));
		// get my distance from the ground:
		float sdist; // creature's distance above the ground (or negative if below)
		al_field3d_readnorm_interp(sdf_dim, distance, norm, &sdist);

		// SENSE FLUID
		// get fluid flow:
		//glm::vec3 flow;
		//fluid.velocities.front().readnorm(norm, &flow.x);
		glm::vec3 fluid = al_field3d_readnorm_interp(field_dim, fluid_velocities.front(), norm);
		// convert to meters per second:
		// (why is this needed? shouldn't it be m/s already?)
		fluid *= idt;

		// NEIGHBOUR SEARCH:
		// how far in the future want to focus attention?
		// maximum number of agents a spatial query can return
		const int NEIGHBOURS_MAX = 8;
		// see who's around:
		std::vector<int32_t> neighbours;
		float agent_range_of_view = o.scale * 10.f;
		float agent_personal_space = o.scale * 1.f;
		float field_of_view = -0.5f; // in -1..1
		// TODO: figure out why the non-toroidal mode isn't working
		int nres = hashspace.query(neighbours, NEIGHBOURS_MAX, glm::vec2(ahead_pos.x, ahead_pos.z), o.idx, agent_range_of_view, 0.f, true);

		// initialize for navigation
		// zero out by default:
		// TODO: or decay?
		o.rot_vel = glm::quat();

		// set my speed in meters per second:
		float speed = o.scale * creature_speed 
				// modify by params:
				* (0.5f + o.params.x)
				//* (1. + downhill*0.5f)		// tends to make them stay downhill
				// * (1.f + 0.1f*rnd::bi()) 
				// * glm::min(1.f, a.health * 10.f)
				// * daylight_factor*3.f
				;
		
		

		switch (o.type) {
		case Creature::TYPE_ANT: {

			speed *= ant_speed;

			// add some field stuff:
			glm::vec3 chem = food_color * o.ant.food
				+ nest_color * o.ant.nestness;
			// add to land, add to emission:
			al_field2d_addnorm_interp(fungus_dim, chemical_field.front(), norm2, chem * dt);
			al_field3d_addnorm_interp(field_dim, emission_field.back(), norm, chem * dt * emission_scale);

			auto nest = island_centres[o.ant.nest_idx];
			// cheat
			nest.y = o.location.y;
			glm::vec3 nest_relative = nest - o.location;
			float nestdist = glm::length(nest_relative) - ant_nestsize;

			//-- my own food decays too:
			o.ant.food *= ant_phero_decay;
			o.ant.nestness *= ant_phero_decay;


			// get antenna locations:
			glm::vec3 a1 = o.location + ant_sensor_size * o.scale * (uf + ux);
			glm::vec3 a2 = o.location + ant_sensor_size * o.scale * (uf - ux);
			// look for nest if we have food
			// or if we lack health
			if (o.ant.food > ant_food_min || o.health < 0.5) {
				//-- are we there yet?
				if (nestdist < 0.) {
					// TODO sounds.ant_nest(a)	
					// drop food and search again:
					//nestfood = nestfood + a.food
					o.ant.food = 0;
					o.ant.nestness = 1;
					o.health = o.ant.food;
				} else {
					//-- look for nest:
					auto normp1 = transform(world2field, a1);
					auto normp2 = transform(world2field, a2);
					float p1 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp1.x, normp1.z)).z;
					float p2 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp2.x, normp2.z)).z;
					console.log("sniff nest %f %f", p1, p2);
					ant_sniff_turn(o, p1, p2);
				}
			} else {
				// look for food (blood)
				float f = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), norm2).x;
				if (f > ant_food_min) {
					//sounds.ant_food(a)	
					// remove it:
					f = glm::clamp(f, 0.f, 1.f);
					al_field2d_addnorm_interp(fungus_dim, chemical_field.back(), norm2, glm::vec3(-f, 0.f, 0.f));
					o.ant.food += f;
					//a.vel = -a.vel;
					o.health = 1;
				
				} else {
					// look for food:
					auto normp1 = transform(world2field, a1);
					auto normp2 = transform(world2field, a2);
					float p1 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp1.x, normp1.z)).y;
					float p2 = al_field2d_readnorm_interp(fungus_dim, chemical_field.back(), glm::vec2(normp2.x, normp2.z)).y;
					
					//DPRINT("%f %f", p1, p2);
					
					ant_sniff_turn(o, p1, p2);
				}

				o.color = chem;
			}

		} break;
		case Creature::TYPE_BOID: {

			// SENSE FUNGUS
			size_t fungus_idx = al_field2d_index_norm(fungus_dim, norm2);
			//float fungal = fungus_field.front()[fungus_idx];
			float fungal = al_field2d_readnorm_interp(fungus_dim, fungus_field.front(), norm2);
			//if(i == objectSel) console.log("fungal %f", fungal);
			float eat = glm::max(0.f, fungal) * fungus_to_boid_transfer;
			//al_field2d_addnorm_interp(fungus_dim, fungus_field.front(), norm2, -eat);
			fungus_field.front()[fungus_idx] -= eat;

			o.color = glm::vec3(o.params) * 0.75f + 0.25f;

			//o.color = glm::vec3(0, 1, 0);
			
		} break;
		case Creature::TYPE_BUG: {

		} break;
		case Creature::TYPE_PREDATOR_HEAD: {
			speed = speed * 2.f;

		} break;
		case Creature::TYPE_PREDATOR_BODY: {

		} break;
		} // end switch

		glm::vec3 copy;
		int copycount = 0;
		glm::vec3 center;
		int centrecount = 0;
		glm::vec3 avoid;
		int avoidcount = 0;
		//for (auto j : neighbours) {
		for (int j=0; j<nres; j++) {
			auto& n = creatures[neighbours[j]];
			// its future location
			auto nfut = n.location + n.scale * n.velocity*dt;

			// get relative vector from a to n:
			glm::vec3 rel = nfut - ahead_pos; //o.location;
			float cdistance = glm::length(rel);
			// skip if we are right on top of each other -- no point
			if (cdistance < 0.0001f || cdistance > agent_range_of_view) continue;

			//o.color = glm::vec3(0, 0, 1);

			// normalize relative vector:
			glm::vec3 nrel = rel / cdistance;
		
			// get distance between bodies (never be negative)
			float distance = glm::max(cdistance - o.scale - n.scale, 0.f);

			// skip if not in my field of view
			float similarity = glm::dot(uf, nrel);
			if (similarity < field_of_view) continue;

			// copy song (params)
			o.params = glm::mix(o.params, n.params, creature_song_copy_factor*dt);
			
			// if too close, stop moving:
			if (distance > 0.0000001f 
				&& distance < agent_personal_space
				&& (o.type != Creature::TYPE_PREDATOR_HEAD || n.type == Creature::TYPE_PREDATOR_HEAD)) {
				// add an avoidance force:
				//float mag = 1. - (distance / agent_personal_space);
				//float avoid_factor = -0.5;

				// factor is 0 at agent_personal_space
				// factor is massive when distance is near zero
				float avoid_factor = ((agent_personal_space/distance) - 1.f);
				avoid -= rel  * avoid_factor;
				avoidcount++;
				//avoid += rel * (mag * avoid_factor);
				//o.velocity *= glm::vec3(0.5f);
				//o.velocity = limit(o.velocity, distance);
			} else {
				// not too near, so use centering force

				// accumulate centres:
				center += rel;
				centrecount++;

				copy += n.velocity;
				copycount++;
			}
		}

		// adjust velocity according to neighbourhood
		// actually velocity is just a function of speed & uf
		// so need to adjust direction
		// (though, could throttle speed to avoid collision)

		//glm::vec3(0,1,0);
		// turn away from lowlands:
		
		if (o.location.y < coastline_height) {
			// instant death
			o.health = -1.;
		} else if (flatness_ahead < creature_flatness_min) {
			// turn away to the nearest flat axis
			// get a very smooth normal:
			float smooth = 0.5f;
			
			// the normal tells us which way we don't want to go
			auto ln = land_normal_ahead;
			
			auto desired_dir = safe_normalize(glm::vec3(-ln.x, 0.f, -ln.z));
			auto q = get_forward_rotation_to(o.orientation, desired_dir);
			o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.5f);


		} else if (o.location.y < coastline_height*2.) {

			// get a very smooth normal:
			float smooth = 0.5f;
			glm::vec3 ln = sdf_field_normal4(sdf_dim, distance, norm, 0.125f/SDF_DIM);

			auto desired_dir = safe_normalize(glm::vec3(-ln.x, 0.f, -ln.z));
			auto q = get_forward_rotation_to(o.orientation, desired_dir);
			o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.5f);
			//o.color = glm::vec3(1,0,0);

		} else if (avoidcount > 0) {
			auto desired_dir = safe_normalize(avoid);
			//desired_dir = quat_unrotate(o.orientation, desired_dir);

			auto q = get_forward_rotation_to(o.orientation, desired_dir);
			o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.25f);
			
		} else {

			switch (o.type) {
			case Creature::TYPE_ANT: {
				
				// TODO:
				// burrow down near nest:
				//h += 0.2*AL_MIN(0, nestdist);
				{
					float range = M_PI * steepness * M_PI;
					glm::quat wander = glm::angleAxis(glm::linearRand(-range, range), up);
					float wander_factor = 0.5f * dt;
					o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, wander, wander_factor));
				}

			} break;
			case Creature::TYPE_BOID: {

				
				if (copycount > 0) {
					// average neighbour velocity
					copy /= float(copycount);

					// split into speed & direction
					float desired_speed = glm::length(copy);

					auto desired_dir = safe_normalize(copy);

					auto q = get_forward_rotation_to(o.orientation, desired_dir);
					o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.125f);
					
					// // get disparity
					// auto influence_diff = desired_dir - uf;
					// float diff_mag = glm::length(influence_diff);
					// if (diff_mag > 0.001f && desired_speed > 0.001f) {
					// 	// try to match speed:
					// 	speed = glm::min(speed, desired_speed);
					// 	// try to match orientation:

					// }
				}
				if (centrecount > 0) {

					center /= float(centrecount);

					auto desired_dir = safe_normalize(center);

					// if centre is too close, rotate away from it?
					// centre is relative to self, but not rotated
					auto q = get_forward_rotation_to(o.orientation, desired_dir);
					o.orientation = glm::slerp(o.orientation, q * o.orientation, 0.125f);
					
				} else {
					float range = M_PI * steepness * M_PI;
					glm::quat wander = glm::angleAxis(glm::linearRand(-range, range), up);
					float wander_factor = 0.15f * dt;
					o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, wander, wander_factor));
				}
			} break;
			case Creature::TYPE_BUG: {

			} break;
			case Creature::TYPE_PREDATOR_HEAD: {

				// speed varies with hunger
				// can we eat?
				for (int j=0; j<nres; j++) {
					auto& n = creatures[neighbours[j]];
					auto rel = n.location - o.location;
					float dist = glm::length(rel) - o.scale - n.scale;
					if (dist < o.scale * predator_eat_range) {


						// jump:
						o.location += rel * 0.5f;
						float e = n.health;
						o.health = AL_MIN(2.f, o.health + e);
						n.health = -0.5f;

						o.pred_head.victim = -1;
						break;
					}
				}

				if (rnd::uni() * 0.2f < dt) {
					// change target:
					o.pred_head.victim = -1;

					if (nres) {
						auto j = neighbours[0];
						Creature& n = creatures[j];
						if (n.type != Creature::TYPE_PREDATOR_HEAD
							&& n.type != Creature::TYPE_PREDATOR_BODY) {
							o.pred_head.victim = j;
							}
					}
				}

				// follow target:
				if (o.pred_head.victim >= 0) {
					Creature& n = creatures[o.pred_head.victim];
					auto rel = n.location - o.location;

					auto desired_dir = safe_normalize(rel);
					// if centre is too close, rotate away from it?
					// centre is relative to self, but not rotated
					auto q = get_forward_rotation_to(o.orientation, desired_dir);
					//o.orientation = glm::slerp(o.orientation, q * o.orientation, .125f);
					o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, q, 0.25f));
				} else {
					// wander
					float range = M_PI;
					glm::quat wander = glm::angleAxis(glm::linearRand(-range, range), up);
					float wander_factor = dt;
					//o.rot_vel = safe_normalize(glm::slerp(o.rot_vel, wander, wander_factor));

				}


				o.color = glm::mix(glm::vec3(0.7), glm::vec3(1, 0.5, 0), o.health);

			} break;
			case Creature::TYPE_PREDATOR_BODY: {

			} break;
			} // end switch
			
		}

		// velocity
		o.velocity = speed * glm::normalize(uf);

		// let song evolve:
		o.params = wrap(o.params + glm::linearRand(glm::vec4(-creature_song_mutate_rate*dt), glm::vec4(creature_song_mutate_rate*dt)), 1.f);
		//o.color = glm::vec3(o.params);

		// float gravity = 2.0f;
		// o.accel.y -= gravity; //glm::mix(o.accel.y, newrise, 0.04f);
		// if (sdist < (o.scale * 0.025f)) { //(o.scale * rnd::uni(2.f))) {
		// 	// jump!
		// 	float jump = rnd::uni();
		// 	o.accel.y = jump * gravity * 2.f * o.scale;

			
		// }

		
		// add my direction to the fluid current
		//glm::vec3 push = quat_uf(o.orientation) * (creature_fluid_push * (float)dt);
		glm::vec3 push = o.velocity * (creature_fluid_push * (float)dt);
		//fluid.velocities.front().addnorm(norm, &push.x);
		al_field3d_addnorm_interp(field_dim, fluid_velocities.front(), norm, push);

		
	}

	// thread-safe:
	inline int nearest_island(glm::vec3 pos) {
		int which = -1;
		float d2 = 1000000000.f;
		for (int i=0; i<NUM_ISLANDS; i++) {
			auto r = island_centres[i] - pos;
			auto d = glm::dot(r, r);
			if (d < d2) {
				which = i;
				d2 = d;
			}
		}
		return which;
	}

	inline glm::vec3 random_location_above_land(float h=0.1f) {
		glm::vec3 result;
		glm::vec2 p;
		glm::vec4 landpt;
		int runaway = 100;
		while (runaway--) {
			p = glm::linearRand(glm::vec2(0.f), glm::vec2(1.f));
			landpt = al_field2d_readnorm_interp(land_dim2, land, p);
			result = transform(field2world, glm::vec3(p.x, landpt.w, p.y));
			if (result.y > h) {
				break;
			}
		}
		//console.log("runaway %d", runaway);
		return result;
	}

	void update_projector_loc();

	inline float ant_sniff_turn(Creature& a, float p1, float p2) {
		//-- is there any pheromone near?
		//-- (use random factor to avoid over-reliance on pheromone trails)
		float pnear = p1+p2; // - rnd::uni();
		if (pnear * rnd::uni() > ant_sniff_min) {
			//-- turn left or right?
			a.rot_vel = glm::angleAxis(p1 > p2 ? -ant_follow : ant_follow, quat_uf(a.orientation)) * a.rot_vel;
		} 
		return pnear;
	}

	inline float daymix(float time, glm::vec3 position) {
		return 0.65f + 0.3f*sin(time*0.1f + (position.x + position.z * 0.2f) * 0.004f);
	}
};

#endif