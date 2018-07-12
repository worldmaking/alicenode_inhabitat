
# TODO list

Bring in the inhabitat species behaviours!!

Remove Segments

Need to decide on a reasonable world and gallery scale. 
- gallery space is maybe about 6m x 6m
- an ant in gallery space is maybe 1-2cm 
- in VR, an ant could be 50cm-100cm 
- that means the relative scale is 1:50
- so the VR space is 50 x 6 = 300m 

creatures shouldn't wander off to lands below the min height; they should turn away
- also, edge-of-world should mean death

fluid boundaries: shouldn't be toroidal, but shouldn't either drift much beyond the islands! 

creatures are kind of sliding due to collision avoidance, but instead they should turn & move

render process currently:
- upload to gpu
- for each proj   
      - draw to gbuffer
      - draw gbuffer
- for each vr eye
      - draw to gbuffer
      - draw gbuffer
- hmd submit
- draw the four fbos to the window

drawscene:
- depends on width, height, projector
- binds: kinectrgb?, null, null, noise, distance, fungus, land, fluid 
      
drawgbuffer:
- depends on destfbo, gbuffer, projector, viewport-into-fbo
- binds: gbuf1, gbuf2, gbuf3, null, distance, fungus, emission, fluid


gbuffer redesign. 
- currently:
      - ivec4 albedo (though alpha isn't currently used)
      - vec3 normal
      - vec3 position
- we may also need:
      - emission (intensity or color? if only intensity, pack into albedo.a?)
      - texcoords (for creatures and other material detail)
      - luma multiplier (e.g. the self-occlusion of creatures)
      - material properties, e.g. type, shininess, patterning parameters, etc.
- maybe the albedo can be float32 too? 
- can also consider separating luma & chroma
- see http://aras-p.info/texts/CompactNormalStorage.html#method04spheremap for a way to pack normals into vec2


Since VR needs 90fps while the projections only need 30fps, why not interleave their FBO updates?

The object.frag shader was way too expensive. To do multiple species need to use the same code with different parameters. And maybe skip hair for now.


While hashspace query seems to work, it also seems to be expensive, and potentially biased. 
I wonder if we can treat it differently, by collating a list of 'near links' in one pass, then iterating over these links to enact their effects?
Have a look at CoS, which I think did something similar



# calibration notes


First, 
Rotation & frustum is now calibrating :-)


Second,
Ground plane is getting derived ok I think. 
It would be great to consider this in a coordinate system relative to the ground, rather than to the kinect. 
Getting relative to ground will be super helpful in calibrating the two systems. Simply project a grid at the floor plane and XZ translate/rotate to line them up. 


Third,
Not verified yet whether K2 points correspond well with the Freenect2 points -- i.e. whether the calibration is portable. 
 
If not, find a way to do the capturing in alicenode-freenect, save to disk, and load into the Max patcher for calibration.  The patcher needs a raw image and a warped cloud to go with it. Freenect's cloud is based on an 'undistorted' depth image, so it seems like it should be ok? 

The Max SDK describes how to format a .jxf file:
https://cycling74.com/sdk/MaxSDK-6.0.4/html/chapter_jit_jxf.html

OR, write a jit.libfreenect2 based on the same alicenode code. 
(probably easier than porting the patcher and opencv objects to C++!)

Fourth,
Seems logical that the kinect0.cloudTransform should actually work as the viewMat, not sure why it didn't. It does in fact turn real-world meters into virtual-world meters, from the perspective of the kinect sensor. 

# better frame rates

was unable to maintain 90fps for Rift on main thread

TODO: find out what are the bottlenecks

A: the biggest one was the object shader, no surprise. Need to find a cheaper routine that can generate all species from the same code path.

Q: if this is primarily limited by the main-thread sim?
A: actually it isn't, the animation is pretty cheap.


To what extent can this be helped by interleaving sim & render, so that while the current frame is drawing, the next frame's data is being prepared:

<sim0><sim1><sim2>...
      <ren0><ren1><ren2>...

Q: can GPU upload happen on a different thread? There are some GL features that allow this (kind of like mmap)




# deferred rendering

The first pass through the scene writes into a multi-target FBO ("gBuffer"), storing base colors, world positions, normals, material properties, etc. In this first pass, vertex shaders essentially define bounding regions of geometry, and interpolatable properties such as ray directions, while frag shaders could refine the bounding regions further (via discard, glFragDepth etc) and write into the gBuffer. 
The second pass does not modify geometry any further, but uses gBuffer data to do lighting etc. and paint the final pixel colour.


# landscape computing

The landscape can be cheaply represented as a heightfield: a 2D array that returns a height value for any horizontal location in XZ space. 
This array will however also need a normal -- creatures need to know the normal in order to align to the surface. 
So one representation could be using a vec4, where xyz capture the normal, and w captures the height.
A basic rendering technique would be to deform a flat quad according to this data -- uploaded as a 2D texture -- e.g. in a frag shader.

# landscape rendering

Currently using a full-screen quad + raymarching rather than a 3D mesh deformed by height field. 

1. submit a full-screen quad, and set up ray origin/direction in the vertex shader. Then raymarch in the vertex shader based on a heightfield passed in via texture data 
or 
2. submit a mesh aligned to the floor, with dimensions equal to the texture data, and use the texture to displace the height in vertex shader.

The advantage of (1) is that we can use other methods to add more detail to the landscape, such as fractal formations etc, whereas a mesh is always going to look like triangles at the peaks. 

Currently raytracing with a commonly used sphere-tracing approach, but this is not actually accurate, and can lead to holes or other glitches when the slopes are steep. We're raytracing from near-ground level, which is the worst-case scenario.  First, the distance from ray to floor can easily be greater than ray to intersection point, which will result in an overshoot. Second, the typical step-size when looking over a near parallel floor is very small -- possibly even smaller than the mesh resolution. Third, rays that will hit the sky will require many steps, even though most of them could have easily been discarded. 

Idea: It's quite possible that simply tracing in fixed steps, rather than distance field steps, would be better. E.g., if the raw height distance (before fractal detailing is computed) is larger than the step size, there's no need to go into more detail. March in fixed steps similar to mesh resolution size until intersection happens, back up, then SDF. Or, march in fixed steps, checking simple distance to surface below. Only if this distance is less than the detail bound, do SDF to refine, and take min of this and fixed step. It might be faster. 

Idea: render a mesh as the bounding volume of the landscape. That is, if the largest floor detail is 1 meter high, add 1 meter to the mesh height when passing in. Start the ray at the bounding volume surface. Lots of rays will be avoided (the sky ones) and lots will then be cheap, as they are already very close to surface detail. The ones that don't, that puncture through, will mainly be around ridgelines. These ones will be expensive as they'll have to keep on traveling and discover the next surface beyond (or the skybox), and the mesh doesn't help here. 

Idea: use a 3D voxel texture large enough to encompass the entire possible world. We'll have some fields like that for fluid effects, motion vectors etc.; perhaps wouldn't be too difficult to maintain an bounding distance field too (helps that these distance fields do not change very rapidly). If the 3D resolution is such that a voxel covers all possible surface detail, then we can simply raymarch to that voxel, then drop into SDF detail at that point. If a ray punctures through, drop back into voxel raymarching until the next close surface or the far clip plane.  This method can be combined with the bounding mesh idea above. I.e. voxels define a distance field that safely bounds the landscape; while a detailed SDF refines the surface below voxel resolution.

Another way: use an octree, so that large empty spaces can rapidly be skipped. But writing an accurate octree sampler for the frag shader isn't easy.

Idea: if the landscape normal points away from the ray, abort and render black (you're inside the mountain). 

# simulation

