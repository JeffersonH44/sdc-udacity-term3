# Model documentation

I basically have a model that does a cubic interpolation to generate trajectories,
in order to generate a smooth trajectories to avoid breaking the "confort" rules
I generate 50 points that also takes in count if there was a previous path on the 
simulator (to avoid any crazy car behavior), also to make te code clear
I transform the coordinates given by the simulator (global coordinates) to 
car coordinates to make the interpolation, then I swap it back to the simulator
coordinates.

For the trajectory selection I have 3 "hardcoded" states ("stay", "left change", 
"right change"). what the model does is only reduce the speed if we have a car
in front and check whether or not is good to do a lane change given a safe distance.

There's some assumptions that I made for the model:

* Always will do a left change if we can, we don't consider which lane change will be better.
* For a lane change we just consider the distance between them but no the velocities that could help to determine whether is good to change lanes
* Given the states that I have, I assume that we have good drivers, so there's no collision avoidance on the model that must be consider and even break the "confort" rules.

# Future work

* Create a module for the finite state machine to be able to add more states easily.
* Create an MPC model that takes in consideration the other cars in the highway (also take in count that the bigger the model, is more **harder** to optimize).

# Parameters

The parameters used in the model are in the Limits namespace of [this](src/utilities.h) file.