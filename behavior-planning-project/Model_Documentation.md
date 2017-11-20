# Model documentation

this model basically creates a trajectory using a cubic interpolation given the conditions of the road: some behavior is "hardcoded" on the model, for example:

* Always will do a left change if we can, we don't consider which lane change will be better.
* For a lane change we just consider the distance between them but no the velocities that could help to determine whether is good to change lanes
* We basically have 3 states ("stay on the same lane", "change left", "change right") but basically we assume that we have good drivers, so there's no collision avoidance on the model, that must be consider and even break the "confort" rules.

# improvements to do

* Create a module for the finite state machine to be able to add more states easily
* Create an MPC model that takes in consideration the other cars in the highway (also take in count that the bigger the model, is more **harder** to optimize)

# Parameters

The parameters used in the model are in the Limits namespace of [this](src/utilities.h) file.