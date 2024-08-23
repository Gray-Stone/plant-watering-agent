
Since it will be very far form fixing stretch ik solver, the decision is made to ignore collision checking, assuming the plant just have a clear env in front of it.

Have the arm do almost a series of dead-reconing moves.


## Problem with traj mode

The problem with traj mode is the ros driver is still very not stable. So position mode must be used. 

Position mode will move the joints very aggressively if not given velocity.

The pos mode will take first point in traj as some trajectory to go to. Which mean if the robot's current joint is offset to somewhere, it might be a small issue.

Also if error arise when commanding position control, some of the joints like lift and arm will just go on forever.