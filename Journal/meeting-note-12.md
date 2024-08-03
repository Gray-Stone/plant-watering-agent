
* Moveit worked, robot issue on motor firmware. Reload firmware sovles it initially, and same error appeared again but fixed by rebooting.

* nav2 working with rtabmap. Got a video manually commanding it around the room.


Next action


* Write a ros node that output flowerpot location.
* 

* Write the overall commander. Listen to everything (map, flower, robot state, etc) issue nav-goal and moveit command.


Verify: 

Using their code, robot doesn't work in trajectory mode after some time.


* Run draw circle, verify it work.
* Do things and keep track of things. 
* periodically re-run the circle
* When it stops working, check follow fixes
    1. Send home command 
    2. Do a soft reset: sudo reboot
    3. Hard reset 
* Write up a brief report of this result.
