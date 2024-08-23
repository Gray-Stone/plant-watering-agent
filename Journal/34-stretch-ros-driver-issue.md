
Current issue of stretch driver.

When in pos/nav mode and commanding the joint to move.

The terminal condition when moving to each waypoint is by checking if the joint has reached goal. 

* wrist pitch have trouble reaching goal that's very close (likely due to gravity) 


The guarded timeout check around each waypoint ues a default timeout which is always 10sec. This mean can't do a long slow move that last more then 10 secs.