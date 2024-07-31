

At some point, starting to get error on launching the realsense camera. After many failure and attempt, the D435 device seems to be disappeared from the machine, checked with v42l, D435 is not showing up.

After a reboot and powered off for 10+ minutes, the camera showed up after reboot. 

However everytime launching the camera resulted in error and failed to bringup the camera. The error is similar to this link.

https://github.com/IntelRealSense/librealsense/issues/9829

After running the stretch_setup new user process, the problem seems to be solved. However this also means re-setting the ros workspace completely.

Seems like the issue is caused by `DCMAKE_BUILD_TYPE=RelWithDebInfo` argument for colcon build. 