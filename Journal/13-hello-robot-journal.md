
## Notes from user manual

https://docs.hello-robot.com/0.3/getting_started/writing_code/

Stretch Body : low level python interface. 

Can directly command joints and run startup.

`stretch_system_check.py` 
`stretch_robot_home.py`

or just in a interactive python env:

```
import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()
```

need to close this connection afterwords: `robot.stop()`. Seems like resource will get hogged if not released properly.

move to stow pose
`robot.stow()` 

ABS move of one joint 

```
robot.arm.move_to(0.25)
robot.push_command()
robot.lift.move_to(0.55)
robot.push_command()
```

Relative move 
```

robot.lift.move_by(0.1)
robot.push_command()

robot.lift.move_by(-0.1)
robot.push_command()
```

Print status  `robot.pretty_print()` 

Directly move robot head to pre-set location: 

```
robot.head.pose('tool')
robot.head.pose('ahead')
robot.head.pose('wheels')
robot.head.pose('back')
```


## Files on the robot


Stretch come with a user account and installed with all program and files.

`stretch_user` have certain calibration files in it. These files are unique to individual robot (not available online). It also have maps and ML models.

`ament_ws` is the ros workspace.

https://docs.hello-robot.com/0.3/developing/basics/#the-home-folder

For other users to use this, there needs to be copies of this folder. 

There is a backup at `/etc/hello-robot`. However this might get outdated.

### settings in the default account

Following content is under the `hello-robot` user account's `.bashrc`

```
######################
# STRETCH BASHRC SETUP
######################
export HELLO_FLEET_PATH=/home/hello-robot/stretch_user
export HELLO_FLEET_ID=stretch-se3-3047
export PATH=${PATH}:~/.local/bin
export LRS_LOG_LEVEL=None #Debug
export PYTHONWARNINGS='ignore:setup.py install is deprecated,ignore:Invalid dash-separated options,ignore:pkg_resources is deprecated as an API,ignore:Usage of dash-separated'
export _colcon_cd_root=/home/hello-robot
source /opt/ros/humble/setup.bash
source /home/hello-robot/ament_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

export ROS_DOMAIN_ID=31
source /home/hello-robot/ament_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
```

So there are a few env-var being exported, the `HELLO_FLEET_PATH` seems to indicate the stretch-user folder. It is mentioned in some parts of the system [Environment Variables](https://docs.hello-robot.com/0.3/developing/basics/#environment-variables). 


Just copying the file is not enough, there are some python library for stretch installed in the default user that's necessary.

## New dev account 

https://docs.hello-robot.com/0.3/installation/add_new_user/

Run this setup commands in this script if wanting to setup a new developer account.

The script being run will do the following:

* set the desktop setting to never dim or sleep the machine 
* If `HELLO_FLEET_ID` is found count as updating, skip next part
    * If not, count as new install,
    * put env-args and sourcing command into .bashrc (only working for bash , need to self port to zsh)
* Creates the `stretch_user` folder if not exist
* rm `ament_ws` and re-create it.
* Set 4 `.desktop` file into `~/.config/autostart`. This seems to be letting gnome desktop to auto launch this when user login (via GUI)
* Install arduino CLI 
* Add user to some groups to have access to devices
* pip install many `hello-robot-stretch-*` packages (seems to be robot drivers)


## Web teleop 

https://docs.hello-robot.com/0.3/getting_started/demos_web_teleop/

Simply run the commands under the `hello-robot` account, its quite simple. However seems like joystick teleop doesn't work after running this demo.
```
hello-robot@stretch-se3-3047:~/ament_ws/src/stretch_web_teleop$ ./launch_interface.sh
Done!
No screen session found.
Using pkill
[sudo] password for hello-robot: 
[PM2] Spawning PM2 daemon with pm2_home=/home/hello-robot/.pm2
[PM2] PM2 Successfully daemonized
[PM2][WARN] No process found
[PM2] [v] All Applications Stopped
[PM2] [v] PM2 Daemon Stopped
[PM2] Spawning PM2 daemon with pm2_home=/home/hello-robot/.pm2
[PM2] PM2 Successfully daemonized
[PM2] Starting /usr/bin/npm in fork_mode (1 instance)
[PM2] Done.

Visit the URL(s) below to see the web interface:
https://localhost/operator
https://192.168.18.198/operator
```
According to the log, I'm guessing joy-stick process is being killed for this process to take over the robot.

