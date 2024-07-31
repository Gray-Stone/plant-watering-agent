import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
import hello_helpers.hello_misc as hm
import numpy as np

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


class DrawCircleNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'draw_circle', 'draw_circle', wait_for_first_pointcloud=False)

        arm_init = 0.1
        lift_init = 1.0
        n = 20
        diameter_m = 0.2
        time_dt = 0.75
        globalv_m = None
        globala_m = None

        t = np.linspace(0, 2*np.pi, n, endpoint=True)
        x = (diameter_m/2) * np.cos(t) + arm_init
        y = (diameter_m/2) * np.sin(t) + lift_init
        circle_mat = np.c_[x, y]

        circle_traj = FollowJointTrajectory.Goal()
        # circle_traj.trajectory.header.stamp = self.get_clock().now().to_msg()
        circle_traj.trajectory.joint_names = ['wrist_extension', 'joint_lift' , 'joint_wrist_yaw']
        for i in range(n):
            pt = circle_mat[i]
            pt_t = i * time_dt
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=pt_t).to_msg()
            point.positions = [pt[0], pt[1] , pt[1]]
            circle_traj.trajectory.points.append(point)
            print(f"appending point {point}")

        self.trajectory_client.send_goal_async(circle_traj)
        rclpy.spin(self)


def main():
    rclpy.init()

    try:
        node = DrawCircleNode()
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

    rclpy.shutdown()

if __name__ == '__main__':
    main()