import rclpy
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Transform, Vector3, PoseStamped, Pose, Twist
from std_msgs.msg import ColorRGBA, Header

from rclpy.time import Time



def MakeSphereMaker(id, pos: Point, header, color: ColorRGBA = ColorRGBA(r=1.0, a=1.0) , scale = 0.1) -> Marker:
    m = Marker()
    m.header = header
    m.type = Marker.SPHERE
    m.id = id
    m.pose.position = pos
    m.color = color
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    return m


def MakeCubeMarker(id,
                       pos: Point,
                       header = None,
                       color=ColorRGBA(r=1.0, g=0.2, b=1.0, a=1.0),
                       size=0.03,
                       alpha=0.9) -> Marker:
    m = Marker()
    if header is None:
        m.header.frame_id = "map"
        m.header.stamp = Time().to_msg()

    else:
        m.header = header
    m.type = Marker.CUBE
    m.id = id
    m.pose.position = pos
    m.color = color
    m.color.a = alpha
    m.scale.x = size
    m.scale.y = size
    m.scale.z = size
    return m

def MakeCylinderMarker(id,
                       pos: Point,
                       header,
                       color=ColorRGBA(r=1.0, g=0.2, b=1.0, a=1.0),
                       diameter=0.02,
                       height=1.8,
                       alpha=0.9) -> Marker:
    m = Marker()
    m.header = header
    m.type = Marker.CYLINDER
    m.id = id
    m.pose.position = pos
    m.pose.position.z += height / 2
    m.color = color
    m.color.a = alpha
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = height
    return m


def MakeTextMarker(id,
                   data: str,
                   pos: PointStamped,
                   color=ColorRGBA(r=0.2, g=0.1, b=1.0, a=1.0),
                   scale=0.15) -> Marker:
    m = Marker()
    m.header = pos.header
    m.type = Marker.TEXT_VIEW_FACING
    m.id = id
    m.pose.position = pos.point
    m.color = color
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    m.text = data
    return m


if __name__ == "__main__":
    rclpy.init()
    node = Node("marker")
    p = node.create_publisher(Marker , "VisualizationMarker", 1)

    def marker_fun():
        ps = PointStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.point.x = 0.0
        ps.point.y = 0.0
        ps.point.z = 2.0
        m = MakeTextMarker(10 , "Watered !", ps)
        p.publish(m)
    node.create_timer(1.0,marker_fun)
    # rclpy.spin_once( node,timeout_sec=5)
    rclpy.spin(node)