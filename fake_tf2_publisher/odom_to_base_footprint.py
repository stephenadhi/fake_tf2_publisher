import rclpy
from rclpy.node import Node

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped


class MaptoOdomTFPublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('odom_to_base_footprint_tf2_publisher')

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'locobot/odom'
        self.odom_trans.child_frame_id = 'locobot/base_footprint'

        self.subscription = self.create_subscription(
            Odometry, 'odom', self.listener_callback, 10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        # update transform
        now = self.get_clock().now()
        self.odom_trans.header.stamp = now.to_msg()

        self.odom_trans.transform.translation.x = data.pose.pose.position.x
        self.odom_trans.transform.translation.y = data.pose.pose.position.y
        self.odom_trans.transform.translation.z = data.pose.pose.position.z
        self.odom_trans.transform.rotation.x = data.pose.pose.orientation.x
        self.odom_trans.transform.rotation.y = data.pose.pose.orientation.y
        self.odom_trans.transform.rotation.z = data.pose.pose.orientation.z
        self.odom_trans.transform.rotation.w = data.pose.pose.orientation.w

        # transform
        self.broadcaster.sendTransform(self.odom_trans)


def main():
    node = MaptoOdomTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()