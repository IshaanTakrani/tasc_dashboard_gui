import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self, node_name: str, topic_name: str, msg_type, period=0.5):
        super().__init__(node_name)

        # Store common attributes
        self.msg_type = msg_type
        self.pub = self.create_publisher(msg_type, topic_name, 10)

        # Timer calls overridden method
        self.timer = self.create_timer(period, self.publish_callback)

    def publish_callback(self):
        """
        Child class *must* override this.
        """
        raise NotImplementedError("Child class must implement publish_callback()")


#

class HeartbeatPublisher(Publisher):
    def __init__(self):
        super().__init__(
            node_name="heartbeat_pub",
            topic_name="heartbeat",
            msg_type=String,
            period=1.0
        )
        self.counter = 0

    def publish_callback(self):
        msg = String()
        msg.data = f"Heartbeat #{self.counter}"
        self.counter += 1
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
