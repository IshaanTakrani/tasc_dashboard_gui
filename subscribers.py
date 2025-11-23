import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self, node_name: str, topic_name: str, msg_type):
        super().__init__(node_name)

        # Create subscription
        self.sub = self.create_subscription(
            msg_type,
            topic_name,
            self.subscription_callback,
            10
        )

    def subscription_callback(self, msg):
        """
        Child class *must* override this method
        """
        raise NotImplementedError("Child class must implement subscription_callback()")



class HeartbeatSubscriber(Subscriber):
    def __init__(self):
        super().__init__(
            node_name="heartbeat_sub",
            topic_name="heartbeat",
            msg_type=String
        )

    def subscription_callback(self, msg):
        self.get_logger().info(f"Received heartbeat: {msg.data}")
