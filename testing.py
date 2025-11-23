#!/usr/bin/env python3
import rclpy
from publishers import HeartbeatPublisher
from subscribers import HeartbeatSubscriber

def main():
    rclpy.init()

    # Create nodes
    heartbeat_pub = HeartbeatPublisher()
    heartbeat_sub = HeartbeatSubscriber()

    # Spin in separate executors
    try:
        from threading import Thread
        pub_thread = Thread(target=rclpy.spin, args=(heartbeat_pub,), daemon=True)
        sub_thread = Thread(target=rclpy.spin, args=(heartbeat_sub,), daemon=True)
        pub_thread.start()
        sub_thread.start()

        # Keep main alive
        pub_thread.join()
        sub_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        heartbeat_pub.destroy_node()
        heartbeat_sub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
