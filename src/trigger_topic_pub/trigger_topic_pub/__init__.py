import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TriggerPublisher(Node):
    def __init__(self):
        super().__init__('trigger_publisher')
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(String, '/trigger_detect', 10)
        
        # 퍼블리시
        msg = String()
        msg.data = 'trigger detected.'
        self.publisher_.publish(msg)
        time.sleep(0.1)
        self.publisher_.publish(msg)
        
def main(args=None):
    try:
        rclpy.init(args=args)
        node = TriggerPublisher()

        rclpy.spin_once(node)

        node.destroy_node()
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
