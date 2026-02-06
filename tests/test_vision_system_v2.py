import rclpy
from rclpy.node import Node
from logistics_interfaces.srv import AnalyzeScene, SetServoMode
from logistics_interfaces.msg import ServoFeedback
import time

class TestVisionSystemV2(Node):
    def __init__(self):
        super().__init__('test_vision_system_v2')
        
        # Clients
        self.cli_analyze = self.create_client(AnalyzeScene, 'analyze_scene')
        self.cli_mode = self.create_client(SetServoMode, 'set_servo_mode')
        
        # Subscriber
        self.sub_feedback = self.create_subscription(ServoFeedback, 'servo_feedback', self.feedback_callback, 10)
        self.feedback_received = False
        
        self.get_logger().info("Waiting for services...")
        self.cli_analyze.wait_for_service(timeout_sec=5.0)
        self.cli_mode.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("Services available.")

    def feedback_callback(self, msg):
        self.feedback_received = True
        self.get_logger().info(f"Feedback received: Mode={msg.current_mode}, ErrorX={msg.error_x:.3f}")

    def test_analysis(self):
        req = AnalyzeScene.Request()
        req.trigger = True
        future = self.cli_analyze.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info(f"Analysis Success! Found {len(res.targets)} targets.")
            for t in res.targets:
                self.get_logger().info(f"  Target: {t.id} Layer: {t.layer_index} Strategy: {t.strategy}")
        else:
            self.get_logger().error("Analysis Failed.")

    def test_servo_mode(self):
        req = SetServoMode.Request()
        req.mode = "FACE_ALIGN"
        future = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.success:
            self.get_logger().info("Set Mode Success! Waiting for feedback...")
            # Wait for feedback
            start = time.time()
            while not self.feedback_received and (time.time() - start) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.feedback_received:
                self.get_logger().info("Feedback verified.")
            else:
                self.get_logger().warn("No feedback received (Check camera input).")
        else:
            self.get_logger().error("Set Mode Failed.")

def main(args=None):
    rclpy.init(args=args)
    tester = TestVisionSystemV2()
    
    print("\n--- Testing Scene Analysis ---")
    tester.test_analysis()
    
    print("\n--- Testing Visual Servo Mode ---")
    tester.test_servo_mode()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
