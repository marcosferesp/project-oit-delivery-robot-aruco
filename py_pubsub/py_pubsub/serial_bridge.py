import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.publisher_ = self.create_publisher(String, '/ultrasonic_data', 10)
        
        # We REMOVED the create_timer! No more main-thread starvation.
        self.running = True
        self.ser = None
        
        try:
            # Connect to ESP32 with a comfortable 500ms timeout
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.5)
            self.get_logger().info('Connected to ESP32 on /dev/ttyACM0 at 115200 baud')
            
            # Start a dedicated background thread strictly for reading serial
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')

    def serial_read_loop(self):
        # This loop runs in the background and NEVER blocks ROS 2 network discovery
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line and ',' in line:
                    self.get_logger().info(f"Publishing to network: {line}")
                    msg = String()
                    msg.data = line
                    # publish() is thread-safe in ROS 2; this pushes to the network instantly
                    self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Serial read warning: {e}')

    def destroy_node(self):
        self.running = False
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()