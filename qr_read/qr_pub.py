import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class QRCodePublisher(Node):
    def __init__(self):
        super().__init__('qr_code_publisher')

        # Open the serial port (adjust the port and baud rate as needed)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        # Create a publisher for the qr_code_topic
        self.publisher = self.create_publisher(String, 'qr_code_topic', 10)

        # Timer to periodically read and publish QR code data
        self.timer = self.create_timer(1.0, self.publish_qr_code)  # Publish every 1 second

    def publish_qr_code(self):
        # Read data from serial port
        if self.ser.in_waiting > 0:  # Check if there's data waiting
            data = self.ser.readline().decode('utf-8').strip()  # Read one line, decode from bytes to string
            self.get_logger().info(f"Publishing QR Code: {data}")

            # Create and publish the message
            msg = String()
            msg.data = data
            self.publisher.publish(msg)
        else:
            self.get_logger().info("No QR code data available")

def main(args=None):
    rclpy.init(args=args)

    qr_code_publisher = QRCodePublisher()
    rclpy.spin(qr_code_publisher)

    qr_code_publisher.ser.close()  # Close the serial port on shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
