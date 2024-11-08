import rclpy
from rclpy.node import Node
from rclpy.servers import Service
from example_interfaces.srv import String  # Import the service type
import serial
from qr_read.srv import ScanQRCode  # Replace with your actual package and service file

class QRCodeScannerService(Node):
    def __init__(self):
        super().__init__('qr_code_scanner_service')

        # Open the serial port (adjust the port and baud rate as needed)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        # Create a service that will respond with the QR code number
        self.srv = self.create_service(
            ScanQRCode,
            'scan_qr_code',
            self.handle_scan_qr_code
        )

    def handle_scan_qr_code(self, request, response):
        # Read data from serial port when the service is called
        if self.ser.in_waiting > 0:  # Check if there's data waiting
            data = self.ser.readline().decode('utf-8').strip()  # Read one line, decode from bytes to string
            self.get_logger().info(f"Received QR Code: {data}")  # Print received data
            response.qr_code_number = data  # Set the response with the QR code number
        else:
            response.qr_code_number = "No data available"  # In case no data is available
        return response

def main(args=None):
    rclpy.init(args=args)

    qr_code_scanner_service = QRCodeScannerService()
    rclpy.spin(qr_code_scanner_service)

    qr_code_scanner_service.ser.close()  # Close the serial port on shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
