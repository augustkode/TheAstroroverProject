import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time
from std_msgs.msg import String

def connect_serial():
    """Find and connect to any available micro:bit (ACM/USB) port."""
    while True:
        try:
            ports = [p.device for p in serial.tools.list_ports.comports() if "ACM" in p.device or "usb" in p.device.lower()]
            if not ports:
                print("No micro:bit detected. Retrying in 3s")
                time.sleep(3)
                continue

            port = ports[0]
            print(f"Connecting to {port} ...")
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"Connected to {port}")
            return ser

        except Exception as e:
            print(f"Serial connect failed ({e}). Retrying in 3s")
            time.sleep(3)


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.driving_mode = "joystick"
        # Subscribe to /driver_mode
        self.subscription = self.create_subscription(String, '/driving_mode', self.mode_callback, 10)
        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.manual_callback, 10)
        # SUbslcibe to cmd_vel_auto
        self.subscription = self.create_subscription(Twist, '/cmd_vel_auto', self.auto_callback, 10)
        # Try connecting to micro:bit dynamically
        self.serial = connect_serial()
        self.get_logger().info('MotorDriver started (ACTION-#### format)')

        self.last_command = "S-0\n"
        self.timer = self.create_timer(0.1, self.send_last_command)

    def manual_callback(self, msg: Twist):
        if self.driving_mode == "joystick":
            self.listener_callback(msg)

    def auto_callback(self, msg: Twist):
        if self.driving_mode == "autonomous":
            self.listener_callback(msg)
    
    def mode_callback(self, msg: String):
        self.driving_mode = msg.data.strip()
        self.get_logger().info(f"Driving mode set to: {self.driving_mode}")


    def listener_callback(self, msg: Twist):
        """Converts joystick Twist message into rover movement command."""

        linear_x = msg.linear.x    # frem/bak
        linear_y = -msg.linear.y   # sideveis
        angular = msg.angular.z    # rotasjon
        

        scale = 3000
        speed_x = int(abs(linear_x) * scale)
        speed_y = int(abs(linear_y) * scale)
        speed_ang = int(abs(angular) * scale)

        command = "S-0\n"

        # --- diagonaler ---
        if abs(linear_x) > 0.2 and abs(linear_y) > 0.2:
            if linear_x > 0 and linear_y > 0:
                command = f"FR-{min(speed_x, speed_y)}\n"
            elif linear_x > 0 and linear_y < 0:
                command = f"FL-{min(speed_x, speed_y)}\n"
            elif linear_x < 0 and linear_y > 0:
                command = f"BR-{min(speed_x, speed_y)}\n"
            elif linear_x < 0 and linear_y < 0:
                command = f"BL-{min(speed_x, speed_y)}\n"

        # --- enkle retninger ---
        elif abs(linear_x) > 0.2:
            if linear_x > 0:
                command = f"F-{speed_x}\n"
            else:
                command = f"B-{speed_x}\n"
        elif abs(linear_y) > 0.2:
            if linear_y > 0:
                command = f"R-{speed_y}\n"
            else:
                command = f"L-{speed_y}\n"

        # --- rotasjon ---
        elif abs(angular) > 0.2:
            if angular > 0:
                command = f"CCW-{speed_ang}\n"   # venstre twist
            else:
                command = f"CW-{speed_ang}\n"    # høyre twist

        self.last_command = command

        self.get_logger().info(
            f"Joystick: x={linear_x:.2f}, y={linear_y:.2f}, z={angular:.2f} → Sent: {command.strip()}"
        )

    def send_last_command(self):
        """Send latest command periodically; reconnect on serial error."""
        try:
            self.serial.write(self.last_command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}. Reconnecting")
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = connect_serial()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
