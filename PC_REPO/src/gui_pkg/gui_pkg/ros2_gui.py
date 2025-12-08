"""
Documentation used:
https://docs.ros2.org/foxy/api/sensor_msgs/msg/FluidPressure.html
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html?utm_source=chatgpt.com
https://gist.github.com/robosam2003/9e8cb1d8581ddd3af098a8813c64e71e threading
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html?utm_source=chatgpt.com#write-the-subscriber-node
https://stackoverflow.com/questions/72690021/how-to-process-a-image-message-with-opencv-from-ros2
https://www.pythonguis.com/docs/qpushbutton/?utm_source=chatgpt.com
https://www.youtube.com/watch?v=92zx_U9Nzf4
https://stackoverflow.com/questions/10381854/how-to-create-screenshot-of-qwidget 

"""


import sys
from PyQt5 import QtWidgets
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtCore import QUrl
from gui_pkg.cars_gui import Ui_MainWindow 
from rclpy.node import Node
import rclpy #ros2
from std_msgs.msg import String
from sensor_msgs.msg import FluidPressure
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Temperature, RelativeHumidity
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSignal
import gui_pkg.cars_gui as Interface_ui
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PyQt5.QtGui import QImage, QPixmap
from datetime import datetime
from pathlib import Path
from sensor_msgs.msg import LaserScan

class Interface(QMainWindow, Interface_ui.Ui_MainWindow):
    pressure_signal = pyqtSignal(float)
    temperature_signal = pyqtSignal(float)
    humidity_signal = pyqtSignal(float)
    image_signal = pyqtSignal(QImage)
    label_image_signal = pyqtSignal(str)
    cpu_temp_signal = pyqtSignal(float)
    arduino_temp_signal = pyqtSignal(float)
    arduino_hum_signal = pyqtSignal(float)
    arduino_led_signal = pyqtSignal(str)
    arduino_mode_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str)
    lidar_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super(Interface, self).__init__(parent)
        self.setupUi(self)
        self.log_file = Path.home() / "A" / "Smarte-Systemer---Rover" / "ros2_ws" / "sensor_log.txt"
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self.btnOpenLog.clicked.connect(self.open_log_file)
        self.btnControlDrivingMode.clicked.connect(self.set_joystick_mode)
        self.btnAutonomousDrivingMode.clicked.connect(self.set_autonomous_mode)
        self.btnTakePhoto.clicked.connect(self.take_picture)
        self.btnAuto.clicked.connect(lambda: self.send_command("AUTO"))
        self.btnOn.clicked.connect(lambda: self.send_command("ON"))
        self.btnOff.clicked.connect(lambda: self.send_command("OFF"))
        self.btnGuard.clicked.connect(lambda: self.send_command("GUARDIAN"))
        self.rosnode = None

        self.pressure_signal.connect(self.update_pressure_label)
        self.temperature_signal.connect(self.update_temperature_label)
        self.humidity_signal.connect(self.update_humidity_label)
        self.image_signal.connect(self.update_camera_image)
        self.cpu_temp_signal.connect(self.update_cpu_temp_label)
        self.label_image_signal.connect(self.update_camera_label)
        self.lidar_signal.connect(self.update_lidar_label)

        self.arduino_temp_signal.connect(self.update_arduino_temp_label)
        self.arduino_temp_signal.connect(self.update_arduino_temp_label_2)
        self.arduino_hum_signal.connect(self.update_arduino_hum_label)
        self.arduino_hum_signal.connect(self.update_arduino_hum_label_2)
        self.arduino_led_signal.connect(self.update_arduino_led_label)
        self.arduino_led_signal.connect(self.update_arduino_led_label_2)
        self.arduino_mode_signal.connect(self.update_arduino_mode_label)

        self.log_signal.connect(self._append_log_line)
    
    def update_lidar_label(self, status):
        self.lblLidarStatus_3.setText(f"Lidar: {status}")

    def update_pressure_label(self, value):
        self.lblPressuseSenseHAT.setText(f"Pressure: {value:.1f} hPa")
    
    def update_temperature_label(self, value):
        self.lblTemperature_Sense_HAT.setText(f"System temperature: {value:.1f} C")

    def update_humidity_label(self, value):
        self.lblHumidity_Sense_HAT.setText(f"System humidity: {value:.1f} %")

    def update_camera_image(self, qimg):
           pix = QPixmap.fromImage(qimg)
           self.lblVideoStream.setPixmap(pix)
           self.lblVideoStream.setScaledContents(True)
        
    def update_camera_label(self, status):
        self.lblCamStatus.setText(f"Cam: {status}") 

    def update_arduino_temp_label(self, value):
        self.lblTemperature.setText(f"Env Temperature: {value:.1f} °C")

    def update_arduino_temp_label_2(self, value):
        self.lblTemperature_2.setText(f"Env Temperature: {value:.1f} C")

    def update_arduino_hum_label(self, value):
        self.lblHum.setText(f"Env Humidity: {value:.1f} %")
    
    def update_arduino_hum_label_2(self, value):
        self.lblHum_2.setText(f"Env Humidity: {value:.1f} %")

    def update_arduino_led_label(self, status):
        self.lblLedStatus.setText(f"LED: {status}")
    
    def update_arduino_led_label_2(self, status):
        self.lblLedStatus_2.setText(f"Led: {status}")

    def update_arduino_mode_label(self, mode):
        self.lblArduinoMode.setText(f"Arduino Mode: {mode}")

    def update_cpu_temp_label(self, value):
        self.lblTemperatureRPi4Processor.setText(f"Processor temperature: {value:.1f} °C")

    def _append_log_line(self, line: str):
        self.txtLog.append(line)
        sb = self.txtLog.verticalScrollBar()
        sb.setValue(sb.maximum())
        try:
            with open(self.log_file, "a") as f:
                f.write(line + "\n")
        except Exception:
            pass
    
    def write_sensor_log(self, text: str):
        try:
            with open(self.log_file, "a") as f:
                f.write(text + "\n")
        except Exception:
            pass


    
    def open_log_file(self):
        try:
        
            if not self.log_file.exists():
                self.log_file.touch()
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.log_file)))
            self.txtLog.append(f"[INFO] Opened log file: {self.log_file}")
        except Exception as e:
            self.txtLog.append(f"[ERROR] Could not open log file: {e}")


    def set_joystick_mode(self):
        if self.rosnode:
            self.rosnode.publish_driving_mode("joystick")
            self.lblDrivingMode.setText("Driving mode: JOYSTICK")
            self.lblDrivingMode_2.setText("Driving mode: JOYSTICK")
            self.log_signal.emit(f"[INFO] Driving Mode set to joystick")

    def set_autonomous_mode(self):
        if self.rosnode:
            self.rosnode.publish_driving_mode("autonomous")
            self.lblDrivingMode.setText("Driving mode: AUTONOMOUS")
            self.lblDrivingMode_2.setText("Driving mode: AUTONOMOUS")
            self.log_signal.emit(f"[INFO] Driving Mode set to autonomous")

    def take_picture(self):
        pixmap = self.lblVideoStream.grab()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        photo_dir = Path.home() / "A" / "Smarte-Systemer---Rover" / "ros2_ws" / "Astro_Pictures"
        photo_dir.mkdir(parents=True, exist_ok=True)
        filename = photo_dir / f"photo_{timestamp}.png"
        if pixmap.save(str(filename), "PNG"):
            self.log_signal.emit(f"[INFO] Saved photo: {filename}")
            print(f"[INFO] Saved photo: {filename}")
        else:
            self.log_signal.emit(f"[ERROR] Could not save photo: {filename}")
            print(f"[ERROR] Could not save photo: {filename}")
    
    def send_command(self, cmd: str):
        if self.rosnode:
            self.rosnode.send_arduino_control(cmd)
            print(f"[GUI] Sent command: {cmd}")
            self.log_signal.emit(f"[INFO] Arduino set to {cmd}")

class InterfaceNode(Node):
    def __init__(self, interface: Interface):
        super().__init__('ros2_gui_node')
        self.interface = interface
        self.bridge = CvBridge()
        self.get_logger().info('ROS2 InterfaceNode startet')

        self.driving_mode_publisher = self.create_publisher(String, '/driving_mode', 10)

        self.pressure_subscription = self.create_subscription(
            FluidPressure,
            '/pressure',
            self.pressure_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.sense_temp_subscription = self.create_subscription(
            Temperature,
            '/temp_h',
            self.sense_temp_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.sense_hum_subscription = self.create_subscription(
            RelativeHumidity,
            '/humidity',
            self.sense_hum_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.driving_mode_subscription = self.create_subscription(
            String,
            '/driving_mode',
            self.driving_mode_callback,
            10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.cpu_temp_subscription = self.create_subscription(
            Temperature,
            '/cpu_temp',
            self.cpu_temp_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.driving_mode_publisher = self.create_publisher(
            String,
            '/driving_mode',
            10
        )

        self.arduino_temp_subscription = self.create_subscription(
            Temperature,
            '/arduino/temperature',
            self.arduino_temp_callback,
            10
        )

        self.arduino_hum_subscription = self.create_subscription(
            RelativeHumidity,
            '/arduino/humidity',
            self.arduino_hum_callback,
            10
        )

        self.arduino_mode_subscription = self.create_subscription(
            String,
            '/arduino/mode',
            self.arduino_mode_callback,
            10
        )

        self.arduino_led_subscription = self.create_subscription(
            String,
            '/arduino/led_status',
            self.arduino_led_callback,
            10
        )

        self.arduino_alarm_subscription = self.create_subscription(
            String,
            '/arduino/alarm',
            self.arduino_alarm_callback,
            10
        )

        self.arduino_control_pub = self.create_publisher(
            String,
            '/arduino/control',
            10
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )



    def pressure_callback(self, msg: FluidPressure):
        pressure_hpa = round(msg.fluid_pressure / 100.0, 1)
        self.get_logger().info(f"Pressure:{pressure_hpa} hpa")
        self.interface.update_pressure_label(pressure_hpa)
        self.interface.pressure_signal.emit(pressure_hpa)
        self.interface.write_sensor_log(f"Pressure: {pressure_hpa} hPa")
        
    def sense_temp_callback(self, msg: Temperature):
        temp_c = msg.temperature
        self.get_logger().info(f"Temperature: {temp_c:.1f} C")
        self.interface.temperature_signal.emit(temp_c)
        self.interface.write_sensor_log(f"SenseHAT Temp: {temp_c:.1f} C")
    
    def cpu_temp_callback(self, msg: Temperature):
        temp_c = msg.temperature
        self.get_logger().info(f"CPU Temperature: {temp_c:.1f} °C")
        self.interface.cpu_temp_signal.emit(temp_c)
        self.interface.write_sensor_log(f"CPU Temp: {temp_c:.1f} °C")

    def gui_log(self, msg: str):
        ts = datetime.now().timestamp()
        self.interface.log_signal.emit(f"[INFO] [{ts:.9f}] [ros2_gui_node]: {msg}")
    
    def sense_hum_callback(self, msg: RelativeHumidity):
        hum_pros = msg.relative_humidity
        self.get_logger().info(f"Humidity: {hum_pros:.1f} %")
        self.interface.humidity_signal.emit(hum_pros)
        self.interface.write_sensor_log(f"SenseHAT Humidity: {hum_pros:.1f} %")

    def driving_mode_callback(self, msg: String):
        self.get_logger().info(f"Driving mode: {msg.data}")
    
    def lidar_callback(self, msg: String):
        self.interface.lidar_signal.emit("CONNECTED")
        self.interface.write_sensor_log(f"Lidar connected")


    def image_callback(self, msg: Image):
        self.interface.label_image_signal.emit("CONNECTED")
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.interface.image_signal.emit(qimg)
    
    def arduino_temp_callback(self, msg: Temperature):
        temp = msg.temperature
        self.get_logger().info(f"Arduino temperature: {temp:.1f} °C")
        self.interface.arduino_temp_signal.emit(temp)
        self.interface.write_sensor_log(f"Arduino Temp: {temp:.1f} °C")

    def arduino_hum_callback(self, msg: RelativeHumidity):
        hum = msg.relative_humidity
        self.get_logger().info(f"Arduino humidity: {hum:.1f} %")
        self.interface.arduino_hum_signal.emit(hum)
        self.interface.write_sensor_log(f"Arduino Humidity: {hum:.1f} %")

    def arduino_led_callback(self, msg: String):
        status = msg.data
        self.get_logger().info(f"Arduino LED status: {status}")
        self.interface.arduino_led_signal.emit(status)
        self.interface.write_sensor_log(f"Arduino LED: {status}")

    def arduino_mode_callback(self, msg: String):
        mode = msg.data
        self.get_logger().info(f"Arduino mode: {mode}")
        self.interface.arduino_mode_signal.emit(mode)
        self.interface.write_sensor_log(f"Arduino Mode: {mode}")

    def arduino_alarm_callback(self, msg: String):
        if msg.data.upper() == "ALARM":
            self.get_logger().warn("ALARM FROM ARDUINO")
            self.gui_log(f"Arduino alarm")


    def publish_driving_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.get_logger().info(f"Publishing driving mode: {mode}")
        self.driving_mode_publisher.publish(msg)
    
    def send_arduino_control(self, command: str):
        msg = String()
        msg.data = command
        self.get_logger().info(f"Publishing command to Arduino: {command}")
        self.arduino_control_pub.publish(msg)
  
# https://gist.github.com/robosam2003/9e8cb1d8581ddd3af098a8813c64e71e
def main(args=None):
    rclpy.init(args=args)

    # Run the interface and the ros node using multithreaded executor
    app = QApplication(sys.argv)
    gui = Interface()

    # Create the interface node
    node = InterfaceNode(gui)
    gui.rosnode = node

    # Setup the interface with the ros node
    #gui.setup(node)

    # Create a multithreaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Start the ROS2 node in a seperate thread
    thread = threading.Thread(target=executor.spin)
    thread.start()
    node.get_logger().info('ROS2 node started')

    # App running in main thread
    try: 
        gui.show()
        sys.exit(app.exec_())
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
