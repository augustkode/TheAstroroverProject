#!/usr/bin/env python3
"""
Arduino → ROS2 bridge
Publiserer sensordata som ROS2 topics.

Kilder brukt:
- https://docs.ros.org/en/humble/Tutorials/
- https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
from std_msgs.msg import String
import serial
import time
import threading
import smtplib
from email.mime.text import MIMEText


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        self.port = '/dev/ttyACM0'
        self.baud = 9600
        self.ser = None

        # ROS2 publishers
        self.temp_pub = self.create_publisher(Temperature, '/arduino/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '/arduino/humidity', 10)
        self.mode_pub = self.create_publisher(String, '/arduino/mode', 10)
        self.pir_pub = self.create_publisher(String, '/arduino/pir', 10)
        self.led_pub = self.create_publisher(String, '/arduino/led_status', 10)

        # Koble til Arduino
        self.connect_serial()

        # Start lesetråd
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.control_sub = self.create_subscription(
            String,
            '/arduino/control',
            self.control_callback,
            10
        )
    
    def control_callback(self, msg):
        command = msg.data.strip().upper()
        self.get_logger().info(f"Mottatt kommando fra GUI: {command}")
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((command + "\n").encode())
                self.get_logger().info(f"Sendte til Arduino: {command}")
            except Exception as e:
                self.get_logger().error(f"Feil ved sending til Arduino: {e}")


    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Koblet til Arduino på {self.port}")
        except Exception as e:
            self.get_logger().error(f"Kunne ikke åpne port: {e}")
            self.ser = None

    def read_loop(self):
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.connect_serial()
                time.sleep(2)
                continue

            try:
                # Be Arduino sende data
                self.ser.reset_input_buffer()
                self.ser.write(b"SENSORMEASUREMENT\n")
                time.sleep(0.2)
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line or "," not in line:
                    continue

                parts = line.split(",")
                if len(parts) == 5:
                    temp = float(parts[0])
                    hum = float(parts[1])
                    led = parts[2]
                    mode = parts[3]
                    pir = parts[4]

                    self.publish_data(temp, hum, led, mode, pir)

            except Exception as e:
                self.get_logger().error(f"Feil ved seriallesing: {e}")
                if self.ser:
                    self.ser.close()
                self.ser = None
                time.sleep(2)

            time.sleep(1)

    def send_email_alert(self, subject, message):
        sender = "astrorover3000@gmail.com"
        recipient = "astrorover3000@gmail.com"
        password = "knjf iluz xdqz ogvr"

        msg = MIMEText(message)
        msg["Subject"] = subject
        msg["From"] = sender
        msg["To"] = recipient

        try:
            with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
                server.login(sender, password)
                server.send_message(msg)
                self.get_logger().info("E-post sendt: ALARM-varsling")
        except Exception as e:
            self.get_logger().error(f"Feil ved sending av e-post: {e}")    

    def publish_data(self, temp, hum, led, mode, pir):
        tmsg = Temperature()
        tmsg.temperature = temp
        self.temp_pub.publish(tmsg)

        hmsg = RelativeHumidity()
        hmsg.relative_humidity = hum
        self.hum_pub.publish(hmsg)

        lmsg = String()
        lmsg.data = led
        self.led_pub.publish(lmsg)

        mmsg = String()
        mmsg.data = mode
        self.mode_pub.publish(mmsg)

        pmsg = String()
        pmsg.data = pir
        self.pir_pub.publish(pmsg)

        self.get_logger().info(f"T:{temp:.1f}C H:{hum:.1f}% Mode:{mode} LED:{led} PIR:{pir}")

        if mode.upper() == "ALARM":
            self.send_email_alert(
                "Alerts",
                f"Alarm active. Check system!\nTemperatur: {temp:.1f}°C\nFuktighet: {hum:.1f}%\nLED: {led}\nPIR: {pir}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Avslutter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
