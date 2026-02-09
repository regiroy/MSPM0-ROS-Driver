import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class MSPM0MotorDriver(Node):
    def __init__(self):
        super().__init__('mspm0_motor_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.get_logger().info(f'Opening serial port {port} @ {baud}')

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1
        )

        time.sleep(2.0)  # allow board reset

        self.send_cmd('$read_flash#')


        # Robot parameters
        self.declare_parameter('wheel_base', 0.18)   # meters
        self.declare_parameter('speed_scale', 500.0) # ROS m/s → MSPM0 units
        self.declare_parameter('max_speed', 1000)

        # cmd_vel subscription
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_cmd_time = self.get_clock().now()

        self.watchdog = self.create_timer(0.2, self.check_timeout)

        self.timer = self.create_timer(0.1, self.read_serial)

    def send_cmd(self, cmd: str):
        self.get_logger().info(f'SEND: {cmd.strip()}')
        self.ser.write(cmd.encode('ascii'))

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.get_logger().info(f'RECV: {line}')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        wheel_base = self.get_parameter('wheel_base').value
        scale = self.get_parameter('speed_scale').value
        max_speed = self.get_parameter('max_speed').value

        # Differential drive kinematics
        v_left = v - (w * wheel_base / 2.0)
        v_right = v + (w * wheel_base / 2.0)

        # Scale and clamp
        left_cmd = int(max(-max_speed, min(max_speed, v_left * scale)))
        right_cmd = int(max(-max_speed, min(max_speed, v_right * scale)))

        cmd = f"$spd:{left_cmd},{left_cmd},{right_cmd},{right_cmd}#"
        self.send_cmd(cmd)


    def destroy_node(self):
        self.get_logger().info('Stopping motors...')
        try:
            self.send_cmd('$pwm:0,0,0,0#')
        except Exception:
            pass
        self.ser.close()
        super().destroy_node()

    def check_timeout(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds
        if dt > 500_000_000:  # 0.5 seconds
            self.send_cmd('$spd:0,0,0,0#')

def main():
    rclpy.init()
    node = MSPM0MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
