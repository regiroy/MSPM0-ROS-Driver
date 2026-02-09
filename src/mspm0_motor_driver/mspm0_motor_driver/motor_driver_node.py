import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class MSPM0MotorDriver(Node):
    def __init__(self):
        super().__init__('mspm0_motor_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        # Odometry parameters
        self.declare_parameter('wheel_radius', 0.0335)   # meters (67mm / 2)
        self.declare_parameter('wheel_base', 0.18)       # meters
        self.declare_parameter('encoder_ticks_per_rev', 11 * 30)  # hall lines * reduction
        # Robot parameters
        self.declare_parameter('speed_scale', 500.0) # ROS m/s → MSPM0 units
        self.declare_parameter('max_speed', 1000)


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

        # Delay encoder upload until startup finishes
        self.create_timer(1.0, self.enable_encoder_upload)
        self.create_timer(0.02, self.update_odometry)  # 50 Hz

        # cmd_vel subscription
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_mtep = None
        self.last_mspd = None
        self.last_mall = None

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.last_cmd_time = self.get_clock().now()

        self.watchdog = self.create_timer(0.2, self.check_timeout)

        self.timer = self.create_timer(0.1, self.read_serial)

    def send_cmd(self, cmd: str):
        self.get_logger().info(f'SEND: {cmd.strip()}')
        self.ser.write(cmd.encode('ascii'))

    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return

            self.get_logger().info(f'RECV: {line}')

            if line.startswith('$MTEP:'):
                values = self._parse_int_values(line)
                if values is not None:
                    self.last_mtep = values
                    self.get_logger().info(f'MTEP pulses: {values}')

            elif line.startswith('$MSPD:'):
                values = self._parse_float_values(line)
                if values is not None:
                    self.last_mspd = values
                    self.get_logger().info(f'MSPD speed: {values}')

            elif line.startswith('$MAll:'):
                values = self._parse_int_values(line)
                if values is not None:
                    self.last_mall = values
                    self.get_logger().info(f'MAll total: {values}')


    def _parse_int_values(self, line):
        try:
            payload = line.split(':')[1].replace('#', '')
            parts = payload.split(',')
            if len(parts) != 4:
                return None
            return [int(p) for p in parts]
        except Exception:
            self.get_logger().warn(f'Int parse error: {line}')
            return None

    def _parse_float_values(self, line):
        try:
            payload = line.split(':')[1].replace('#', '')
            parts = payload.split(',')
            if len(parts) != 4:
                return None
            return [float(p) for p in parts]
        except Exception:
            self.get_logger().warn(f'Float parse error: {line}')
            return None


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

    def enable_encoder_upload(self):
        self.get_logger().info('Enabling encoder upload')
        self.send_cmd('$upload:1,1,1#')

    def _yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def update_odometry(self):
        if self.last_mtep is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value

        # Pulses per 10 ms → pulses per second
        pulses_left = (self.last_mtep[0] + self.last_mtep[1]) / 2.0
        pulses_right = (self.last_mtep[2] + self.last_mtep[3]) / 2.0

        pulses_left *= 100.0
        pulses_right *= 100.0

        # Convert pulses → meters
        dist_left = (pulses_left / ticks_per_rev) * (2 * math.pi * wheel_radius) * dt
        dist_right = (pulses_right / ticks_per_rev) * (2 * math.pi * wheel_radius) * dt

        dist = (dist_left + dist_right) / 2.0
        dtheta = (dist_right - dist_left) / wheel_base

        # Integrate pose
        self.x += dist * math.cos(self.theta + dtheta / 2.0)
        self.y += dist * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        self.publish_odometry(dist / dt if dt > 0 else 0.0,
                              dtheta / dt if dt > 0 else 0.0)

    def publish_odometry(self, linear_vel, angular_vel):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = self._yaw_to_quaternion(self.theta)

        msg.twist.twist.linear.x = linear_vel
        msg.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = MSPM0MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
