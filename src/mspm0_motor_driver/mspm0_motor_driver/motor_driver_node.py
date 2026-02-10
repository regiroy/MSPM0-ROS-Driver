import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus


class MSPM0MotorDriver(Node):
    def __init__(self):
        super().__init__('mspm0_motor_driver')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('wheel_radius', 0.0335)   # meters
        self.declare_parameter('wheel_base', 0.18)       # meters
        self.declare_parameter('encoder_ticks_per_rev', 11 * 30)

        self.declare_parameter('speed_scale', 500.0)
        self.declare_parameter('max_speed', 1000)

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.get_logger().info(f'Opening serial port {port} @ {baud}')

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1
        )

        # ---------------- Diagnostics ----------------
        self.updater = Updater(self)
        self.updater.setHardwareID('mspm0_motor_driver')

        self.serial_ok = True
        self.last_serial_rx_time = self.get_clock().now()
        self.battery_voltage = None

        self.updater.add('Motor Driver Status', self.diag_driver_status)
        self.updater.add('Battery Voltage', self.diag_battery)

        # ---------------- Startup ----------------
        time.sleep(2.0)
        self.send_cmd('$read_flash#')

        # ---- Serial state ----
        self.last_mtep = None
        self.last_mspd = None
        self.last_mall = None

        # ---------------- Timers ----------------
        self.create_timer(0.02, self.update_odometry)     # 50 Hz
        self.create_timer(0.1, self.read_serial)           # Serial RX
        self.create_timer(0.2, self.check_timeout)         # Watchdog
        self.encoder_timer = self.create_timer(1.0, self.enable_encoder_upload)
        self.create_timer(1.0, self.updater.update)
        self.create_timer(5.0, self.request_voltage)       # ✅ single voltage poll

        # ---------------- ROS Interfaces ----------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------------- Odometry State ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.is_stopped = True
        self.last_rx_log_time = self.get_clock().now()
        self.rx_log_period_sec = 1.0  # log at most once per second

        self.last_cmd_time = self.get_clock().now()

        self.watchdog = self.create_timer(0.2, self.check_timeout)
        self.timer = self.create_timer(0.1, self.read_serial)
        # ✅ Battery / voltage polling timer
        self.create_timer(5.0, lambda: self.send_cmd('$read_vol#'))

        # Accumulated encoder pulses since last odom update
        self.accum_left_ticks = 0.0
        self.accum_right_ticks = 0.0

        # ---------------- Logging Throttle ----------------
        self.last_rx_log_time = self.get_clock().now()
        self.rx_log_period_sec = 1.0

        self.last_cmd_time = self.get_clock().now()
        self.is_stopped = True


    def send_cmd(self, cmd: str):
        self.get_logger().info(f'SEND: {cmd.strip()}')
        self.ser.write(cmd.encode('ascii'))

    def read_serial(self):
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except Exception:
                self.serial_ok = False
                return

            if not line:
                return

            # Mark serial alive
            self.serial_ok = True
            self.last_serial_rx_time = self.get_clock().now()

            # Throttled RX debug log
            now = self.get_clock().now()
            if (now - self.last_rx_log_time).nanoseconds > self.rx_log_period_sec * 1e9:
                self.get_logger().debug(f'RECV: {line}')
                self.last_rx_log_time = now

            # ---------- Parse messages ----------
            if line.startswith('$MTEP:'):
                values = self._parse_int_values(line)
                if values is not None:
                    self.last_mtep = values

                    # Average front + rear per side
                    left = (values[0] + values[1]) / 2.0
                    right = (values[2] + values[3]) / 2.0
                    self.accum_left_ticks += left
                    self.accum_right_ticks += right

            elif line.startswith('$MSPD:'):
                values = self._parse_float_values(line)
                if values is not None:
                    self.last_mspd = values

            elif line.startswith('$MAll:'):
                values = self._parse_int_values(line)
                if values is not None:
                    self.last_mall = values

            elif line.startswith('$Battery:'):
                try:
                    self.battery_voltage = float(
                        line.replace('$Battery:', '').replace('V#', '')
                    )
                except Exception:
                    self.get_logger().warn(f'Battery parse failed: {line}')

            else:
                # Expected during boot / flash read
                self.get_logger().debug(f'Ignoring serial message: {line}')


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

        left_cmd = int(max(-max_speed, min(max_speed, v_left * scale)))
        right_cmd = int(max(-max_speed, min(max_speed, v_right * scale)))

        cmd = f"$spd:{left_cmd},{left_cmd},{right_cmd},{right_cmd}#"
        self.send_cmd(cmd)

        # IMPORTANT: mark robot as active again
        self.is_stopped = False


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
            if not self.is_stopped:
                self.get_logger().warn('cmd_vel timeout — stopping motors')
                self.send_cmd('$spd:0,0,0,0#')
                self.is_stopped = True


    def enable_encoder_upload(self):
        self.get_logger().info('Enabling encoder upload')
        self.send_cmd('$upload:1,1,1#')
        self.encoder_timer.cancel()

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

        # Consume accumulated encoder pulses
        left_ticks = self.accum_left_ticks
        right_ticks = self.accum_right_ticks

        # Reset accumulators
        self.accum_left_ticks = 0.0
        self.accum_right_ticks = 0.0


        # Convert pulses → meters
        dist_left = (left_ticks / ticks_per_rev) * (2 * math.pi * wheel_radius)
        dist_right = (right_ticks / ticks_per_rev) * (2 * math.pi * wheel_radius)


        dist = (dist_left + dist_right) / 2.0
        dtheta = (dist_right - dist_left) / wheel_base

        # Integrate pose
        self.x += dist * math.cos(self.theta + dtheta / 2.0)
        self.y += dist * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        self.publish_odometry(dist / dt if dt > 0 else 0.0,
                              dtheta / dt if dt > 0 else 0.0)
        self.publish_tf()


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

    def publish_tf(self):
        if not self.get_parameter('publish_tf').value:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('base_frame').value

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = self._yaw_to_quaternion(self.theta)
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)


    def diag_driver_status(self, stat: DiagnosticStatusWrapper):
        stat.summary(stat.OK, 'OK')

        if not self.serial_ok:
            stat.summary(stat.ERROR, 'Serial connection lost')

        dt = (self.get_clock().now() - self.last_serial_rx_time).nanoseconds
        if dt > 2e9:
            stat.summary(stat.WARN, 'No serial data received')

        stat.add('Serial OK', str(self.serial_ok))
        stat.add('Last RX age (s)', f'{dt / 1e9:.2f}')

        return stat

    def diag_battery(self, stat: DiagnosticStatusWrapper):
        if self.battery_voltage is None:
            stat.summary(stat.WARN, 'Battery voltage unknown')
            stat.add('Voltage (V)', 'unknown')
            return stat

        if self.battery_voltage < 6.8:
            stat.summary(stat.ERROR, 'Battery critically low')
        elif self.battery_voltage < 7.2:
            stat.summary(stat.WARN, 'Battery low')
        else:
            stat.summary(stat.OK, 'Battery OK')

        stat.add('Voltage (V)', f'{self.battery_voltage:.2f}')
        return stat


    def request_voltage(self):
        if self.serial_ok:
            self.send_cmd('$read_vol#')

def main():
    rclpy.init()
    node = MSPM0MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
