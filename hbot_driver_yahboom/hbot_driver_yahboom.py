import rclpy
from rclpy.node import Node
from Rosmaster_Lib import Rosmaster
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import tf2_ros
import tf2_py as tf2

from math import pi, sin, cos, tan, atan2, sqrt

# 1. Control the robot with cmd_vel
# 2. Publish the odometry from the encoders
# 3. Publish the battery voltage
# 4. Publish the IMU data
# 5. Publish the TF data

MAX_ENCODER_VALUE = 2147483647
MIN_ENCODER_VALUE = -2147483648

MIN_BATTERY_VOLTAGE = 7.5 # (2.5V per cell*3)
MAX_BATTERY_VOLTAGE = 12.6 # (4.2V per cell*3)

class MyNode(Node):
  def __init__(self):
    super().__init__('hbot_driver_yahboom_node')

    # Declare parameters
    self.declare_parameter('port', '/dev/myserial')
    self.declare_parameter('baudrate', 115200)
    self.declare_parameter('timeout', 0.002)
    self.declare_parameter('wheel_base', 0.2)
    self.declare_parameter('wheel_radius', 0.05)
    self.declare_parameter('odom_frequency', 10)
    self.declare_parameter('gear_ratio', 56)
    self.declare_parameter('encoder_resolution', 11)
    self.declare_parameter('publish_odom_tf', False)
    self.declare_parameter('publish_imu', False)
    self.declare_parameter('imu_frequency', 200)

    # Get parameters
    _port = self.get_parameter('port').value
    _timeout = self.get_parameter('timeout').value
    self.__wheel_base = self.get_parameter('wheel_base').value
    self.__wheel_radius = self.get_parameter('wheel_radius').value
    self.__odome_freq = self.get_parameter('odom_frequency').value
    self.__gear_ratio = self.get_parameter('gear_ratio').value
    self.__encoder_resolution = self.get_parameter('encoder_resolution').value
    self.__is_publish_odom_tf = self.get_parameter('publish_odom_tf').value
    self.__is_publish_imu = self.get_parameter('publish_imu').value
    self.__imu_freq = self.get_parameter('imu_frequency').value

    self.bot = Rosmaster(1, _port, _timeout, True)
    self.bot.create_receive_threading()

    # Create timer
    self.create_timer(1.0/self.__odome_freq, self.odom_timer_callback)
    if self.__is_publish_imu:
      self.create_timer(1.0/self.__imu_freq, self.imu_timer_callback)

    # Create subscribers
    self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    # create publisher
    self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
    self.batery_vol_pub = self.create_publisher(Float32, 'battery/voltage', 10)
    self.batery_percent_pub = self.create_publisher(Float32, 'battery/percent', 10)
    if self.__is_publish_odom_tf:
      self.odom_tf_pub = self.create_publisher(TransformStamped, 'odom_tf', 10)
    if self.__is_publish_imu:
      self.imu_pub = self.create_publisher(Imu, 'imu', 10)

    self.__ENCODER_CIRCLE = self.__gear_ratio * self.__encoder_resolution * 4

    # Print firmware version
    self.get_logger().info(f"Controller firmware version: {self.bot.get_version()}")
    self.get_logger().info(f"Controller battery voltage: {self.bot.get_battery_voltage()}")
    self.bot.set_motor(0, 0, 0, 0) # Stop the robot
    self.__last_encoder_left, self.__last_encoder_right, _, _ = self.bot.get_motor_encoder()
    # Beep 3 times to indicate that the robot is ready
    for i in range(3):
      self.bot.set_beep(100)
      self.sleep(0.1)
    # Idicate that the robot is ready by turning on the LED
    self.bot.set_colorful_effect(1, 5)
    self.get_logger().info("Init driver successful! - Yahboom controller ***************")

  def cmd_vel_callback(self, msg):
    # Convert Twist message to vel_left and vel_right
    vel_linear = msg.linear.x
    vel_angular = msg.angular.z

    vel_left = (2 * vel_linear - vel_angular * self.__wheel_base) / 2
    vel_right = (2 * vel_linear + vel_angular * self.__wheel_base) / 2

    self.bot.set_motor(vel_left, vel_right, 0, 0)

  def odom_timer_callback(self):
    dt = 1.0 / self.__odome_freq
    # Get encoder data
    encoder_left, encoder_right, _, _ = self.bot.get_motor_encoder()

    delta_enc_left = encoder_left - self.__last_encoder_left
    delta_enc_right = encoder_right - self.__last_encoder_right
    if (abs(delta_enc_left) > self.__ENCODER_CIRCLE):
      delta_enc_left = (delta_enc_left - MAX_ENCODER_VALUE * 2) if (delta_enc_left > 0) else (delta_enc_left + MAX_ENCODER_VALUE * 2)
    if (abs(delta_enc_right) > self.__ENCODER_CIRCLE):
      delta_enc_right = (delta_enc_right - MAX_ENCODER_VALUE * 2) if (delta_enc_right > 0) else (delta_enc_right + MAX_ENCODER_VALUE * 2)

    self.__last_encoder_left = encoder_left
    self.__last_encoder_right = encoder_right

    # Calculate odometry
    delta_s_left = delta_enc_left * self.__wheel_radius / self.__ENCODER_CIRCLE
    delta_s_right = delta_enc_right * self.__wheel_radius / self.__ENCODER_CIRCLE

    delta_s = (delta_s_left + delta_s_right) / 2
    delta_theta = (delta_s_right - delta_s_left) / self.__wheel_base

    delta_x = delta_s * cos(delta_theta)
    delta_y = delta_s * sin(delta_theta)

    self.__x += delta_x
    self.__y += delta_y
    self.__theta += delta_theta

    # Publish odometry
    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.header.stamp = self.get_clock().now().to_msg()
    odom_msg.twist.twist.linear.x = delta_x / dt
    odom_msg.twist.twist.linear.y = 0
    odom_msg.twist.twist.angular.z = delta_theta / dt
    odom_msg.pose.pose.position.x = self.__x
    odom_msg.pose.pose.position.y = self.__y
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation.x = 0
    odom_msg.pose.pose.orientation.y = 0
    odom_msg.pose.pose.orientation.z = sin(self.__theta / 2)
    odom_msg.pose.pose.orientation.w = cos(self.__theta / 2)

    self.odom_pub.publish(odom_msg)

    # Publish TF
    if self.__is_publish_odom_tf:
      odom_tf_msg = TransformStamped()
      odom_tf_msg.header.stamp = self.get_clock().now().to_msg()
      odom_tf_msg.header.frame_id = "odom"
      odom_tf_msg.child_frame_id = "base_link"
      odom_tf_msg.transform.translation.x = self.__x
      odom_tf_msg.transform.translation.y = self.__y
      odom_tf_msg.transform.translation.z = 0
      odom_tf_msg.transform.rotation.x = 0
      odom_tf_msg.transform.rotation.y = 0
      odom_tf_msg.transform.rotation.z = sin(self.__theta / 2)
      odom_tf_msg.transform.rotation.w = cos(self.__theta / 2)

      self.odom_tf_pub.publish(odom_tf_msg)

    # Publish battery voltage
    battery_vol_msg = Float32()
    battery_vol_msg.data = self.bot.get_battery_voltage()
    if (self.batery_vol_pub.get_subscription_count() > 0):
      self.batery_vol_pub.publish(battery_vol_msg)

    # Publish battery percent
    battery_percent_msg = Float32()
    battery_percent_msg.data = (battery_vol_msg.data - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100
    if (self.batery_percent_pub.get_subscription_count() > 0):
      self.batery_percent_pub.publish(battery_percent_msg)

    # Publish motor current

  def imu_timer_callback(self):
    roll, pitch, yaw = self.bot.get_imu_attitude_data()
    # Convert roll, pitch, and yaw to quaternion
    quaternion = tf2.transformations.quaternion_from_euler(roll, pitch, yaw)

    imu_msg = Imu()
    imu_msg.header.stamp = self.get_clock().now().to_msg()
    imu_msg.header.frame_id = "imu_link"
    imu_msg.orientation_covariance[0] = -1
    imu_msg.angular_velocity_covariance[0] = -1
    imu_msg.linear_acceleration_covariance[0] = -1

    imu_msg.orientation.x = quaternion[0]
    imu_msg.orientation.y = quaternion[1]
    imu_msg.orientation.z = quaternion[2]
    imu_msg.orientation.w = quaternion[3]

    gx, gy, gz = self.bot.get_gyroscope_data()
    imu_msg.angular_velocity.x = gx
    imu_msg.angular_velocity.y = gy
    imu_msg.angular_velocity.z = gz

    ax, ay, az = self.bot.get_accelerometer_data()
    imu_msg.linear_acceleration.x = ax
    imu_msg.linear_acceleration.y = ay
    imu_msg.linear_acceleration.z = az

    if (self.imu_pub.get_subscription_count() > 0):
      self.imu_pub.publish(imu_msg)

def main(args=None):
  rclpy.init(args=args)
  node = MyNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
