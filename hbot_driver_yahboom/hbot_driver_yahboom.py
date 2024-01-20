import rclpy
from rclpy.node import Node
from Rosmaster_Lib import Rosmaster
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
import tf2_ros
import tf2_py as tf2

from math import sin, cos, pi
import time

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
    self.declare_parameter(name='port', value="/dev/myserial",
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Serial port to connect to the robot"))
    self.declare_parameter(name='debug', value=False,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Enable debug mode"))
    self.declare_parameter(name='timeout', value=0.002,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Timeout for serial communication"))
    self.declare_parameter(name='wheel_base', value=0.2,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Distance between 2 wheels"))
    self.declare_parameter(name='wheel_diameter', value=0.05,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Wheel diameter"))
    self.declare_parameter(name='odom_frequency', value=10,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Frequency to publish odom data"))
    self.declare_parameter(name='gear_ratio', value=56,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Gear ratio"))
    self.declare_parameter(name='encoder_resolution', value=11,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Encoder resolution"))
    self.declare_parameter(name='max_rpm', value=100,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Max RPM"))
    self.declare_parameter(name='publish_odom_tf', value=False,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Publish odom TF"))
    self.declare_parameter(name='publish_imu', value=False,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description="Publish IMU data"))
    self.declare_parameter(name='imu_frequency', value=200,
                           descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Frequency to publish IMU data"))

    # Get parameters
    _port = self.get_parameter('port').get_parameter_value().string_value
    _debug = self.get_parameter('debug').get_parameter_value().bool_value
    _timeout = self.get_parameter('timeout').get_parameter_value().double_value
    self.__wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
    self.__wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
    self.__gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().integer_value
    self.__max_rpm = self.get_parameter('max_rpm').get_parameter_value().integer_value
    self.__encoder_resolution = self.get_parameter('encoder_resolution').get_parameter_value().integer_value
    self.__odome_freq = self.get_parameter('odom_frequency').get_parameter_value().integer_value
    self.__is_publish_odom_tf = self.get_parameter('publish_odom_tf').get_parameter_value().bool_value
    self.__is_publish_imu = self.get_parameter('publish_imu').get_parameter_value().bool_value
    self.__imu_freq = self.get_parameter('imu_frequency').get_parameter_value().integer_value

    self.get_logger().info(f"Parameters: wheel_base: {self.__wheel_base}, wheel_diameter: {self.__wheel_diameter}, gear_ratio: {self.__gear_ratio}, encoder_resolution: {self.__encoder_resolution}, max_rpm: {self.__max_rpm}, odom_frequency: {self.__odome_freq}, publish_odom_tf: {self.__is_publish_odom_tf}, publish_imu: {self.__is_publish_imu}, imu_frequency: {self.__imu_freq}")

    # Create Rosmaster object
    self.get_logger().info(f"Connect to driver at {_port} with debug: {_debug}")
    self.bot = Rosmaster(1, _port, _timeout, _debug)
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
    self.__x = 0.0
    self.__y = 0.0
    self.__theta = 0.0

    # Beep 3 times to indicate that the robot is ready
    for i in range(3):
      self.bot.set_beep(100)
      time.sleep(0.2)

    # Idicate that the robot is ready by turning on the LED
    self.bot.set_colorful_effect(1, 5)
    self.get_logger().info("Init driver successful! - Yahboom controller ***************")

  def vel_to_pwm(self, vel):
    # convert linear vel (m/s) to rpm
    rpm = vel * 60 / (3.14 * self.__wheel_diameter)
    rpm = min(rpm, self.__max_rpm)
    rpm = max(rpm, -self.__max_rpm)

    # convert rpm to pwm
    pwm =( rpm / self.__max_rpm) * 100
    return int(pwm)

  def cmd_vel_callback(self, msg):
    # self.get_logger().info(f"Received cmd_vel: {msg}")
    # Convert Twist message to vel_left and vel_right
    vel_linear = msg.linear.x
    vel_angular = msg.angular.z

    vel_left = vel_linear - vel_angular * self.__wheel_base / 2
    vel_right = vel_linear + vel_angular * self.__wheel_base / 2

    pwd_left = self.vel_to_pwm(vel_left)
    pwd_right = - self.vel_to_pwm(vel_right)

    self.bot.set_motor(pwd_left, pwd_right, 0, 0)
    self.get_logger().info(f"Set motor: {vel_left}, {vel_right} | {vel_left * 60 / (3.14 * self.__wheel_diameter)} {vel_right * 60 / (3.14*self.__wheel_diameter) } | {pwd_left}, {pwd_right}")

  def odom_timer_callback(self):
    dt = 1.0 / self.__odome_freq
    # Get encoder data
    encoder_left, encoder_right, _, _ = self.bot.get_motor_encoder()
    encoder_left = - encoder_left
    encoder_right = encoder_right

    delta_enc_left = encoder_left - self.__last_encoder_left
    delta_enc_right = encoder_right - self.__last_encoder_right
    if (abs(delta_enc_left) > self.__ENCODER_CIRCLE):
      delta_enc_left = (delta_enc_left - MAX_ENCODER_VALUE * 2) if (delta_enc_left > 0) else (delta_enc_left + MAX_ENCODER_VALUE * 2)
    if (abs(delta_enc_right) > self.__ENCODER_CIRCLE):
      delta_enc_right = (delta_enc_right - MAX_ENCODER_VALUE * 2) if (delta_enc_right > 0) else (delta_enc_right + MAX_ENCODER_VALUE * 2)

    self.__last_encoder_left = encoder_left
    self.__last_encoder_right = encoder_right

    # Calculate odometry
    delta_s_left = delta_enc_left * self.__wheel_diameter * pi /( self.__ENCODER_CIRCLE)
    delta_s_right = delta_enc_right * self.__wheel_diameter * pi /( self.__ENCODER_CIRCLE)

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
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.angular.z = delta_theta / dt
    odom_msg.pose.pose.position.x = self.__x
    odom_msg.pose.pose.position.y = self.__y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
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
      odom_tf_msg.transform.translation.z = 0.0
      odom_tf_msg.transform.rotation.x = 0.0
      odom_tf_msg.transform.rotation.y = 0.0
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
