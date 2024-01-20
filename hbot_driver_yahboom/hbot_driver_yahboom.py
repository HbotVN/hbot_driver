import rclpy
from rclpy.node import Node
from Rosmaster_Lib import Rosmaster
from geometry_msgs.msg import Twist

# 1. Control the robot with cmd_vel
# 2. Publish the odometry from the encoders
# 3. Publish the battery voltage
# 4. Publish the IMU data
# 5. Publish the TF data

class MyNode(Node):
  def __init__(self):
    super().__init__('hbot_driver_yahboom')

    # Declare parameters
    self.declare_parameter('port', '/dev/myserial')
    self.declare_parameter('baudrate', 115200)
    self.declare_parameter('timeout', 0.002)
    self.declare_parameter('wheel_base', 0.2)
    self.declare_parameter('wheel_radius', 0.05)


    # Get parameters
    _port = self.get_parameter('port').value
    _timeout = self.get_parameter('timeout').value
    self.__wheel_base = self.get_parameter('wheel_base').value
    self.__wheel_radius = self.get_parameter('wheel_radius').value

    self.bot = Rosmaster(1, _port, _timeout, True)
    self.bot.create_receive_threading()

    # Create timer
    self.create_timer(0.1, self.timer_callback)

    # Create subscribers
    self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
    # create publisher
    self.


    self.get_logger(f"Controller firmware version: {self.bot.get_version()}")

  def cmd_vel_callback(self, msg):
    # Convert Twist message to vel_left and vel_right
    vel_linear = msg.linear.x
    vel_angular = msg.angular.z

    vel_left = (2 * vel_linear - vel_angular * self.__wheel_base) / 2
    vel_right = (2 * vel_linear + vel_angular * self.__wheel_base) / 2

    self.bot.set_motor(vel_left, vel_right, 0, 0)

def main(args=None):
  rclpy.init(args=args)
  node = MyNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
