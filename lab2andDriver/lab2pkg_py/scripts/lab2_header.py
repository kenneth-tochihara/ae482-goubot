from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
PI = 3.1415926535897932384626433

"""
command msg
------------------------
float64[] destination
float64 v
float64 a
bool io_0


position msg
------------------------
float64[] position
bool isReady


gripper_input msg
------------------------
int32 DIGIN 
float64 AIN0
float64 AIN1


Twist msg
------------------------
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z


"""