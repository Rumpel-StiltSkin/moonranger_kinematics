
import rospy, unittest, io
import numpy as np
from std_msgs.msg import Float32MultiArray
from moonranger_kinematics.four_wheel import FourWheel

class TestKinematics(unittest.TestCase):
	def test_drive_arc(self):
		pass

package = 'moonranger_kinematics'
if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, 'Tests the overall kinematic controller', TestKinematics)