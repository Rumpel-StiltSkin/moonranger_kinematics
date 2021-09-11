import rospy, unittest, io
from moonranger_kinematics.two_wheel import TwoWheel

package = 'moonranger_kinematics'

class TestTwoWheel(unittest.TestCase):
    """Test Two Wheel kinematic model."""
    def test_one(self):
        self.assert_(True)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, 'Tests for two wheeled model', TestTwoWheel)