import rospy, unittest, io
from kinematic_models.four_wheel import FourWheel

package = 'moonranger_kinematics'

class TestTwoWheel(unittest.TestCase):
    """Test Two Wheel kinematic model."""
    def test_one(self):
        self.assert_(True)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, 'Tests for two wheeled model', TestTwoWheel)