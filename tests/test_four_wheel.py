import rospy, unittest, io
import numpy as np
from moonranger_kinematics.four_wheel import FourWheel

package = 'moonranger_kinematics'

class TestFourWheel(unittest.TestCase):
    def __init__(self, methodName: str) -> None:
        """Test Four Wheel kinematic model."""
        super().__init__(methodName=methodName)
        self.tester = FourWheel(0.12, 0.2235, 0.3498, 0.409)
        self.test_rover = FourWheel(0.1, 0.2, 0.1, 0.07)

    def test_jacobian(self):
        test = self.test_rover.jacobian
        compare = np.array(
            [
                [0         , -0.1700  , -0.1000  , 1.0000  , 0       , 0       , -0.0700  , 0        , 0        , 0       ]        ,
                [0.1700    , 0        , 0.2000   , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
                [0.1000    , -0.2000  , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]        ,
                [0         , -0.1700  , 0.1000   , 1.0000  , 0       , 0       , 0        , -0.0700  , 0        , 0       ]        ,
                [0.1700    , 0        , 0.2000   , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
                [-0.1000   , -0.2000  , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]        ,
                [0         , -0.1700  , -0.1000  , 1.0000  , 0       , 0       , 0        , 0        , -0.0700  , 0       ]        ,
                [0.1700    , 0        , -0.2000  , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
                [0.1000    , 0.2000   , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]        ,
                [0         , -0.1700  , 0.1000   , 1.0000  , 0       , 0       , 0        , 0        , 0        , -0.0700 ]        ,
                [0.1700    , 0        , -0.2000  , 0       , 1.0000  , 0       , 0        , 0        , 0        , 0       ]        ,
                [-0.1000   , 0.2000   , 0        , 0       , 0       , 1.0000  , 0        , 0        , 0        , 0       ]
            ]
        )
        self.assertIsNone(np.testing.assert_allclose(test, compare))

    def test_jacobian_size(self):
        test = self.tester.jacobian.shape
        compare = (12, 10)
        self.assertEqual(test, compare)
    
    def test_jacobian_body_size(self):
        test = self.tester.body_jacobian.shape
        compare = (12, 6)
        self.assertEqual(test, compare)
    
    def test_jacobian_wheel_size(self):
        test = self.tester.wheel_jacobian.shape
        compare = (12, 4)
        self.assertEqual(test, compare)

    def test_jacobian_joint_inv_size(self):
        test = self.tester.inv_wheel_jacobian.shape
        compare = (4, 12)
        self.assertEqual(test, compare)

    def test_jacobian_body_inv_size(self):
        test = self.tester.inv_body_jacobian.shape
        compare = (6, 12)
        self.assertEqual(test, compare)
    
    def test_wheel_jacobian_inv(self):
        test = np.round(np.linalg.pinv(self.tester.wheel_jacobian), 8)
        compare = self.tester.inv_wheel_jacobian
        self.assertIsNone(np.testing.assert_allclose(test, compare))

    def test_body_jacobian_inv(self):
        test = np.round(np.linalg.pinv(self.tester.body_jacobian), 8)
        compare = self.tester.inv_body_jacobian
        # self.assert_(np.testing.assert_allclose(test, compare))
        self.assertIsNone(np.testing.assert_allclose(test, compare))
    
    def test_actuation_kinematics(self):
        
        test = self.test_rover.actuation([0, 0, 0, 1, 0, 0])
        compare = np.array([
            1.0,
            1.0,
            1.0,
            1.0
        ])
        self.assertIsNone(np.testing.assert_allclose(test, compare))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, 'Tests for four wheeled model', TestFourWheel)