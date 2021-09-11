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
        '''
        Verify jacobian declaration
        '''
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
        '''
        Test jacobian size
        '''
        test = self.tester.jacobian.shape
        compare = (12, 10)
        self.assertEqual(test, compare)
    
    # the following tests check sizes of various jacobians
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
        '''
        Verify the static inverse matches the calculated inverse
        '''
        test = np.round(np.linalg.pinv(self.tester.wheel_jacobian), 8)
        compare = self.tester.inv_wheel_jacobian
        self.assertIsNone(np.testing.assert_allclose(test, compare))

    def test_body_jacobian_inv(self):
        '''
        Verify the static inverse matches the calculated inverse
        '''
        test = np.round(np.linalg.pinv(self.tester.body_jacobian), 8)
        compare = self.tester.inv_body_jacobian
        # self.assert_(np.testing.assert_allclose(test, compare))
        self.assertIsNone(np.testing.assert_allclose(test, compare))
    
    def test_actuation_kinematics_straight(self):
        '''
        Drive in a straigt line
        '''
        test = np.round(self.test_rover.actuation([0, 0, 0, 1, 0, 0]), 4)
        compare = np.array([
            14.2857,
            14.2857,
            14.2857,
            14.2857,
        ]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))

    def test_actuation_kinematics_arc(self):
        '''
        Drive in an arc
        '''
        test = np.round(self.test_rover.actuation([0, 0, 0.1, 0.5, 0, 0]), 4)
        compare = np.array([
            7.0,
            7.2857,
            7.0,
            7.2857,
        ]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))
    
    def test_actuation_kinematics_point_turn(self):
        '''
        Drive in a point turn
        '''
        test = np.round(self.test_rover.actuation([0, 0, np.pi, 0.0, 0, 0]), 4)
        compare = np.array([
            -4.4880 ,
            4.4880  ,
            -4.4880 ,
            4.4880
        ]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))
    
    def test_navigation_forward(self):
        '''
        Driving forward, determine body velocity
        '''
        test = np.round(self.test_rover.navigation([0.1, 0.1, 0.1, 0.1]), 4)
        compare = np.array([
            0, 
            0, 
            0, 
            0.007,
            0, 
            0
        ]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))

    def test_navigation_arc(self):
        '''
        Driving forward, determine body velocity
        '''
        test = np.round(self.test_rover.navigation([0.1, 0.3, 0.1, 0.3]), 4)
        compare = np.array([
            0, 
            0, 
            0.014,
            0.014,
            0, 
            0
        ]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))

    def test_navigation_disparate_wheels(self):
        '''
        Say the wheels are moving at different velocities, and a couple are moving slower than desired
        '''
        test = np.round(self.test_rover.navigation([0.15, 0.33, 0.08, 0.25]), 4)
        compare = np.array([
            0, 
            0, 
            0.0122,
            0.0142,
            0, 
            0
        ]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, 'Tests for four wheeled model', TestFourWheel)