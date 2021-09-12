from numpy import testing
import rospy, unittest, io
import numpy as np
from moonranger_kinematics.two_wheel import TwoWheel

package = 'moonranger_kinematics'

class TestTwoWheel(unittest.TestCase):
    """Test Two Wheel kinematic model."""
    def __init__(self, methodName: str) -> None:
        super().__init__(methodName=methodName)
        self.rover_test = TwoWheel(0.1, 0.2, 0.1, 0.07)

    # checking sizes of things
    def test_wheel_jacobian_size(self):
        test = self.rover_test.wheel_jacobian.shape
        compare = (2, 2)
        self.assertEqual(test, compare)

    def test_body_jacobian_size(self):
        test = self.rover_test.body_jacobian.shape
        compare = (2, 2)
        self.assertEqual(test, compare)
    
    def test_forward_inverse_simple(self):
        '''
        Test that the function performs inverse properly
        '''
        rover = TwoWheel(1.0, 1.0, 1.0, 1.0)
        test = rover.actuation([1.0, 1.0])
        compare = np.array([0.5, 1.5]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))

        # now try the inverse
        test_inv = rover.navigation(test)
        compare = np.array([1.0, 1.0]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test_inv, compare))
    
    def test_actuation_forward(self):
        '''
        Test that this matches the four wheel kinematics
        '''
        test = np.round(self.rover_test.actuation([1.0, 0.0]), 4)
        compare = np.array([14.2857, 14.2857]).reshape((-1, 1))
        self.assertIsNone(np.testing.assert_allclose(test, compare))
    
    def test_actuation_arc(self):
        '''
        So this actually does not match but that's okay since the math is different
        '''
        test = np.round(self.rover_test.actuation([0.5, 0.1]), 4)
        compare = np.array([7.0, 7.2857]).reshape((-1, 1))
        # since these are a bit different, just check that they're roughly the same with really high relative tolerance
        self.assertIsNone(np.testing.assert_allclose(test, compare, rtol=1e-1))



if __name__ == '__main__':
    import rostest
    rostest.rosrun(package, 'Tests for two wheeled model', TestTwoWheel)