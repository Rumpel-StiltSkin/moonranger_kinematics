# ============================================================================
#
#       Filename:  two_wheel.py
#
#    Description:  Contains a simple differential drive model
#
#        Version:  1.0
#        Created:  09/11/2021
#       Revision:  none
#
#         Author:  Ben Kolligs
#          Email:  bkolligs@andrew.cmu.edu
#   Organization:  Planetary Robotics Lab
#
# ============================================================================
import numpy as np

class TwoWheel:
    def __init__(self, w, l, h, r):
        '''
        This initializes the two wheeled differential drive model for moonranger
        '''
        # width of the vehicle from the center frame (total vehicle is 2w)
        self.width         = w
        # length of vehicle from center frame (total vehicle is 2l)
        self.length        = l
        self.height        = h
        self.wheel_radius  = r

        self.wheel_jacobian = 0.5*np.array(
            [
                [1   , 1     ] ,
                [1/w , 1/w   ]
            ]
        )
        self.body_jacobian = 1/r * np.array(
            [
                [1, -w/2 ],
                [1, w/2  ]
            ]
        )

    def actuation(self, body_velocity):
        '''
        Perform the actuation kinematics: body -> joint
        Output is radians per second
        '''
        if not isinstance(body_velocity, np.ndarray):
            body_velocity = np.array(body_velocity)
            body_velocity = body_velocity[3:5].reshape((-1, 1))
        assert body_velocity.shape == (2, 1)

        vel = np.matmul(self.body_jacobian, body_velocity)
        
        return np.array([vel[0], vel[1], vel[0], vel[1]]).reshape(4,1)  # [FL, FR, RL, RR]

    def navigation(self, wheel_velocity):
        '''
        Perform the navigation kinematics: joint -> body.
        Output is in meters per second
        '''
        if not isinstance(wheel_velocity, np.ndarray):
            wheel_velocity = np.array(wheel_velocity)
            wheel_velocity = wheel_velocity.reshape((-1, 1))
        assert wheel_velocity.shape == (2, 1)

        return np.matmul(self.wheel_jacobian, wheel_velocity)
