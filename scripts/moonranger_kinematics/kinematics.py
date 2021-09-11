#!/usr/bin/python3
# ============================================================================
#
#       Filename:  kinematics.py
#
#    Description:  The kinematics node for performing actuation (body->joints) and navigation (joints-> body) velocities. Contains ability to use different models for verification and comparison. 
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
import rospy
from std_msgs.msg import Float32
from moonranger_kinematics.two_wheel import TwoWheel
from moonranger_kinematics.four_wheel import FourWheel
from moonranger_kinematics.utils import drive_arc_convert
from moonranger_kinematics.msg import DriveArc, WheelRates, BodyVel

class Kinematics:
    def __init__(self) -> None:
        # get the model to use and select it with the parameters
        self.model_to_use  = rospy.get_param("/kinematic_model", "four_wheel")
        self.radius        = rospy.get_param("{0}/radius".format(self.model_to_use), 0.1)
        self.height        = rospy.get_param("{0}/height".format(self.model_to_use), 0.1)
        self.length        = rospy.get_param("{0}/length".format(self.model_to_use), 0.1)
        self.width         = rospy.get_param("{0}/width".format(self.model_to_use), 0.1)
        self.vx            = rospy.get_param("four_wheel/contact_constraints/vx", 0)
        self.vy            = rospy.get_param("four_wheel/contact_constraints/vy", 0)
        # define the models
        self.models = {
            'two_wheel': TwoWheel(),
            'four_wheel' : FourWheel(
                self.width,
                self.length,
                self.height,
                self.radius,
                self.vx, 
                self.vy
            )
        }
        self.model = self.models[self.model_to_use]

        self.drive_arc_topic = rospy.get_param("drive_arc_topic", "/navigation/drive_arc_cmd")
        self.wheels_topic = rospy.get_param("wheels_topic", "/body_vel")

        # subscribers
        self.drive_arc_sub = rospy.Subscriber(self.drive_arc_topic, DriveArc, self.drive_arc_sub_callback, queue_size=1)
        self.body_vel_sub = rospy.Subscriber(self.wheels_topic, Float32, self.wheels_sub_callback, queue_size=1)

        # publishers
        self.wheel_speeds_pub = rospy.Publisher("/kinematics/wheels", Float32, queue_size=1)
        self.body_vel_pub = rospy.Publisher("/kinematics/body", Float32, queue_size=1)

        # start the ros node
        while not rospy.is_shutdown():
            # spin to wait for messages
            rospy.spin()

    def drive_arc_sub_callback(self, msg):
        '''
        Coverts drive arcs into ingestable forms for the kinematics models
        '''
        psi_dot, x_dot = drive_arc_convert(
            msg.velocity,
            msg.radius,
            msg.time
        )
        self.wheel_rates = self.model.actuation(
            [
                0, 
                0, 
                psi_dot, 
                x_dot, 
                0, 
                0
            ]
        )

        wheel_msg = Float32(self.wheel_rates)
        self.wheel_speeds_pub.publish(wheel_msg)

    def wheels_sub_callback(self, msg):
        '''
        Feeds the kinematic models body velocity so it 
        '''
        wheel_rates = msg.data
        self.body_vel = self.model.navigation(
            [
                wheel_rates[0],
                wheel_rates[1],
                wheel_rates[2],
                wheel_rates[3],
            ]
        )

        body_msg = Float32(self.body_vel)
        self.body_vel_pub.publish(body_msg)

if __name__ == "__main__":
    rospy.init_node("moonranger_kinematics_node")
    try:
        kinematics = Kinematics()
    except rospy.ROSInitException:
        pass