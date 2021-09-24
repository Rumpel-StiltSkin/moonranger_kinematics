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

from rasm.msg import RASM_DRIVE_ARC_MSG
from sensor_msgs.msg import Joy

class Kinematics:
    def __init__(self) -> None:
        # get the model to use and select it with the parameters
        self.model_to_use  = rospy.get_param("~kinematic_model", "four_wheel")
        self.radius        = rospy.get_param("~{0}/radius".format(self.model_to_use), 0.1)
        self.height        = rospy.get_param("~{0}/height".format(self.model_to_use), 0.1)
        self.length        = rospy.get_param("~{0}/length".format(self.model_to_use), 0.1)
        self.width         = rospy.get_param("~{0}/width".format(self.model_to_use), 0.1)
        self.vx            = rospy.get_param("~four_wheel/contact_constraints/vx", 0)
        self.vy            = rospy.get_param("~four_wheel/contact_constraints/vy", 0)
        # motor factor to actually send to motor
        self.motor_factor  = rospy.get_param("~motor_factor", 1.0)
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
        self.drive_cmd = Joy() # [0,FL,FR,RL,RR,0]
        self.drive_arc_msg = []
        self.wheel_rates = [0,0,0,0] # FL,FR,RL,RR

        # self.drive_arc_topic = rospy.get_param("drive_arc_topic", "/navigation/drive_arc_cmd")
        self.wheels_topic = rospy.get_param("wheels_topic", "/wheels")

        # subscribers
        # self.drive_arc_sub = rospy.Subscriber(self.drive_arc_topic, DriveArc, self.drive_arc_sub_callback, queue_size=1)
        self.wheels_sub = rospy.Subscriber(self.wheels_topic, WheelRates, self.wheels_sub_callback, queue_size=1)
        self.drive_arc_sub = rospy.Subscriber("/navigator/drive_arc", RASM_DRIVE_ARC_MSG, self.drive_arc_sub_callback, queue_size=1)

        # publishers
        self.wheel_speeds_pub = rospy.Publisher("/kinematics/wheels", WheelRates, queue_size=1)
        self.body_vel_pub = rospy.Publisher("/kinematics/body", BodyVel, queue_size=1)
        self.joy_pub = rospy.Publisher('joy', Joy, queue_size=1)

        # start the ros node
        while not rospy.is_shutdown():
            curr_time = rospy.get_rostime()

            if self.drive_arc_msg and curr_time.secs + self.drive_arc_msg.duration > rospy.get_rostime().secs:
                try:
                    self.wheel_rates_to_joy()
                    self.joy_pub.publish(self.drive_cmd)
                except e:
                    print(e)
            else:
                self.drive_cmd.axes = [0, 0, 0, 0, 0, 0]
                self.joy_pub.publish(self.drive_cmd)

            rospy.spin()

    def drive_arc_sub_callback(self, msg):
        '''
        Coverts drive arcs into ingestable forms for the kinematics models
        '''
        self.drive_arc_msg = msg

        psi_dot, x_dot = drive_arc_convert(
            msg.speed,
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

        # account for motor command here by converting to RPM
        self.wheel_rates = self.wheel_rates * self.motor_factor
        wheel_msg = WheelRates(self.wheel_rates)
        self.wheel_speeds_pub.publish(wheel_msg)

    def wheels_sub_callback(self, msg):
        '''
        Feeds the kinematic models body velocity so it 
        '''
        # convert the motor readings into wheel rates in rad/s
        wheel_rates = msg.rates / self.motor_factor
        self.body_vel = self.model.navigation(
            [
                wheel_rates[0],
                wheel_rates[1],
                wheel_rates[2],
                wheel_rates[3],
            ]
        )

        body_msg = BodyVel(self.body_vel)
        self.body_vel_pub.publish(body_msg)

    def wheel_rates_to_joy(self):
        ROVER_MAX_SPEED = 0.07 # m/s
        JOY_MAX_INPUT = 0.7

        if not self.wheel_rates:
            rospy.logwarn("Couldn't get Wheel Rates")
            return False

        v_front_left = self.wheel_rates[0] * self.radius # to get m/s
        v_front_right = self.wheel_rates[1] * self.radius
        v_rear_left = self.wheel_rates[2] * self.radius
        v_rear_right = self.wheel_rates[3] * self.radius

        joy_v_fl = self.wheelVeltoJoy(v_front_left, ROVER_MAX_SPEED, JOY_MAX_INPUT)
        joy_v_fr = self.wheelVeltoJoy(v_front_right, ROVER_MAX_SPEED, JOY_MAX_INPUT)
        joy_v_rl = self.wheelVeltoJoy(v_rear_left, ROVER_MAX_SPEED, JOY_MAX_INPUT)
        joy_v_rr = self.wheelVeltoJoy(v_rear_right, ROVER_MAX_SPEED, JOY_MAX_INPUT)

        # [0, FL, FR, RL, RR, 0]
        self.drive_cmd.axes = [0, joy_v_fl, joy_v_fr, joy_v_rl, joy_v_rr, 0]

    def wheelVeltoJoy(v, vel_range, joy_vel_range):
        # formula: [x_min, x_max] => [a, b]
        # [-0.07, 0.07] => [-0.7, 0.7]
        # new_val = (b-a) * ( (x-x_min)/(x_max-x_min) ) + a
        return (2*joy_vel_range) * ( (v + vel_range) / (2*vel_range) ) - joy_vel_range


if __name__ == "__main__":
    rospy.init_node("moonranger_kinematics_node")
    rospy.loginfo("(Kinematic Model) for Drive Arc Conversion - Initialising")

    try:
        kinematics = Kinematics()
    except rospy.ROSInitException:
        pass
