#!/usr/bin/env python
#
# DESCRIPTION
# range2speed2 uses range sensor (messages) to calculate speed.
# when the robot is moving to a target (e.g a wall) the measuring begins a soon as the target 
# is within the range (1.2) of the range sensor
# the robot and the measuring stops when range is below 0.15m
#
# RUN
# rosrun simple_movement range2speed2.py
#
#

import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class RobotMovement:
    range = 0
    speed = 0
    maxspeed = 1000000
    minspeed = 0.137
    rotspeed = 2
    range_subscriber = None
    velocity_publisher = None
    vel_msg = None
    rate = None

    measuring = False

    last_time = None
    last_range = 0
    current_speed = 0.0

    def __init__(self):

        # init_arguments(self)

        self.rate = rospy.Rate(30)  # 10hz
        self.range_topic = "sonars"
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.range_subscriber = rospy.Subscriber(self.range_topic, Range, self.process_range)
        rospy.loginfo("wating for message " + self.range_topic)
        rospy.wait_for_message(self.range_topic, Range)
        rospy.loginfo("starting...")

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        self.last_time = rospy.Time.now()

    def process_range(self, range):
        # rospy.loginfo_throttle(1,rospy.get_caller_id() + "Range: %s", range.range)
        # rospy.loginfo(rospy.get_caller_id() + " Range: %s", range.range)
        self.range = range.range

    def measure_and_stop(self):

        # measure speed
        current_range = self.range
        current_time = rospy.Time.now()
        duration = current_time - self.last_time
        self.current_speed = (self.last_range - current_range) / duration.to_sec()

        # stopping
        self.vel_msg.linear.x = 0
        rospy.loginfo(rospy.get_caller_id() + " OBSTACLE: stopping SPEED: %s", self.current_speed)
        self.velocity_publisher.publish(self.vel_msg)

        # disale measuring
        self.measuring = False


    def move(self, speed):
        if speed > self.maxspeed:
            self.speed = self.maxspeed
        elif speed < self.minspeed:
            self.speed = 0
        else:
            self.speed = speed

        self.vel_msg.linear.x = self.speed

        while not rospy.is_shutdown():

            # Force the robot to stop
            if self.range < 0.15:
                self.measure_and_stop()
                return
            if self.range < 1.20 and not self.measuring:
                self.last_range = self.range
                self.last_time = rospy.Time.now()
                self.measuring = True

            # rospy.loginfo(rospy.get_caller_id() + " MOVE: velocity %s RANGE: %s CURRENT_SPEED: %s", self.vel_msg.linear.x, self.range, self.current_speed)

            self.velocity_publisher.publish(self.vel_msg)

            self.rate.sleep()

            # self.vel_msg.linear.x = self.speed


if __name__ == '__main__':

    # Receiveing the user's input
    # print("Let's move your robot")
    # speed = input("Input your speed:")
    # distance = input("Type your distance:")
    # isForward = input("Forward?: ")#True or False
    # isReturn = input("Return?:")#True or False
    # isEndless= input("Endless?:")#True or False

    # Checking if the movement is forward or backwards
    # if(isForward):
    #   vel_msg.linear.x = abs(speed)
    #    speed = abs(speed)
    # else:
    #   vel_msg.linear.x = -abs(speed)
    #    speed = -abs(speed)

    try:
        # Starts a new node
        rospy.init_node('simple_move_range_speed', anonymous=True)
        robot_movement = RobotMovement()

        # wait half a sec to get range data bevor moving
        # rospy.sleep(0.5)
        while not rospy.is_shutdown():
            print("Let's move your robot")
            speed = input("Input your speed:")
            speed = abs(speed)
            robot_movement.move(speed)

    except rospy.ROSInterruptException: pass
