#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class RobotMovement:
    range = 0
    speed = 0
    #maxspeed = 0.364
    maxspeed = 0.564
    #minspeed = 0.137
    minspeed = 0.237
    rotspeed = 2
    range_subscriber = None
    velocity_publisher = None
    vel_msg = None
    rate = None

    def __init__(self):

        # init_arguments(self)

        self.rate = rospy.Rate(10)  # 10hz
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #self.range_subscriber = rospy.Subscriber("range", Range, self.process_range)
        self.range_subscriber = rospy.Subscriber("sonars", Range, self.process_range)

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def process_range(self, range):
        # rospy.loginfo_throttle(1,rospy.get_caller_id() + "Range: %s", range.range)
        # rospy.loginfo(rospy.get_caller_id() + " Range: %s", range.range)
        self.range = range.range

    def stop_and_turn(self):
        self.vel_msg.linear.x = 0
        rospy.loginfo(rospy.get_caller_id() + " OBSTACLE: stopping")
        self.velocity_publisher.publish(self.vel_msg)
        rospy.sleep(1)
        self.vel_msg.angular.z = self.rotspeed * (-1)**random.randrange(2)
        rospy.loginfo(rospy.get_caller_id() + " OBSTACLE: turning speed %s", self.vel_msg.angular.z)
        self.velocity_publisher.publish(self.vel_msg)
        rospy.sleep(1)
        self.vel_msg.angular.z = 0

    def move(self,speed):
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
                self.stop_and_turn()
            elif self.range < 0.7:
                newspeed = self.speed * self.range * 1.4  # just a factor
                if newspeed < self.minspeed:
                    self.vel_msg.linear.x = self.minspeed
                else:
                    self.vel_msg.linear.x = newspeed
            # elif self.range < 0.5:
            #     newspeed = self.speed * 0.5
            #     if newspeed < self.minspeed:
            #         self.vel_msg.linear.x = self.minspeed
            #     else:
            #         self.vel_msg.linear.x = newspeed
            # elif self.range < 0.7:
            #     newspeed = self.speed * 0.7
            #     if newspeed < self.minspeed:
            #         self.vel_msg.linear.x = self.minspeed
            #     else:
            #         self.vel_msg.linear.x = newspeed

            rospy.loginfo(rospy.get_caller_id() + " MOVE: velocity %s", self.vel_msg.linear.x)

            self.velocity_publisher.publish(self.vel_msg)

            self.rate.sleep()

            self.vel_msg.linear.x = self.speed


if __name__ == '__main__':

    # Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed:")
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

    speed = abs(speed)

    try:
        # Starts a new node
        rospy.init_node('simple_move_range', anonymous=True)
        robot_movement = RobotMovement()

        # wait half a sec to get range data bevor moving
        rospy.sleep(0.5)
        # Testing our function
        robot_movement.move(speed)
    except rospy.ROSInterruptException: pass
