#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class RobotMovement:
    range = 0
    speed = 0
    maxspeed = 2
    minspeed = 0
	range_subscriber = None
	velocity_publisher = None
	vel_msg = None
    rate = None


	def __init__(self):
		init_arguments(self)

        self.rate = rospy.Rate(10) # 10hz
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	    self.range_subscriber = rospy.Subscriber("range", Range, self.process_range)

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def process_range(self, range):
        rospy.loginfo_throttle(rospy.get_caller_id() + "Range: %s", range.range)
        self.range = range.range

    def stop_and_turn(self):
        self.vel_msg.linear.x = 0
        rospy.loginfo(rospy.get_caller_id() + " publish linear velocity %s", vel_msg.linear.x)
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(1)
        self.vel_msg.angular.z = 2
        rospy.loginfo(rospy.get_caller_id() + " publish angular velocity %s", vel_msg.angular.z)
        rospy.sleep(1)
        self.vel_msg.angular.z = 0

    def move(self,speed):
        self.speed = speed
        self.vel_msg.linear.x = self.speed

        while not rospy.is_shutdown():

            #Force the robot to stop
            if range < 0.1:
                self.stop_and_turn()
            elif range < 0.3:
                vel_msg.linear.x = self.speed * 0.75
            elif range < 0.5:
                vel_msg.linear.x = self.speed * 0.5

            rospy.loginfo(rospy.get_caller_id() + " publish velocity %s", vel_msg.linear.x)

            self.velocity_publisher.publish(self.vel_msg)

            self.rate.sleep()

            self.vel_msg.linear.x = self.speed


if __name__ == '__main__':

    #Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed:")
    # distance = input("Type your distance:")
    # isForward = input("Forward?: ")#True or False
    # isReturn = input("Return?:")#True or False
    # isEndless= input("Endless?:")#True or False

    #Checking if the movement is forward or backwards
    #if(isForward):
        #vel_msg.linear.x = abs(speed)
    #    speed = abs(speed)
    #else:
        #vel_msg.linear.x = -abs(speed)
    #    speed = -abs(speed)

    speed = abs(speed)

    # Starts a new node


    try:
        rospy.init_node('simple_move_range', anonymous=True)
        robot_movement = RobotMovement()
        #Testing our function
        robot_movement.move(speed)
    except rospy.ROSInterruptException: pass
