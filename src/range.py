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
	movement_command = ""
	state_machine_publisher = None
	fine_movement = False
	odometry_subscriber = None
	last_orientation = None
	update_reference_odometry = False
	searching_for_ball = False
	prepared_to_stop = False
	angular_offset = 0.75

	ball_at_bottom_of_frame = False

	def __init__(self):
		init_arguments(self)
    self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.range_subscriber = rospy.Subscriber("range", Range, self.process_range)
		
		self.movement_message = Twist()

  def process_range(self, range):
    rospy.loginfo(rospy.get_caller_id() + "Range: %s", range.range)
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

g_range = 0
speed = 0
g_maxspeed = 2
g_minspeed = 0

def stop_and_turn():

    vel_msg.linear.x = 0
    rospy.loginfo(rospy.get_caller_id() + " publish linear velocity %s", vel_msg.linear.x)
    velocity_publisher.publish(vel_msg)
    rospy.sleep(1)
    vel_msg.angular.z = 2
    rospy.loginfo(rospy.get_caller_id() + " publish angular velocity %s", vel_msg.angular.z)
    rospy.sleep(1)
    vel_msg.angular.z = 0

    
def range_callback(range):
    global g_range
    rospy.loginfo(rospy.get_caller_id() + "Range: %s", range.range)
    g_range = range.range

def move():
    global speed    
    # Starts a new node
    rospy.init_node('simple_move_range', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("range", Range, range_callback)
    
    vel_msg = Twist()
    rate = rospy.Rate(10) # 10hz

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

    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    vel_msg.linear.x = speed

    while not rospy.is_shutdown():
                               
        #Force the robot to stop
        if range < 0.1:
          stop_and_turn()
        elif range < 0.3:
          vel_msg.linear.x = speed * 0.75
        elif range < 0.5:
          vel_msg.linear.x = speed * 0.5
        
        rospy.loginfo(rospy.get_caller_id() + " publish velocity %s", vel_msg.linear.x)
        
        velocity_publisher.publish(vel_msg)

        rate.sleep()

        vel_msg.linear.x = speed
        

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
