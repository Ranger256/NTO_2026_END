#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_y_cyclic(speed=1.0, distance=4.0):
    rospy.init_node('move_test')
    pub = rospy.Publisher('/Human/cmd_vel', Twist, queue_size=1)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    cmd = Twist()
    
    while not rospy.is_shutdown():
        # Вперёд по Y (0 → 4)
        cmd.linear.y = speed
        for _ in range(int(distance / speed * 10)):
            pub.publish(cmd)
            rate.sleep()
        
        # Назад по Y (4 → 0)
        cmd.linear.y = -speed
        for _ in range(int(distance / speed * 10)):
            pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    move_y_cyclic(speed=0.2, distance=4.0)