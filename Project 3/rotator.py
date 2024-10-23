#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def rotate_turtles(turtle_names):
    publishers = {}
    for name in turtle_names:
        topic = f'/{name}/cmd_vel'
        publishers[name] = rospy.Publisher(topic, Twist, queue_size=10)

    twist = Twist()
    twist.angular.z = 1.0  

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        for pub in publishers.values():
            pub.publish(twist)
        rate.sleep()

def main():
    rospy.init_node('turtle_rotator', anonymous=True)

    turtle_names = [
        'k1', 'k2', 'k3', 'k4', 'k5', 'k6',
        'k7', 'k8', 'k9', 'k10', 'k11', 'k12',
        'k13', 'k14'
    ]

    # Rotate all turtles indefinitely
    rotate_turtles(turtle_names)

if __name__ == '__main__':
    try:
        print("Everything is ok!")
        main()
    except rospy.ROSInterruptException as e:
        print(f"There is an error: {e}")