#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill

def kill_turtle(name):
    rospy.wait_for_service('kill')
    try:
        kill_turtle = rospy.ServiceProxy('kill', Kill)
        kill_turtle(name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(x, y, theta, name)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('turtle_spawner', anonymous=True)

    # Kill the first turtle
    kill_turtle('turtle1')

    # Coordinates to form the letter 'k'
    spawn_turtle(4, 3, 0, 'k1')
    spawn_turtle(8, 3, 0, 'k2')
    spawn_turtle(4, 4, 0, 'k3')
    spawn_turtle(7, 4, 0, 'k4')
    spawn_turtle(4, 5, 0, 'k5')
    spawn_turtle(6, 5, 0, 'k6')
    spawn_turtle(4, 6, 0, 'k7')
    spawn_turtle(5, 6, 0, 'k8')
    spawn_turtle(4, 7, 0, 'k9')
    spawn_turtle(6, 7, 0, 'k10')
    spawn_turtle(4, 8, 0, 'k11')
    spawn_turtle(7, 8, 0, 'k12')
    spawn_turtle(4, 9, 0, 'k13')
    spawn_turtle(8, 9, 0, 'k14')

    # Coordinates to form the character '^'
    #spawn_turtle(7, 6, 0, 'c1')
    #spawn_turtle(6, 7, 0, 'c2')
    #spawn_turtle(5, 6, 0, 'c3')
    #spawn_turtle(8, 5, 0, 'c4')
    #spawn_turtle(4, 5, 0, 'c5')
    #spawn_turtle(9, 4, 0, 'c6')
    #spawn_turtle(3, 4, 0, 'c7')

    rospy.loginfo("Turtles spawned.")

if __name__ == '__main__':
    try:
        print("Everything's ok!")
        main()
    except:
        print("There is an error.")
        pass