#!/usr/bin/env python

# NOTE: joints = ['FIRST_1','FIRST_2','FIRST_3','SECOND_1','THUMB_1','THUMB_2']
# THUMB_1 is a thumb_up joint 
# THUMB_1 +Theta is toward the palm

# i_move and i_moveAll receive the diff not the abosolute theta value

import roslib; roslib.load_manifest('hiro_control')
import rospy

from hiro_control.srv import *

import math
import sys

sys.path.append( rospy.get_param('/hiro_hand_driver_path', '/home/vektor/4/ros-pkg/hiro_control/driver/hiro_hand') )
#rospy.loginfo('%s', sys.path)

import hand

r = 0
l = 1

def handle_control_hand_srv(req):
    if req.cmd == 1:
        grasp()
    elif req.cmd == 2:
        ungrasp() 
    
    return ControlHandResponse("(Un)Grasp: Succeeded")
    
def grasp():
    # Open the hand: joints = ['FIRST_1','FIRST_2','FIRST_3','SECOND_1','THUMB_1'=-69.99916992187501,'THUMB_2'=86.472509765625]
    open_hand_pos_diff_from_initial = [0.0, 0.0, 0.0, 0.0, -70., 85.]

    rospy.loginfo('%s', 'Open the hand ...')
    hand.i_moveAll(r,open_hand_pos_diff_from_initial)
    rospy.sleep(10.)# TODO wait appropriately ....    
    
    # TODO use the pressure value
    
    # GRASP
    # Pressure values:
    # before grasp: [-11.0, -17.0, 18.0, 0.0, 18.0, 12.0
    # after grasp: [12.0, -12.0, 24.0, 21.0, 44.0, 12.0]    
    # POS for the Georgia small can: [0.0, 0.0, 0.0, 39.999462890625004, -29.995751953125, 86.46767578125001]
    georgia_can_grasp_pos_diff_from_open_hand = [0.0, 0.0, 0.0, 37., -29.995751953125-(-69.99916992187501), 0]
    
    rospy.loginfo('%s', 'Grasp ...')
    hand.i_moveAll(r,georgia_can_grasp_pos_diff_from_open_hand)    
    rospy.sleep(10.)# TODO wait appropriately ....    
    
def ungrasp():
    # TODO should check the current pos
    open_hand_pos_diff_from_grasp = [0.0, 0.0, 0.0, -37., -69.99916992187501-(-29.995751953125), 0]
    
    rospy.loginfo('%s', 'Open the hand ...')  
    hand.i_moveAll(r,open_hand_pos_diff_from_grasp)
    rospy.sleep(10.)# TODO wait appropriately ....
    
    rospy.loginfo('%s', 'Go initial pose ...')
    hand.go_initial(r)
    rospy.sleep(10.)# TODO wait appropriately ....

def init_hand():
    hand.connect( rospy.get_param('/hiro_hand_ip_addr','192.168.128.130') )
    
    hand.servoOn(r)
    #hand.servoOn(l)
    
    # This gp_initial() move all finger joints to 0. position
    hand.go_initial(r)
    #hand.go_initial(l)
    
def shutdown_hand():
    hand.servoOff(r)    
    #hand.servoOff(l)
    
def hiro_hand_machine():
    rospy.init_node('hiro_hand_machine', log_level=rospy.DEBUG)
    
    init_hand()
    
#    grasp_srv = rospy.Service('grasp', ControlHand, handle_grasp_srv)
#    grasp_srv = rospy.Service('ungrasp', ControlHand, handle_ungrasp_srv)
    
    grasp_srv = rospy.Service('control_hand', ControlHand, handle_control_hand_srv)
    
    rospy.loginfo('%s', 'hiro_hand_machine: up and running...')
    
    # The main loop is here!
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():            
        rate.sleep()
        
    shutdown_hand()
  
if __name__ == "__main__":
    hiro_hand_machine()
