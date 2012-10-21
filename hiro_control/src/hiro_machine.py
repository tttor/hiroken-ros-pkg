#!/usr/bin/env python
import roslib; roslib.load_manifest('hiro_control')
import rospy

from sensor_msgs.msg import JointState

from hiro_control.srv import *

import math
import sys

sys.path.append( rospy.get_param('/hiro_driver_path', '/home/vektor/4/ros-pkg/hiro_control/driver/hiro') )
#rospy.loginfo('%s', sys.path)
    
import KSP

hiro = KSP.Robot ( rospy.get_param('/hiro_ip_addr','192.168.128.129') )

def monitor_joint_state():
    raw_chest_joint_state = KSP.dblArray(1)
    raw_head_joint_states = KSP.dblArray(2)
    raw_rarm_joint_states = KSP.dblArray(6)
    
    hiro.getChestAngle(raw_chest_joint_state)
    raw_chest_joint_state[0] = math.radians(raw_chest_joint_state[0]) 
    
    hiro.getNeckAngles(raw_head_joint_states)        
    for i in range(2):
        raw_head_joint_states[i] = math.radians(raw_head_joint_states[i])
    
    hiro.rarm.getAngles(raw_rarm_joint_states)        
    for i in range(6):
        raw_rarm_joint_states[i] = math.radians(raw_rarm_joint_states[i])

#    # FOR TESTING PURPOSE, should not coexist with the real one
#    raw_chest_joint_state[0] = math.radians(0.)# For the torso, including the chest
#    
#    raw_head_joint_state[0] = math.radians(0.)# For the head, [0] is yaw/pan,
#    raw_head_joint_state[1] = math.radians(0.)# For the head, [1] is pitch/tilt
#    
#    raw_rarm_joint_state[0] = math.radians(0.)# joint_rshoulder_yaw
#    raw_rarm_joint_state[1] = math.radians(0.)# joint_rshoulder_pitch
#    raw_rarm_joint_state[2] = math.radians(0.)# joint_relbow_pitch
#    raw_rarm_joint_state[3] = math.radians(0.)# joint_rwrist_yaw
#    raw_rarm_joint_state[4] = math.radians(0.)# joint_rwrist_pitch
#    raw_rarm_joint_state[5] = math.radians(0.)# joint_rwrist_roll
        
    joint_states = JointState()
    
    # RARM joints
    joint_states.header.stamp = rospy.Time.now()
    joint_states.header.frame_id = '/none'

    joint_states.name.append('joint_chest_yaw')    
    joint_states.position.append(raw_chest_joint_state[0])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)   
    
    joint_states.name.append('joint_rshoulder_yaw')    
    joint_states.position.append(raw_rarm_joint_states[0])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)

    joint_states.name.append('joint_rshoulder_pitch')    
    joint_states.position.append(raw_rarm_joint_states[1])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)

    joint_states.name.append('joint_relbow_pitch')    
    joint_states.position.append(raw_rarm_joint_states[2])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)

    joint_states.name.append('joint_rwrist_yaw')    
    joint_states.position.append(raw_rarm_joint_states[3])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)

    joint_states.name.append('joint_rwrist_pitch')    
    joint_states.position.append(raw_rarm_joint_states[4])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)

    joint_states.name.append('joint_rwrist_roll')    
    joint_states.position.append(raw_rarm_joint_states[5])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)
    
    # Head joints
    joint_states.name.append('joint_head_yaw')    
    joint_states.position.append(raw_head_joint_states[0])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)
    
    joint_states.name.append('joint_head_pitch')    
    joint_states.position.append(raw_head_joint_states[1])
    joint_states.velocity.append(0)
    joint_states.effort.append(0)

    # Publish
    joint_states_pub = rospy.Publisher("joint_states", JointState)
    
    joint_states_pub.publish(joint_states)

def handle_control_rarm(req):
#def handle_control_rarm():
#    # Wacth out the order + should be in DEGREE
#    hiro.makePose(0.,# joint_chest_yaw
#                   0.,0.,# Head 
#                  -0.6, 0., -100.,  15.2, 9.4,  3.2,# right arm values
#                  -9.0, -154.8, -149.9, -116.1, -21.8, -10.0, # left arm joint values (IN DEGREE), which is the HOME_POSE values
#                   10)
    
    # Wacth out the order + should be in DEGREE
    hiro.makePose(math.degrees(req.desired.positions[0]),# joint_chest_yaw
                   0.,0.,# Head 
                   math.degrees(req.desired.positions[1]), math.degrees(req.desired.positions[2]), math.degrees(req.desired.positions[3]), math.degrees(req.desired.positions[4]), math.degrees(req.desired.positions[5]), math.degrees(req.desired.positions[6]),# right arm values
                   -9.0, -154.8, -149.9, -116.1, -21.8, -10.0, # left arm joint values (IN DEGREE), which is the HOME_POSE values
                   20)# Speed; Prev (integer): 10; 20; 25

    hiro.wait();# Wait until the currently running operation finished
    
    # Sense the results, should wait until makePose above is finished
    chest_joint_state = KSP.dblArray(1)
    rarm_joint_state = KSP.dblArray(6)

    hiro.getChestAngle(chest_joint_state)
    chest_joint_state[0] = math.radians(chest_joint_state[0]) 
    
    hiro.rarm.getAngles(rarm_joint_state)        
    for i in range(6):
        rarm_joint_state[i] = math.radians(rarm_joint_state[i])    

    # Wrap the response
    actual_positions = [chest_joint_state[0], rarm_joint_state[0], rarm_joint_state[1], rarm_joint_state[2], rarm_joint_state[3], rarm_joint_state[4], rarm_joint_state[5]]# Actually, this is rarm group that contains rarm joints and a chest joint
    actual_velocities = [req.desired.velocities[0], req.desired.velocities[1], req.desired.velocities[2], req.desired.velocities[3], req.desired.velocities[4], req.desired.velocities[5], req.desired.velocities[6]]# Up to now, just as the same as the req
    actual_accelerations = [req.desired.accelerations[0], req.desired.accelerations[1], req.desired.accelerations[2], req.desired.accelerations[3], req.desired.accelerations[4], req.desired.accelerations[5], req.desired.accelerations[6]]# Up to now, just as the same as the req
    actual_time_from_start = rospy.Duration(0.5)# This is useless so far
        
    res = trajectory_msgs.msg.JointTrajectoryPoint(actual_positions, actual_velocities, actual_accelerations, actual_time_from_start)
            
    return ControlArmResponse(res)

def init_hiro():
    rospy.loginfo('%s', 'Calibrating joints')
    hiro.jointCalib()# If the joint is really not calibrated yet, it moves arms to the front, otherwise it does not move arms at all

    rospy.sleep(5.)  

    rospy.loginfo('%s', 'Going to the home_pose')
    hiro.makeEscapePose(10)# This is what we call the home_pose. 
    hiro.wait()
    
def hiro_machine():
    rospy.init_node('hiro_machine', log_level=rospy.DEBUG)
    
    init_hiro()
    
    control_rarm_srv = rospy.Service('control_rarm', ControlArm, handle_control_rarm)
    
    # TODO make shutdown service
    rospy.loginfo('%s', 'hiro_machine: up and running...')
         
    # The main loop is here!
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():            
        monitor_joint_state()    
  
        rate.sleep()

if __name__ == "__main__":
    hiro_machine()
