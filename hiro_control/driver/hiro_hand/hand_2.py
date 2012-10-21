#!/usr/bin/env python
# This file is derived from .../Hiro3Finger/script/client.py
from setup import getObjRef

R=0
L=1
RCONF='RHAND.conf'
LCONF='LHAND.conf'

joints = ['FIRST_1','FIRST_2','FIRST_3','SECOND_1','THUMB_1','THUMB_2']

# These 2 functions below are customized by Yoshinori Tone in Dec 29, 2011: get_electricity(id), get_speed(id):
def get_electricity(id):
    ele = []
    for jname in joints:
        dat=v3f.stat(id,jname)
        ele.append(dat[1][2])
    return ele

def get_speed(id):
    speed = []
    for jname in joints:
        dat=v3f.stat(id,jname)
        speed.append(dat[1][1])
    return speed
# ===================================================================================================================    
def connect(ip_addr):
    global v3f
    v3f = getObjRef(ip_addr)
    v3f.handInit(R,RCONF);
    v3f.handInit(L,LCONF);
    return v3f

def read(id,jname,dataid):
    dat=v3f.read(id,jname,dataid)
    print dat[1]

def write(id,jname,dataid,wdata):
    v3f.write(id,jname,dataid,wdata)

def servoOn(id):
    v3f.servoSwitch(id,'all',True)

def servoOff(id):
    v3f.servoSwitch(id,'all',False)

def calib(id,jname):
    v3f.calibration(id,jname)

def calibAll(id):
    v3f.calibration(id,'all')

def clear():
    v3f.clearCanBuffer(R)
    v3f.clearCanBuffer(L)

def go_initial(id):
    v3f.goInitial(id)

def get_angles(id):
    ang = []
    for jname in joints:
        dat=v3f.stat(id,jname)
        ang.append(dat[1][0])
    return ang

def i_move(id,jname,ang):
    dat=v3f.stat(id,jname)
    target=dat[1][0]+ang
    v3f.jointAngle(id,jname,target)

def i_servo(id,jname,ang):
    dat=v3f.stat(id,jname)
    target=dat[1][0]+ang
    v3f.servo(id,jname,target)

def i_moveAll(id,ang):
    dat=get_angles(id)
    for i in range(6):
        ang[i] += dat[i]
    v3f.jointAngleAll(id,ang)

def i_servoAll(id,ang):
    dat=get_angles(id)
    for i in range(6):
        ang[i] += dat[i]
    v3f.servoAll(id,ang)

def a_move(id,jname,target):
    v3f.jointAngle(id,jname,target)

def a_servo(id,jname,target):
    v3f.servo(id,jname,target)

def a_moveAll(id,target):
    v3f.jointAngleAll(id,target)

def a_servoAll(id,target):
    v3f.servoAll(id,target)

def test(id,cnt):
    go_initial(id)
    dat1=[-5,-5,-5,-5,-5,-5]
    dat2=[50,-10,30,60,-30,30]
    for i in range(cnt):
        a_moveAll(id,dat1)
        a_moveAll(id,dat2)

connect('192.168.128.130')# Commented because we want to access connect directly from hiro_machine
