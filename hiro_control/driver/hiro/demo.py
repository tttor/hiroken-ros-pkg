#! /usr/bin/python -i
# -*- coding: utf-8 -*-

"""@package demo
ロボット制御クラスライブラリをpythonから利用するサンプル
a sample python script using "class library to control hiro"
"""

import KSP;

robot = KSP.Robot("192.168.128.129");
rbt=robot;

arm   = KSP.dblArray(KSP.DOFS_ARM);   # for robot.[rl]arm.getAngles()
whole = KSP.dblArray(KSP.DOFS_WHOLE); # for robot.getAngles()

def jointCalib():
	robot.jointCalib()

def goInit():
    """ @brief ロボットを初期姿勢まで動かす """
    robot.makePose(0,0,0, 
                 -0.6, 0, -100,  15.2, 9.4,  3.2,
                  0.6, 0, -100, -15.2, 9.4, -3.2, 10);
    return ;

def goEscape():
    ## @ロボットをエスケープ姿勢まで動かす
    robot.makeEscapePose(10);
    return ;

def servoOFF():
    ## @全身のサーボを切る
    robot.servoOFF();
    return;

def servoON():
    robot.servoON();
    return;

def shutdown():
    robot.poweroff();
    return;

def test0():
    robot.rhand.setDefaultComplementType(KSP.COMPLEMENTBY_LINE);
    robot.lhand.setDefaultComplementType(KSP.COMPLEMENTBY_LINE);
    rxyz = KSP.dblArray(3);
    lxyz = KSP.dblArray(3);
    
    rxyz[0]= 300;  lxyz[0]= 300;
    rxyz[1]=-200;  lxyz[1]= 200;
    rxyz[2]= 200;  lxyz[2]= 200;

    robot.rhand.moveTo(rxyz);
    robot.lhand.moveTo(lxyz);
    
    robot.wait();
    for i in range(3):
        robot.rhand.move(0,0,50, 20, KSP.COMPLEMENTBY_ANGLE);
        robot.lhand.move(0,0,50, 20, KSP.COMPLEMENTBY_ANGLE);
        robot.wait();
        
        robot.rhand.moveTo(rxyz);
        robot.lhand.moveTo(lxyz);
        robot.wait();

def nod():
    tiltAngles = [ -20, 20, -20, 20, 0]
    for tilt in tiltAngles:
        robot.faceTo(0, tilt, 20)
        robot.wait()

def nodNo():
    panAngles = [ -20, 20, -20, 0]
    for pan in panAngles:
        robot.faceTo(pan, 0, 20)
        robot.wait()

def showDIN(userPortOnly = False):
    ret = robot.getDin()
    if userPortOnly:
        channels = range(16, 32)
    else:
        channels = range(0, 32)
    for i in channels:
        if i % 8 == 0:
            print('\nDIN  CH%2d-%2d:'%(i, i+7)),
        print('%1d'%(ret>>i & 1)),

def showDOUT(userPortOnly = False):
    ret = robot.getDoutAll()
    if userPortOnly:
        channels = range(16, 32)
    else:
        channels = range(0, 32)
    for i in channels:
        if i % 8 == 0:
            print('\nDOUT CH%2d-%2d:'%(i, i+7)),
        print('%1d'%(ret>>i & 1)),

def flipDOUT():
    ch = 16
    val = robot.getDout(ch)
    if val == 0:
        val = 0xFFFFFFFF # always on
    elif val == 0xFFFFFFFF:
        val = 100 # blink (T=val*10[msec], duty=50%)
    else:
        val = 0 # always off
    print ('setDout ch%2d:%d'%(ch, val))
    robot.setDout(ch, val)
