#! /usr/bin/env python3
import numpy as np  
import atexit
import time
import os
import signal
from numpy.linalg import norm, solve
from threading import Lock
from tkinter import Frame, Label, Tk

import rospy
from fake_moma.msg import MomaCmd
from geometry_msgs.msg import Pose
from moma_param import MomaParam

JOINT_ID = 0
Q_UP = "z"
Q_DOWN = "x"
UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"
AUTO = "g"

state = [False, False, False, False, False, False]
state_lock = Lock()
state_pub = None
root = None

vel = 0
omega = 0
q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
dq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
auto = True

def shift_q(q):
    qq = q
    for i in range(7):
        if qq[i] > np.pi:
            qq[i] -= 2 * np.pi
    return qq

def norm_yaw(yaw):
    y = yaw
    if y > 2*np.pi:
        y -= 2*np.pi
    elif y < 0.0:
        y += 2*np.pi
    return y

def keyeq(e, c):
    return e.char == c or e.keysym == c

def keyup(e):
    global state

    with state_lock:
        if keyeq(e, UP):
            state[0] = False
        elif keyeq(e, LEFT):
            state[1] = False
        elif keyeq(e, DOWN):
            state[2] = False
        elif keyeq(e, RIGHT):
            state[3] = False
        elif keyeq(e, Q_UP):
            state[4] = False
        elif keyeq(e, Q_DOWN):
            state[5] = False

def keydown(e):
    global state
    global auto
    global JOINT_ID

    with state_lock:
        if keyeq(e, QUIT):
            shutdown()
        elif keyeq(e, UP):
            state[0] = True
        elif keyeq(e, LEFT):
            state[1] = True
        elif keyeq(e, DOWN):
            state[2] = True
        elif keyeq(e, RIGHT):
            state[3] = True
        elif keyeq(e, Q_UP):
            state[4] = True
        elif keyeq(e, Q_DOWN):
            state[5] = True
        elif keyeq(e, "1") or keyeq(e, "2") or keyeq(e, "3") or keyeq(e, "4")\
             or keyeq(e, "5") or keyeq(e, "6") or keyeq(e, "7"):
            JOINT_ID = int(e.keysym) - 1
            print("\033[92mNow, joint id is ", int(e.keysym), "\033[93m\U000026A0\033[0m")
        elif keyeq(e, AUTO):
            auto = not auto
            if auto:
                print("\033[92mSet mode to AUTO. \033[93m\U000026A0\033[0m")
            else:
                print("\033[92mSet mode to MANUAL. \033[93m\U000026A0\033[0m")

def publish_cb(_):
    global vel
    global omega
    global q
    global auto
    global JOINT_ID

    if auto:
        return
    with state_lock:
        if state[0]:
            vel = min(vel+0.1, max_velocity)
        elif state[2]:
            vel = max(vel-0.1, -max_velocity)
        else:
            vel = 0

        if state[1]:
            omega = min(omega+0.1, max_w)
        elif state[3]:
            omega = max(omega-0.1, -max_w)
        else:
            omega = 0
        
        if state[4]:
            qq = q[JOINT_ID]+0.05
            q[JOINT_ID] = min(qq, max_q[JOINT_ID])
        elif state[5]:
            qq = q[JOINT_ID]-0.05
            q[JOINT_ID] = max(qq, min_q[JOINT_ID])
        
        # if state[4]:
        #     qq = dq[JOINT_ID]+0.05
        #     dq[JOINT_ID] = min(qq, max_dq[JOINT_ID])
        # elif state[5]:
        #     qq = dq[JOINT_ID]-0.05
        #     dq[JOINT_ID] = max(qq, min_dq[JOINT_ID])

        command.speed = vel
        command.angular_velocity = omega
        command.gripper_state = True
        command.q.data = shift_q(q)
        command.dq.data = np.array(dq)
        if state_pub is not None:
            state_pub.publish(command)

def exit_func():
    os.system("xset r on")

def shutdown():
    root.destroy()
    rospy.signal_shutdown("shutdown")

def main():
    global state_pub
    global root
    global command
    global max_velocity
    global max_w
    global min_q
    global min_dq
    global max_q
    global max_dq

    command = MomaCmd()
    max_velocity = MomaParam.max_v
    max_w = MomaParam.max_w
    
    min_q = MomaParam.joint_pos_limit_min
    max_q = MomaParam.joint_pos_limit_max
    min_dq = list(-np.array(MomaParam.joint_vel_limit))
    max_dq = MomaParam.joint_vel_limit

    state_pub = rospy.Publisher("/moma_cmd", MomaCmd, queue_size=1 )
    rospy.Timer(rospy.Duration(0.05), publish_cb)
    # rospy.Subscriber("/manual_target", Pose, pose_cb)

    atexit.register(exit_func)
    os.system("xset r off")

    root = Tk()
    frame = Frame(root, width=100, height=100)
    frame.bind("<KeyPress>", keydown)
    frame.bind("<KeyRelease>", keyup)
    frame.pack()
    frame.focus_set()
    lab = Label(
        frame,
        height=10,
        width=30,
        text="Focus on this window\nand use the WASDZX keys\nto drive the moma.\n\nPress G to change mode\n\nPress Q to quit",
    )
    lab.pack()
    print("Press %c to quit" % QUIT)
    print("Press %c to change mode" % AUTO)
    print("\033[92mNow, mode is AUTO. \033[93m\U000026A0\033[0m")

    root.mainloop()


if __name__ == "__main__":
    rospy.init_node("keyboard_control", disable_signals=True)

    signal.signal(signal.SIGINT, lambda s, f: shutdown())
    time.sleep(1) 
    main()
