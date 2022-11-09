#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


import sys, select, termios, tty

"""
Robot screws mapping

 BL(4)       FL(1)
(-/-/-)     (-/-/-)
      ----->
(-/-/-)     (-/-/-)
 BR(3)       FR(2)
"""
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        i    
   j    k    l
        ,    
For Holonomic mode (strafing), hold down the shift key:
---------------------------
       
   J         L
   
anything else : stop
q/z : increase/decrease max speeds by 10%
CTRL-C to quit
"""

moveBindings = { #screws 1,2,3,4 as illustrated above
        'i':(1,1,-1,-1),
        'j':(-1,1,-1,1),
        'l':(1,-1,1,-1),
        ',':(-1,-1,1,1),
        'J':(-1,1,1,-1),
        'L':(1,-1,-1,1),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher_FR = rospy.Publisher('/robominer/FR_screw_velocity_controller/command', Float64, queue_size = 1)
        self.publisher_FL = rospy.Publisher('/robominer/FL_screw_velocity_controller/command', Float64, queue_size = 1)
        self.publisher_BR = rospy.Publisher('/robominer/BR_screw_velocity_controller/command', Float64, queue_size = 1)
        self.publisher_BL = rospy.Publisher('/robominer/BL_screw_velocity_controller/command', Float64, queue_size = 1)

        self.speed = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, FL, FR, BR, BL, speed):
        self.condition.acquire()
        self.FL = FL
        self.FR = FR
        self.BR = BR
        self.BL = BL
        self.speed = speed
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            self.condition.release()

            # Publish.
            self.publisher_FR.publish(self.FR)
            self.publisher_FL.publish(self.FL)
            self.publisher_BL.publish(self.BL)
            self.publisher_BR.publish(self.BR)

        # Publish stop message when thread exits.
        self.publisher_FL.publish(0)
        self.publisher_FR.publish(0)
        self.publisher_BR.publish(0)       
        self.publisher_BL.publish(0)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed):
    return "currently:\tspeed %s " % (speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 1.5)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    FR = 0
    FL = 0
    BR = 0
    BL = 0
    status = 0

    try:
        pub_thread.update(FL, FR, BR, BL, speed)

        print(msg)
        print(vels(speed))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                FL = speed*moveBindings[key][0]
                FR = speed*moveBindings[key][1]
                BR = speed*moveBindings[key][2]
                BL = speed*moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                print(vels(speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and FR == 0 and FL == 0 and BR == 0 and BL == 0:
                    continue
                FR = 0
                FL = 0
                BR = 0
                BL = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(FL, FR, BR, BL, speed)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
