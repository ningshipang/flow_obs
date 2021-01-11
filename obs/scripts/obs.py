#!/usr/bin/env python
#coding=utf-8

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
# from std_msgs import Float64  Float64MultiArray
from std_msgs.msg import Float64
import time
import math
import numpy as np

#keyboard input
import threading
import tkinter



class Px4Controller:

    def __init__(self):
        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.local_enu_position = None

        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.imu_rate = np.array([0, 0, 0])

        self.state = None
        self.mavros_state = State()
        self.command = TwistStamped()
        self.mav_pos = np.array([0, 0, 0])
        self.mav_vel = np.array([0, 0, 0])
        self.cmd_vel = np.array([0, 0, 0])
        self.cmd_body_vel = np.array([0, 0, 0])
        self.cmd_force_vel = np.array([0, 0, 0])
        self.ttc = 0
        self.cmd_pos_vel = np.array([0, 0, 0])
        # self.q = Queue()
        self.mav_yaw = 0
        self.cmd_yaw = 0
        self.mav_roll = 0
        self.mav_pitch = 0
        self.mav_R = np.zeros((3,3))
        self.desire_pos = np.array([0, 8, 0])  #[20, 30, 0]
        self.start_yaw = 0

        '''
        ros subscribers   /balance_force  /ttc_mean
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.balance_force = rospy.Subscriber("/balance_force", Float64, self.force_callback)
        self.ttc_mean = rospy.Subscriber("/ttc_mean", Float64, self.ttc_callback)
        self.mav_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.mav_vel_cb)
        # self.imu_sub = rospy.Subscriber("mavros/imu/data", Imu, self.mav_vel_cb)

        self.is_takeoff, self.is_hover, self.is_Forward, self.is_land = False, False, False, False
        self.is_force = False
        self.is_offboard = False
        
        self.P_pos = 0.1
        self.P_yaw = 0.1
        self.P_heigh = 0.5
        self.start_height = 0 
        self.distance = 0 

        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")

    def call(self, event):
        k = event.keysym
        if k == "a":
            self.is_takeoff = True
            self.is_hover = False
            self.is_Forward = False
            self.is_land = False
            self.is_force = False
        elif k == "b":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = True
            self.is_land = False
            self.is_force = False
        elif k == "c":
            self.is_takeoff = False
            self.is_hover = True
            self.is_Forward = False
            self.is_land = False
            self.is_force = False
        elif k == "d":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = False
            self.is_land = True
            self.is_force = False
        elif k == "e":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = False
            self.is_land = False
            self.is_force = True
        elif k == "p":
            self.is_offboard = True
        time.sleep(0.02)

    def read_kbd_input(self):
        win = tkinter.Tk()
        frame = tkinter.Frame(win,width=200,height=120)
        frame.bind("<Key>",self.call)
        frame.focus_set()
        frame.pack()
        win.mainloop()


    def start(self):
        rospy.init_node("offboard_node")
        rate = rospy.Rate(20)

        while(not self.mavros_state.connected):
            print(self.mavros_state.connected)
            rate.sleep()

        for i in range(10):
            self.vel_pub.publish(self.command)
            # self.arm_state = self.arm()
            # self.offboard_state = self.offboard()
            rate.sleep()
        
        self.offboard_state = SetMode()
        self.offboard_state.custom_mode = "OFFBOARD"
        self.arm_state = CommandBool()
        self.arm_state.value = True

        start_time = rospy.Time.now()
        '''
        main ROS thread
        '''
        cnt = -1
        while (rospy.is_shutdown() is False):
            cnt += 1
            if self.is_offboard == False:
                if self.mavros_state.mode == "OFFBOARD":
                    # self.flightModeService.call(custom_mode='POSCTL')
                    resp1 = self.flightModeService(0, "POSCTL")
                if cnt % 10 == 0:
                    print("Enter MANUAL mode")
                rate.sleep()
                self.start_height = self.mav_pos[2]
                self.start_yaw = self.mav_yaw
                continue
            else:
                if self.mavros_state.mode != "OFFBOARD":
                    # self.flightModeService.call(custom_mode='OFFBOARD')
                    resp1 = self.flightModeService( 0,self.offboard_state.custom_mode )
                    if resp1.mode_sent:
                        print("Offboard enabled")
                    start_time = rospy.Time.now()
                # print("mav_pos: {}".format(self.mav_pos))
                # print("mav_vel: {}".format(self.mav_vel))  
                # print("ttc: {}".format(self.ttc))  
                # print("distance: {}".format(self.distance))
                # print("imu_rate: {}".format(self.imu_rate))
                # print("force: {}".format(self.cmd_force_vel))
                self.mav_state()
            
            self.command.twist.linear.x = self.cmd_vel[0]
            self.command.twist.linear.y = self.cmd_vel[1]
            self.command.twist.linear.z = self.cmd_vel[2]
            self.command.twist.angular.z = self.cmd_yaw
            self.vel_pub.publish(self.command)
            rate.sleep()
 
    def mav_state(self):
        self.pos_vel()
        self.control_yaw()
        # cmd = self.cmd_pos_vel + self.cmd_force_velc
        cmd = self.cmd_force_vel
        cmd = self.SatIntegral(cmd, 3, -3)
        if self.is_takeoff == True:
            self.cmd_body_vel = np.array([0, 0, 1])
            self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0
        elif self.is_Forward == True:
            self.cmd_body_vel = np.array([2, 0, 0])
            self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.start_height = self.mav_pos[2]
            self.cmd_vel[2] = 0
            # self.cmd_vel = np.array([0, 2, 0])
            self.cmd_yaw = 0
            # self.cmd_vel = [0, 1, 0]
        elif self.is_hover == True:
            # self.cmd_body_vel = np.array([0, 0, 0])
            # self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.cmd_vel = np.array([0, 0, 0])
            self.cmd_vel[2] = 0
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0
        elif self.is_land == True:
            # self.cmd_body_vel = np.array([0, 0, -1])
            # self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.cmd_vel = np.array([0, 0, -1])
            self.cmd_vel[0] = 0
            self.cmd_vel[1] = 0
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0
        elif self.is_force == True:
            self.cmd_vel = np.array([0, 1.5, 0])  #1.0
            # self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            # force_yaw = self.control_yaw()
            # self.cmd_yaw = self.cmd_yaw
            self.cmd_vel = cmd + self.cmd_vel
        else:
            self.cmd_vel = np.array([0, 0, 0])
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        # self.mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        self.mav_pitch = math.asin(2*(q0*q2 - q1*q3))
        self.mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
        R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                          [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                          [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
        R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]])
        self.mav_R = R_ae.dot(R_ba)  #机体坐标系  x————向右  y————向前   z————向上
        

    def force_callback(self, msg):
        dlt_x = (self.desire_pos[0] - self.mav_pos[0]) * (self.desire_pos[0] - self.mav_pos[0])
        dlt_y = (self.desire_pos[1] - self.mav_pos[1]) * (self.desire_pos[1] - self.mav_pos[1])
        self.distance = math.sqrt(dlt_x + dlt_y)
        self.cmd_force_vel = np.array([0, 0, 0])
        self.cmd_force_vel[0] = 0.10 * msg.data   #0.1
        print("cmd_force: {}".format(self.cmd_force_vel[0]))
        self.cmd_force_vel = self.mav_R.dot(self.cmd_force_vel)  #body to NEU
        self.cmd_force_vel[2] = 0
        # print("force_vel: {}".format(self.cmd_force_vel))
        # if self.distance > 4:
        #     self.cmd_force_vel = np.array([0, 0, 0])
        # self.cmd_force_vel = np.array([0, 0, 0])
        # return self.mav_force_vel
    
    def ttc_callback(self, msg):
        self.ttc = msg.data   #
        # self.cmd_force_vel = self.mav_R.dot(self.cmd_force_vel)  #body to NEU
        # return self.mav_force_vel

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def pos_vel(self):
        self.cmd_pos_vel = self.sat(self.P_pos * (self.desire_pos - self.mav_pos), 2)
        # pos_error = self.desire_pos - self.mav_pos
        # 保持进入路径跟随模式时，高度不发生变化
        self.cmd_pos_vel[2] = self.Saturation(self.P_heigh * (self.start_height - self.mav_pos[2]), 1, -1)
        # if self.distance > 8:
        print("cmd_pos_vel: {}".format(self.cmd_pos_vel))

    # 保持朝目标方向
    def control_yaw(self):
        desire_yaw = math.atan2(self.desire_pos[1] - self.mav_pos[1], self.desire_pos[0] - self.mav_pos[0])
        # desire_yaw = -2.7  #1.2  -2.7
        desire_yaw = self.start_yaw
        dlt_yaw = self.minAngleDiff(desire_yaw, self.mav_yaw)
        self.cmd_yaw = self.Saturation(self.P_yaw * dlt_yaw, 0.2, -0.2)
        # pos_error = self.desire_pos - self.mav_pos
        # self.cmd_pos_vel[2] = 0
        print("desire_yaw: {}".format(desire_yaw))
        print("mav_yaw: {}".format(self.mav_yaw))
        print("cmd_yaw: {}".format(self.cmd_yaw))

    def minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a/n*maxv
        else:
            return a

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a
    
    def Saturation(self, a, up, down):
        if a > up:
            a = up
        elif a < down:
            a = down
        return a

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg
        self.imu_rate = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

if __name__ == '__main__':
    con = Px4Controller()
    # con.read_kbd_input()
    # threading.Thread(target=con.read_kbd_input, args=())
    inputThread = threading.Thread(target=con.read_kbd_input)
    inputThread.start()
    con.start()
