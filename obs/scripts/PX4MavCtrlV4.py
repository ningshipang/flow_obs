import socket
import threading
import tkinter
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct

is_takeoff, is_hover, is_Forward, is_land = False, False, False, False


class PX4_CUSTOM_MAIN_MODE:
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
    PX4_CUSTOM_MAIN_MODE_SIMPLE = 9

class PX4_CUSTOM_SUB_MODE_AUTO:
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)

class PX4MavCtrler:

    def __init__(self, port):
        self.f = fifo
        self.stopFlag = False
        self.mav0 = mavlink2.MAVLink(self.f)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.ip = '127.0.0.1'  # 服务器ip和端口
        self.port = port
        self.uavAngEular = [0, 0, 0]
        self.uavAngRate = [0, 0, 0]
        self.uavPosNED = [0, 0, 0]
        self.uavVelNED = [0, 0, 0]
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = False
        self.isArmed = False
        self.hasSendDisableRTLRC = False
        self.is_takeoff, self.is_hover, self.is_Forward, self.is_land = False, False, False, False
        print("Thread Started!")


    def InitMavLoop(self):
        self.the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))    
        self.lastTime = 0            
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1.start()
        self.t2 = threading.Thread(target=self.OffboardSendMode, args=())
        self.startTime = time.time()
        self.lastTime2 = 0
        self.startTime2 = time.time()
        self.t3 = threading.Thread(target=self.read_kbd_input, args=())

    def keyInput(self):
        self.t3.start()

    def call(self, event):
        k = event.keysym
        if k == "a":
            self.is_takeoff = True
            self.is_hover = False
            self.is_Forward = False
            self.is_land = False
        elif k == "b":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = True
            self.is_land = False
        elif k == "c":
            self.is_takeoff = False
            self.is_hover = True
            self.is_Forward = False
            self.is_land = False
        elif k == "d":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = False
            self.is_land = True
        time.sleep(0.02)

    def read_kbd_input(self):
        win = tkinter.Tk()
        frame = tkinter.Frame(win,width=200,height=120)
        frame.bind("<Key>",self.call)
        frame.focus_set()
        frame.pack()
        win.mainloop()
        
    def sat(self,inPwm,thres=1):
        outPwm= inPwm
        if inPwm>thres:
            outPwm = thres
        elif inPwm<-thres:
            outPwm = -thres
        return outPwm

    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        buf = self.mav0.command_long_encode(self.the_connection.target_system, self.the_connection.target_component,
                                            command, 0,
                                            param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))
        #print(self.the_connection.target_system)
        #print(self.the_connection.target_component)


    #def SendManualCTRL(self,x,y,z,r):
    #    buf = self.mav0.manual_control_encode(self.the_connection.target_system,x,y,z,r,0).pack(self.mav0)
    #    self.udp_socket.sendto(buf, (self.ip, self.port))

    def sendMavOffboardCmd(self,type_mask,coordinate_frame, x,  y,  z,  vx,  vy,  vz,  afx,  afy,  afz,  yaw, yaw_rate):
        time_boot_ms = int(time.time()*1000000)
        buf = self.mav0.set_position_target_local_ned_encode(time_boot_ms,self.the_connection.target_system,
                                                             self.the_connection.target_component,
                                                             coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                             afy,  afz,  yaw, yaw_rate).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))


    def sendMavOffboardAPI(self,EnList,coordinate_frame,pos,vel,acc,yaw,yawrate):
        enPos = EnList[0]
        enVel = EnList[1]
        enAcc = EnList[2]
        enForce = EnList[3]
        enYaw = EnList[4]
        EnYawrate= EnList[5]
        y=int(0)
        if not enPos:
            y = y | 7

        if not enVel:
            y = y | (7<<3)

        if not enAcc:
            y = y | (7<<6)

        if not enForce:
            y = y | (1<<9)

        if not enYaw:
            y = y | (1<<10)

        if not EnYawrate:
            y = y|(1<<11)
        type_mask = y
        time_boot_ms = (time.time()-self.startTime)*1000
        buf = self.mav0.set_position_target_local_ned_encode(int(time_boot_ms),self.the_connection.target_system,
                                                             self.the_connection.target_component,
                                                             coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                             vel[0],  vel[1],  vel[2],  acc[0],
                                                             acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    def SendVelNED(self,vx,vy,vz,yawrate):
        #self.initOffboard()
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate
        #self.the_connection.base_mode
        #self.sendMavOffboardAPI(EnList, coordinate_frame, pos, vel, acc, yaw, yawrate)

    def sendUE4Cmd(self,cmd,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("i52s",1234567890,cmd)
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("i52s",1234567890,cmd)
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4Pos(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) 
                
        
    def SendVelFRD(self,vx,vy,vz,yawrate):
        #self.initOffboard()
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        #self.vel = [vx,vy,vz]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate
        #self.sendMavOffboardAPI(EnList, coordinate_frame, pos, vel, acc, yaw, yawrate)

    def SendPosNED(self,x,y,z,yaw):
        #self.initOffboard()
        self.EnList = [1,0,0,0,1,0]
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw
        # if not self.hasSendDisableRTLRC:
        #     self.sendMavOffboardAPI(EnList, coordinate_frame, pos, vel, acc, yaw, yawrate)
        #     self.sendMavOffboardAPI(EnList, coordinate_frame, pos, vel, acc, yaw, yawrate)
        #     self.initOffboard()
        #self.sendMavOffboardAPI(EnList, coordinate_frame, pos, vel, acc, yaw, yawrate)

    def initOffboard(self):
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = True
        #self.SendSetMode(4)
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
        time.sleep(0.5)
        self.t2.start()
        self.sendMavOffboardAPI(self.EnList, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        self.sendMavOffboardAPI(self.EnList, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        if not self.hasSendDisableRTLRC:
            self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
            self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
            self.hasSendDisableRTLRC = True
        if not self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)


    def endOffboard(self):
        self.isInOffboard = False
        if self.hasSendDisableRTLRC:
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, False)
            self.hasSendDisableRTLRC = False
        #self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_STABILIZED)
        if self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)
        self.t2.join()

    def sendMavSetParam(self,param_id, param_value, param_type):
        buf = self.mav0.param_set_encode(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    def SendHILCtrlMsg(self):
        controls = [1100,1900,1100,0,0,0,0,0,1600,1700,1200,0,0,0,0,0]
        buf = self.mav0.hil_actuator_controls_encode(int(time.time()*1000000),controls,1,1).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))
        print("Msg Send.")

    def SendHILCtrlMsg1(self):
        #controls = [1100,1900,1100,0,0,0,0,0,1600,1700,1200,0,0,0,0,0]
        #buf = self.mav0.hil_actuator_controls_encode(int(time.time()*1000000),controls,1,1).pack(self.mav0)
        name = b'hello'
        buf = self.mav0.debug_vect_encode(name, int(time.time()*1000000), 1100, 1500, 1700).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))
        print("Msg1 Send.")

    def SendMavArm(self, isArm):
        #print("Arm")
        if (isArm):
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        else:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)

    def SendRcOverride(self, ch1=1500, ch2=1500, ch3=1100, ch4=1500, ch5=1100, ch6=1100, ch7=1500, ch8=1500):
        buf = self.mav0.rc_channels_override_encode(self.the_connection.target_system,
                                                    self.the_connection.target_component, ch1, ch2,
                                                    ch3, ch4, ch5, ch6, ch7, ch8).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    def sendMavManualCtrl(self, x,y,z,r):
        buf = self.mav0.manual_control_encode(self.the_connection.target_system, x,y,z,r,0).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))



    def SendSetMode(self,mainmode,cusmode=0):
        basemode = mavlink2.MAV_MODE_FLAG_HIL_ENABLED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.SendMavCmdLong(mavlink2.MAV_CMD_DO_SET_MODE, basemode, mainmode, cusmode)

    def stopRun(self):
        self.stopFlag=True
        time.sleep(0.5)
        self.t1.join()
        if(self.isInOffboard):
            self.endOffboard()
        self.the_connection.close()

    def getMavMsg(self):
        self.lastTime = time.time()
        while True:
            if self.stopFlag:
                break
            self.lastTime = self.lastTime + 0.01
            sleepTime = self.lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.lastTime = time.time()
            # print(time.time())
            while True:
                if self.stopFlag:
                    break
                msg = self.the_connection.recv_match(
                    type=['ATTITUDE', 'LOCAL_POSITION_NED','HEARTBEAT'],
                    blocking=False)
                if msg is not None:
                    #print(msg)
                    if msg.get_type() == "ATTITUDE":
                        self.uavAngEular[0] = msg.roll
                        self.uavAngEular[1] = msg.pitch
                        self.uavAngEular[2] = msg.yaw
                        #print(msg.yaw)
                        self.uavAngRate[0] = msg.rollspeed
                        self.uavAngRate[1] = msg.pitchspeed
                        self.uavAngRate[2] = msg.yawspeed
                    if msg.get_type() == "LOCAL_POSITION_NED":
                        self.uavPosNED[0] = msg.x
                        self.uavPosNED[1] = msg.y
                        self.uavPosNED[2] = msg.z
                        self.uavVelNED[0] = msg.vx
                        self.uavVelNED[1] = msg.vy
                        self.uavVelNED[2] = msg.vz
                        #print(msg.z)
                        #print(msg.vz)
                        #print("Local Pos: "+str(msg.x)+", "+str(msg.y)+", "+str(msg.z))
                    if msg.get_type() == "HEARTBEAT":
                        isArmed = msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
                        if not self.isArmed and isArmed:
                            print("PX4 Armed!")
                        if self.isArmed and not isArmed:
                            print("PX4 DisArmed!")
                        self.isArmed = isArmed
                        #print("HeartBeat!")
                else:
                    break
        print("Mavlink Stoped.")

    def OffboardSendMode(self):
        self.lastTime2 = time.time()
        while True:
            if not self.isInOffboard:
                break
            self.lastTime2 = self.lastTime2 + 0.01
            sleepTime = self.lastTime2 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.lastTime2 = time.time()
            # print(time.time())
            if self.isInOffboard:
                self.sendMavOffboardAPI(self.EnList, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw, self.yawrate)
                #print("Offboard Msg Send")
        print("Offboard Stoped.")
