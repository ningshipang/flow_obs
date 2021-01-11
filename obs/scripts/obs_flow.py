import time
import mmap
import numpy as np
import cv2.cv2 as cv2
from pymavlink.dialects.v20 import common as mavlink2
# from enum import Enum
import math

import win32gui, win32ui, win32con
from ctypes import windll

import sys
import PX4MavCtrlV4 as PX4MavCtrl

#keyboard input
import threading
import tkinter

# is_takeoff = False
# is_hover = False
# is_Forward = False
# is_land = False
# lastTime, startTime, timeInterval = 0, 0, 0
# is_initialize_rc = False

width = 720
height = 405
channel = 4

#optical keypoint
num = 0
FRAME_STEP = 1
keypoints = []
prev_keypoints = []
flow = []
st, err = 0, 0
old_gray = []
is_keypoints = True
pxvec = []
B = []


# ShiTomasi corner detection的参数
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7) #使用的邻域数 默认是3 这里是7
# 光流法参数
# winSize ：每个金字塔等级的搜索窗口的winSize大小。
# maxLevel 未使用的图像金字塔层数，基于0的最大金字塔等级数;如果设置为0，则不使用金字塔（单级），如果设置为1，则使用两个级别，
#依此类推;如果将金字塔传递给输入，那么算法将使用与金字塔一样多的级别，但不超过maxLevel
# criteria ：参数，指定迭代搜索算法的终止条件（在指定的最大迭代次数criteria.maxCount之后或当搜索窗口移动小于criteria.epsilon时）
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# 创建随机生成的颜色
color = np.random.randint(0, 255, (100, 3))



def window_enumeration_handler(hwnd, window_hwnds):
    if win32gui.GetClassName(hwnd) == "UnrealWindow":
        window_hwnds.append(hwnd)

class WinInfo:
    def __init__(self, hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC):
        self.hWnd = hWnd
        self.width = width
        self.height = height
        self.saveDC = saveDC
        self.saveBitMap = saveBitMap
        self.mfcDC = mfcDC
        self.hWndDC = hWndDC

def getHwndInfo(hWnd):
    left, top, right, bot = win32gui.GetClientRect(hWnd)
    width = right - left
    height = bot - top
    print((width,height))
    if hWnd and width == 0 and height == 0:
        print("窗口不能最小化，请保持窗口在后台")
        sys.exit(1)

    # 返回句柄窗口的设备环境，覆盖整个窗口，包括非客户区，标题栏，菜单，边框
    hWndDC = win32gui.GetWindowDC(hWnd)

    # 创建设备描述表
    mfcDC = win32ui.CreateDCFromHandle(hWndDC)
    # 创建内存设备描述表
    saveDC = mfcDC.CreateCompatibleDC()
    # 创建位图对象准备保存图片
    saveBitMap = win32ui.CreateBitmap()
    # 为bitmap开辟存储空间
    saveBitMap.CreateCompatibleBitmap(mfcDC, width, height)

    #hPen = win32gui.CreatePen(win32con.PS_SOLID,1,win32api.RGB(255,0,0)) # 定义框颜色
    #win32gui.SelectObject(hWndDC,hPen)        
    # hbrush = win32gui.GetStockObject(win32con.NULL_BRUSH) # 定义透明画刷，这个很重要！！
    # prebrush=win32gui.SelectObject(hWndDC,hbrush)

    info = WinInfo(hWnd, width, height, saveDC, saveBitMap, mfcDC, hWndDC)
    return info
    # return {'width':width,'height':height,'saveDC':saveDC,'saveBitMap':saveBitMap}

def getCVImg(wInfo):
    wInfo.saveDC.SelectObject(wInfo.saveBitMap)
    # 保存bitmap到内存设备描述表
    #wInfo.saveDC.BitBlt((0,0), (wInfo.width,wInfo.height), wInfo.mfcDC, (0, 0), win32con.SRCCOPY)

    # 如果要截图到打印设备：
    ###最后一个int参数：0-保存整个窗口，1-只保存客户区。如果PrintWindow成功函数返回值为1
    result = windll.user32.PrintWindow(wInfo.hWnd, wInfo.saveDC.GetSafeHdc(), 1)
    # print(result) #PrintWindow成功则输出1

    # 保存图像
    ##方法一：windows api保存
    ###保存bitmap到文件
    # saveBitMap.SaveBitmapFile(saveDC,"img_Winapi.bmp")

    signedIntsArray = wInfo.saveBitMap.GetBitmapBits(True)

    ##方法三（第二部分）：opencv+numpy保存
    ###PrintWindow成功，保存到文件，显示到屏幕
    im_opencv = np.frombuffer(signedIntsArray, dtype='uint8')
    im_opencv.shape = (wInfo.height, wInfo.width, 4)
    
    #cv2.cvtColor(im_opencv, cv2.COLOR_BGRA2RGB)
    return im_opencv

def Optical_flow(object):
    global num, FRAME_STEP, old_gray, prev_keypoints, is_keypoints, pxvec, B
    frame = getCVImg(ImgInfo1)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#转化为灰度虚图像
    # vis = frame.copy()
    # cv2.imshow('frame', frame_gray)
    # cv2.waitKey(1)
    if(num % (15*FRAME_STEP) == 0):
        keypoints = []
        keypoints = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        # print("first keypoint is ok")
        old_gray = frame_gray.copy() #更新前一帧图片和角点的位置
        cv2.imshow('old_gray', old_gray)
        cv2.waitKey(1)
        if keypoints is not None:
            prev_keypoints = keypoints.copy()
        else:
            prev_keypoints = np.float32([[0,0], [0,0]])
        # prev_keypoints = keypoints.copy()
        # print("num is ".format(num))
        num = num + 1

    mask = np.zeros_like(frame)
    keypoints, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, prev_keypoints, None, **lk_params)
    if st.any() == 1:
        is_keypoints = True
    else:
        is_keypoints = False

    # 选择good points
    if is_keypoints == True:
        good_key = keypoints[st == 1]
        good_oldkey = prev_keypoints[st == 1]
    else:
        good_key = np.float32([[0,0], [0,0]])
        good_oldkey = np.float32([[0,0], [0,0]])
    # good_key = keypoints[st == 1]
    # good_oldkey = prev_keypoints[st == 1]

     # 计算光流
    # if is_keypoints == True: 
    flow = good_key - good_oldkey
    flow = flow.reshape(-1, 2)
    print("flow is {}".format(flow))
    print("good_key is {}".format(good_key))

    #计算延伸焦点 Focus of Extend
    pxvec = []
    for line in range(len(flow)):  #计算平移分量
        pxvec_x = flow[line][0] - (good_key[line][0] * good_key[line][1] * object.uavAngRate[0]
                   - (1 + good_key[line][0] * good_key[line][0]) * object.uavAngRate[1]
                   + good_key[line][1] * object.uavAngRate[2])
        pxvec_y = flow[line][1] - ((1 + good_key[line][1] * good_key[line][1]) * object.uavAngRate[0]
                   - good_key[line][0] * good_key[line][1] * object.uavAngRate[1]
                   - good_key[line][0] * object.uavAngRate[2])
        pxvec.append((pxvec_x, pxvec_y))
    pxvec = np.float32(pxvec).reshape(-1, 2)

    #求解A和B矩阵
    trans = np.array([[0, -1], [1, 0]])
    A = pxvec.dot(trans)
    B = []
    # print("A size is {}".format(A.shape))
    # print("range(len(pxvec)) size is {}".format(range(len(pxvec))))
    for line_B in range(len(pxvec)):  #逐个计算B
        trans_b = good_key[line_B][0] * pxvec[line_B][1] - good_key[line_B][1] * pxvec[line_B][0]
        B.append(trans_b)
    # print("original B size is {}".format(range(len(B))))
    B = np.float32(B).reshape(-1, 1)
    # print("B size is {}".format(range(len(B))))
    #求解foe
    if is_keypoints == True:
        try:
            ATA = np.linalg.inv(A.T.dot(A))
            foe = ATA.dot(A.T)
            FOE = foe.dot(B)
        except:
            FOE = np.float32([0, 0]).reshape(-1, 1)
    else:
        FOE = np.float32([0, 0]).reshape(-1, 1)
    # print("A is {}".format(A))
    # print("ATA is {}".format(ATA))
    # print("ATA size is {}".format(ATA.shape))
    # print("FOE size is {}".format(FOE.shape))
    
    #计算碰撞时间 Time to Collision
    TTC = []
    for i in range(len(pxvec)):
        ttc_up = (flow[i][0] - FOE[0][0]) * (flow[i][0] - FOE[0][0]) + (flow[i][1] - FOE[1][0]) * (flow[i][1] - FOE[1][0])
        ttc_down = pxvec[i][0] * pxvec[i][0] + pxvec[i][1] * pxvec[i][1]
        ttc = math.sqrt(ttc_up/ttc_down)
        TTC.append(ttc)
    
    #统计图像块中的点数
    BLOCK_NUM_X = 16
    BLOCK_NUM_Y = 16 #将图像分解成16*16的矩阵块
    # img_width = 720 img.shape[0]
    # img_height = 405
    img_width = frame_gray.shape[1]
    img_height = frame_gray.shape[0]
    kp_cnt = np.zeros((BLOCK_NUM_X,BLOCK_NUM_Y))
    ttc_sum = np.zeros((BLOCK_NUM_X,BLOCK_NUM_Y))
    for j in range(len(pxvec)):
        index_x = int((good_key[j][0] * BLOCK_NUM_X)/(img_width))
        index_y = int((good_key[j][1] * BLOCK_NUM_Y)/(img_height))
        if index_x >16 or index_x ==16:
            index_x = 15
        else:
            index_x = index_x
        if index_y >16 or index_y ==16:
            index_y = 15
        else:
            index_y = index_y
        kp_cnt[index_x][index_y] = kp_cnt[index_x][index_y] + 1
        ttc_sum[index_x][index_y] = TTC[j] #每个块中总碰撞时间
    
    #计算块中的平均碰撞时间
    ttc_block = np.zeros((BLOCK_NUM_X,BLOCK_NUM_Y))
    for block_i in range(BLOCK_NUM_X):
        for block_j in range(BLOCK_NUM_Y):
            if(kp_cnt[block_i][block_j] != 0):
                ttc_block[block_i][block_j] = ttc_sum[block_i][block_j] / kp_cnt[block_i][block_j]
    

    
    # 绘制跟踪框（即跟踪轨迹）
    for i, (new, old) in enumerate(zip(good_key, good_oldkey)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)
        frame = cv2.circle(frame, (a, b), 5, color[i].tolist(), -1)

     # 显示碰撞时间
    for ttc_i in range(BLOCK_NUM_X):
        for ttc_j in range(BLOCK_NUM_Y):
            if(kp_cnt[ttc_i][ttc_j] != 0):
                font = cv2.FONT_HERSHEY_SIMPLEX  # 使用默认字体
                cv2.putText(frame, "ttc:" + str(ttc_block[ttc_i][ttc_j]),
                           (round(img_width/BLOCK_NUM_X * ttc_i), round(img_height/BLOCK_NUM_Y * ttc_j)), 
                           font, 0.5, (255, 255, 255), 1,8)
                print("TTC show is ok")
        

    img = cv2.add(frame, mask)  #使用cv2.add()将mask和frame的像素点相加并进行展示
    cv2.imshow('frame', img)
    cv2.waitKey(1)

    old_gray = frame_gray.copy() #更新前一帧图片和角点的位置
    prev_keypoints = good_key.reshape(-1, 1, 2) #间隔两个转换为1列
    num = num + 1

def clearHWND(wInfo):
    win32gui.DeleteObject(wInfo.saveBitMap.GetHandle())
    wInfo.saveDC.DeleteDC()
    wInfo.mfcDC.DeleteDC()
    win32gui.ReleaseDC(wInfo.hWnd, wInfo.hWndDC)
    

if __name__=="__main__":
    mav = PX4MavCtrl.PX4MavCtrler(20100)
    mav.InitMavLoop()
    mav.sendUE4Cmd(b'RflyChangeMapbyName VisionRing',0)
    mav.sendUE4Cmd(b'RflyChangeMapbyName VisionRing',1)
    time.sleep(0.5)
    #新建一个球形，设置位置和坐标，使用默认样式红色
    # mav.sendUE4Pos(100,152,0,[80,0,-2],[0,0,0])  #(100,152,0,[3,0,-2],[0,0,0])
    # mav.sendUE4Pos(100,152,0,[40,0,-2],[0,0,0])  #(100,152,0,[3,0,-2],[0,0,0])
    # mav.sendUE4Pos(100,152,0,[50,10,-2],[0,0,0])  #(100,152,0,[3,0,-2],[0,0,0])
    mav.sendUE4Pos(100,152,0,[60,-10,-2],[0,0,0])  #(100,152,0,[3,0,-2],[0,0,0])
    time.sleep(0.5)
    #将视角切换到1视角（前置摄像头）    
    mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',1)
    time.sleep(0.5)
    #设置前置摄像头位置在相对机体中心[0.3,0,0.05]的地方
    mav.sendUE4Cmd(b'RflyCameraPosAng 0.3 0 0 0 0 0',1)

    window_hwnds = []
    win32gui.EnumWindows(window_enumeration_handler, window_hwnds)
    
    
    print(len(window_hwnds))
    if len(window_hwnds)<2:
        print("RflySim3D窗口数小于2个，本程序无法正常运行")
        mav.stopRun()
        sys.exit(1)    

    #设置前置摄像头画面为720x405
    mav.sendUE4Cmd(b'r.setres 720x405w',1)    
    time.sleep(2)    
    
    #infoClass=[]
    Wd01 = window_hwnds[0]
    hasFoundWd = False
    for hwd in window_hwnds:
        info = getHwndInfo(hwd)
        if info.width == 720:
            Wd01 = hwd
            hasFoundWd = True
            window_hwnds.remove(hwd)
            break

    #没有找到
    if not hasFoundWd:
        print("第一个RflySim3D窗口不响应本Python程序的窗口改变指令，请重新运行本程序")
        mav.stopRun()
        sys.exit(1)    
    else:
        print("第一个RflySim3D窗口已经找到，并设置视景和分辨率")
        
    
    ImgInfo1 = getHwndInfo(Wd01)

    startTime = time.time()
    lastTime = time.time()
    timeInterval = 0.01
    flag = 0
    # #启用键盘
    mav.initOffboard()
    mav.keyInput()
    print("Simulation Start.")

    while True: 
        lastTime = lastTime + timeInterval
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()
        #以下代码0.01s执行一次

        #光流程序
        Optical_flow(mav)

        if mav.is_takeoff == True:  #key_a
            # mav.initOffboard()
            mav.SendMavArm(True) #解锁命令
            mav.SendPosNED(mav.uavPosNED[0], mav.uavPosNED[1], mav.uavPosNED[2] - 5, 0) #飞到目标点 0,0，-5位置
            # print("mav_state: {}".format(1))
        elif mav.is_Forward == True:   #key_b
            mav.SendVelFRD(2, 0, 0, 0)
            # print("mav_state: {}".format(2))
        elif mav.is_hover == True:      #key_c
            mav.SendPosNED(mav.uavPosNED[0], mav.uavPosNED[1], mav.uavPosNED[2], 0)
            # print("mav_state: {}".format(3))
        elif mav.is_land == True:       #key_d
            mav.SendPosNED(mav.uavPosNED[0], mav.uavPosNED[1], mav.uavPosNED[2] + 5, 0)
            mav.SendMavArm(False) #上锁命令
            # print("mav_state: {}".format(4))
        # k = cv2.waitKey(30)  # & 0xff
        # if k == 27:
        #     break
    # 结束，释放内存
    win32gui.DeleteObject(saveBitMap.GetHandle())
    saveDC.DeleteDC()
    mfcDC.DeleteDC()
    win32gui.ReleaseDC(hWnd,hWndDC)


        


    
    
    

