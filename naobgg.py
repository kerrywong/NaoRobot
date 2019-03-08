# -*- coding: UTF-8 -*-
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser
import time
import math
import os
import threading
import inspect
import ctypes


def stop_nao(robot_ip, port):
    '''
    结束 NAO 当前打球进程
    :param robot_ip: 机器人 ip 地址
    :param port: 端口
    '''
    to_stop = ALProxy("ALMemory", robot_ip, port)
    while True:
        stop = to_stop.getData("RearTactilTouched")
        if stop == 1.0:
            hdzzmotionProxy = ALProxy("ALMotion", robot_ip, port)
            hdzzmotionProxy.openHand('RHand')
            time.sleep(1.0)
            print 'time sleep'
            hdzzmotionProxy.closeHand('RHand')
            hdzzmotionProxy.rest()
            stop_thread(t)


def _async_raise(tid, exctype):
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def recycle_club(robot_ip, port):
    '''
    收杆
    '''
    rcmotionProxy = ALProxy("ALMotion", robot_ip, port)
    rcmotionProxy.angleInterpolationWithSpeed('RShoulderRoll', -math.pi / 4, 0.2)
    rcmotionProxy.angleInterpolationWithSpeed('RWristYaw', 0, 0.2)
    rcmotionProxy.angleInterpolationWithSpeed('RElbowRoll', math.pi / 36, 0.2)
    rcmotionProxy.angleInterpolationWithSpeed('RShoulderPitch', math.pi * 85 / 180, 0.2)
    rcmotionProxy.angleInterpolationWithSpeed('RShoulderRoll', -math.pi / 9, 0.2)
    rcmotionProxy.angleInterpolationWithSpeed('RElbowYaw', math.pi *95 / 180, 0.2)


def nao_start(robot_ip, port, parameter):
    start_say = ALProxy("ALTextToSpeech", robot_ip, port)
    start_say.say("开始打球")
    nsmotion_proxy = ALProxy("ALMotion", robot_ip, port)
    nsposture_proxy = ALProxy("ALRobotPosture", robot_ip, port)
    nsmemory_proxy = ALProxy("ALMemory", robot_ip, port)
    nstts = ALProxy("ALTextToSpeech", robot_ip, port)

    nsmotion_proxy.wakeUp()
    nsposture_proxy.goToPosture("Stand", 0.8)

    names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowRoll', 'RElbowYaw', 'RWristYaw', 'RHand']
    targetAngles = [math.pi * 7 / 18, 0.0, math.pi * 7 / 18, math.pi / 2, 0.0, 1.0]
    nsmotion_proxy.angleInterpolationWithSpeed(names, targetAngles, 0.2)

    names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowRoll', 'LElbowYaw', 'LWristYaw']
    targetAngles = [math.pi * 80 / 180, math.pi *15 / 180, -math.pi *75 / 180, -math.pi * 55 / 180, 0]
    nsmotion_proxy.angleInterpolationWithSpeed(names, targetAngles, 0.2)

    while True:
        nsbook = 0
        #等待球杆
        while True:
            touched_flag_0 = nsmemory_proxy.getData("FrontTactilTouched")
            if touched_flag_0 == 1.0:
                touched_flag_0 = 0.0
                nstts.say("谢谢")
                break

        time.sleep(3.0)                                                 # 抓杆时停顿时间
        nsmotion_proxy.angleInterpolationWithSpeed('RHand', 0.2, 0.2)  # 抓杆，可调抓杆力度

        while True:
            touched_flag_1 = nsmemory_proxy.getData("FrontTactilTouched")      #开始打球
            touched_flag_2 = nsmemory_proxy.getData("MiddleTactilTouched")     #重新抓杆
            if touched_flag_1 == 1.0:
                nstts.say("我准备好了")
                break
            elif touched_flag_2 == 1.0:
                nsbook = 1
                break

        if nsbook == 1:
            nsmotion_proxy.angleInterpolationWithSpeed('RHand', 1.0, 0.2)
        else:
            break

    #击球
    nsmotion_proxy.angleInterpolationWithSpeed('RWristYaw', math.pi / 6, 0.2)
    time.sleep(2.0)
    nsmotion_proxy.angleInterpolationWithSpeed('RWristYaw', -math.pi / 6, parameter['hit_power'])
    time.sleep(2.0)
    recycle_club(robot_ip, port)


    # 击球后 前进、旋转、前进
    nsmotion_proxy.setMoveArmsEnabled(False, False)
    nsmotion_proxy.moveTo(parameter['first_walk'], 0.0, 0.0, [["MaxStepFrequency", 0.8]])  # 调整参数设置击球后前进距离
    # 旋转90度
    nsmotion_proxy.moveTo(0.0, 0.0, -math.pi / 2, parameter['nao_walk'])
    parameter['all_angle'] -= math.pi / 2
    # 前进
    nsmotion_proxy.moveTo(parameter['first_step'], 0.0, 0.0, parameter['nao_walk'])  # 调整参数设置前进距离
    #矫正
    nsmotion_proxy.moveTo(0, 0, parameter['straight_angle_a'], parameter['nao_walk'])
    nsmotion_proxy.moveTo(0, parameter['straight_y_a'], 0, parameter['nao_walk'])


def find_search_red_ball(robot_ip, port, parameter):

    fsrbcam_proxy = ALProxy("ALVideoDevice", robot_ip, port)
    fsrbredball_proxy = ALProxy("ALRedBallDetection", robot_ip, port)
    fsrbmotion_proxy = ALProxy("ALMotion", robot_ip, port)
    fsrbmemory_proxy = ALProxy("ALMemory", robot_ip, port)
    fsrbtts = ALProxy("ALTextToSpeech", robot_ip, port)

    head_angle = 60

    fsrbcam_proxy.setActiveCamera(1)
    fsrbredball_proxy.subscribe("redBallDetected")

    time_book = None

    for u in range(0, 5):
        fsrbmotion_proxy.angleInterpolationWithSpeed("HeadYaw", (head_angle - u * 30) * math.pi / 180, 0.2)
        fsrbmotion_proxy.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.2)
        time.sleep(2.0)

        first_value = fsrbmemory_proxy.getData("redBallDetected")
        if first_value:
            time_book = first_value[0]
        time.sleep(1.0)

        for i in range(3):
            time.sleep(1.0)
            ball_data = fsrbmemory_proxy.getData("redBallDetected")
            ball_angle = fsrbmotion_proxy.getAngles("HeadYaw", True)

        if time_book:
            if ball_data[0] == time_book:
                fsrbtts.say("红球不在此处")
            else:
                fsrbtts.say("发现红球")
                the_result = [ball_angle, ball_data[1], 0]
                fsrbredball_proxy.unsubscribe("redBallDetected")
                return the_result

        else:
            if ball_data:
                fsrbtts.say("发现红球")
                the_result = [ball_angle, ball_data[1], 0]
                fsrbredball_proxy.unsubscribe("redBallDetected")
                return the_result
            else:
                fsrbtts.say("红球不在此处")

    fsrbredball_proxy.unsubscribe("redBallDetected")
    fsrbtts.say("前进继续找红球")
    print '前进'
    fsrbmotion_proxy.moveTo(parameter['nao_step'], 0.0, 0.0, parameter['nao_walk'])                #此处可修改参数，每次找球前进的距离
    fsrbmotion_proxy.moveTo(0, 0, parameter['straight_angle_b'], parameter['nao_walk'])   #矫正
    fsrbmotion_proxy.moveTo(0, parameter['straight_y_b'], 0, parameter['nao_walk'])  # 矫正
    return [0,0,1]


def calculate_robot_to_redball(Allball_data, robot_ip, port, parameter):

    crtrmotion_proxy = ALProxy("ALMotion", robot_ip, port)
    crtrmemory_proxy = ALProxy("ALMemory", robot_ip, port)
    crtrredball_proxy = ALProxy("ALRedBallDetection", robot_ip, port)
    crtrcam_proxy = ALProxy("ALVideoDevice", robot_ip, port)


    crtrcam_proxy.setActiveCamera(1)

    head_angle = Allball_data[0]       #头偏转角
    wz_camera = Allball_data[1][0]  #alpha角

    #-------------------- 第一次，转向，正对红球 ------------------------------

    crtrmotion_proxy.moveTo(0, 0, head_angle[0] + wz_camera, parameter['nao_walk'])
    parameter['all_angle'] += head_angle[0] + wz_camera

    #------------------------第二次，机器人走到距离红球30厘米的位置----------------------
    crtrmotion_proxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2)
    crtrmotion_proxy.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.2)

    crtrredball_proxy.subscribe("redBallDetected")
    time.sleep(2.0)
    for i in range(0, 3):
        val = crtrmemory_proxy.getData("redBallDetected")
        valHeadYaw = crtrmotion_proxy.getAngles("HeadYaw",True)
        time.sleep(1.0)

    ballinfo = val[1]

    thetah = ballinfo[0] + valHeadYaw[0]
    thetav = ballinfo[1]+(39.7*math.pi/180.0)           #crf 修改参数

    x = parameter['nao_red_high'] / (math.tan(thetav)) - 0.3
    y = 0

    crtrmotion_proxy.moveTo(x, y, thetah, parameter['nao_walk'])
    parameter['all_angle'] += thetah

    #---------------------------第三次，机器人修正角度对准红球--------------------
    crtrmotion_proxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2)
    crtrmotion_proxy.angleInterpolationWithSpeed("HeadPitch", 30*math.pi/180.0, 0.2)

    time.sleep(2.0)
    for i in range(0,3):
        val = crtrmemory_proxy.getData("redBallDetected")
        valHeadYaw = crtrmotion_proxy.getAngles("HeadYaw",True)
        time.sleep(1.0)

    ballinfo = val[1]
    thetah = ballinfo[0] + valHeadYaw[0]

    crtrmotion_proxy.moveTo(0, 0, thetah, parameter['nao_walk'])
    parameter['all_angle'] += thetah

    #------------------------第四次，机器人走到距离红球10厘米的位置----------------------
    time.sleep(2.0)
    for i in range(0,3):
        val = crtrmemory_proxy.getData("redBallDetected")
        valHeadYaw = crtrmotion_proxy.getAngles("HeadYaw",True)
        time.sleep(1.0)
    ballinfo = val[1]

    thetah = ballinfo[0] + valHeadYaw[0]
    thetav = ballinfo[1]+(69.7*math.pi/180.0)
    x = (parameter['nao_red_high']-0.03)/(math.tan(thetav)) - 0.07           #crf 调整参数
    y = 0.0
    crtrmotion_proxy.moveTo(x, y, thetah, parameter['nao_walk'])
    parameter['all_angle'] += thetah

    #---------------------------第五次，计算出红球距机器人的距离-------------------
    time.sleep(2.0)
    for i in range(0,3):
        val = crtrmemory_proxy.getData("redBallDetected")
        time.sleep(1.0)
    thetav = val[1][1]+(69.7*math.pi/180.0)         #crf 调整参数
    dx = (parameter['nao_red_high']-0.03)/(math.tan(thetav))   #dx作为了三角形的一条边
    crtrredball_proxy.unsubscribe("redBallDetected")
    return dx


def first_search_nao_mark(robot_ip, port, parameter):
    fsnmtts = ALProxy("ALTextToSpeech", robot_ip, port)
    fsnmmemory_proxy = ALProxy("ALMemory", robot_ip, port)
    fsnmcam_proxy = ALProxy("ALVideoDevice", robot_ip, port)
    fsnmlandmark_proxy = ALProxy("ALLandMarkDetection", robot_ip, port)
    fsnmmotion_proxy = ALProxy("ALMotion", robot_ip, port)

    head_yaw_angle = -math.pi / 2
    fsnmcam_proxy.setActiveCamera(0)

    fsnmlandmark_proxy.subscribe("landmarkTest")
    while (head_yaw_angle <= math.pi / 2):
        fsnmmotion_proxy.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
        fsnmmotion_proxy.angleInterpolationWithSpeed("HeadYaw", head_yaw_angle, 0.3)

        time.sleep(2.0)
        for i in range(0,2):
            mark_data = fsnmmemory_proxy.getData("LandmarkDetected")
            time.sleep(1.0)

        if(mark_data and isinstance(mark_data, list) and len(mark_data) >= 2):
            fsnmtts.say("找到球洞")
            markwz_camera = mark_data[1][0][0][1]
            markangularSize = mark_data[1][0][0][3]
            head_angle = fsnmmotion_proxy.getAngles("HeadYaw",True)
            markhead_angle = markwz_camera + head_angle[0]
            distanceFromCameraToLandmark = parameter['nao_mark_size'] / (2 * math.tan(markangularSize / 2))
            allmark_data = [markhead_angle, distanceFromCameraToLandmark, 0]
            fsnmlandmark_proxy.unsubscribe("landmarkTest")
            return allmark_data

        else:
            fsnmtts.say("此处没有球洞")

        head_yaw_angle = head_yaw_angle + 30 * math.pi / 180
    fsnmtts.say("查找球洞失败")
    allmark_data = [0,0,1]
    fsnmlandmark_proxy.unsubscribe("landmarkTest")
    return allmark_data


def triangle_calculation(x,s,alpha, parameter):

    x += parameter['add_all']   #机器人到红球距离
    if s == 9999:
        alpha = -alpha
    else:
        s += parameter['add_all']   #机器人到球洞距离

    if (alpha <= 0.0):
        alpha = abs(alpha)
        if s == 9999:
            theta = math.pi - alpha
        else:
            l2 = x*x + s*s - 2*x*s*math.cos(alpha)      #球与球洞距离的平方
            l = math.sqrt(l2)
            costheta = (x*x + l2 - s*s)/(2*x*l)     #红球顶点角
            theta = math.acos(costheta)

        if theta >= math.pi/2:
            turn_angle = theta - math.pi/2
            disy = -(x*math.sin(turn_angle) - parameter['adjustment_y'])    #向右多走
            disx = x*math.cos(turn_angle) - parameter['adjustment_x']       #向前少走

        elif theta < math.pi/2:
            turn_angle = math.pi/2 - theta
            disx = x*math.cos(turn_angle) - parameter['adjustment_x']       #向前少走
            disy = x*math.sin(turn_angle) + parameter['adjustment_y']       #向左多走
            turn_angle = -turn_angle

        return [disy,disx,turn_angle, 0]

    elif (alpha > 0.0):
        if s == 9999:
            theta = math.pi - alpha
        else:
            l2 = x*x + s*s - 2*x*s*math.cos(alpha)
            l = math.sqrt(l2)
            costheta = (x*x + l2 - s*s)/(2*x*l)
            theta = math.acos(costheta)

        if theta >= math.pi/2:
            turn_angle = theta - math.pi/2
            disy = x*math.sin(turn_angle) + parameter['adjustment_y'] - parameter['move_robot_y']    #向左多走   左边球洞偏移
            disx = x*math.cos(turn_angle) - parameter['adjustment_x'] - parameter['move_robot_x']    #向前少走   左边球洞偏移
            turn_angle = -turn_angle

        elif theta < math.pi/2:
            turn_angle = math.pi/2 - theta
            disx = x*math.cos(turn_angle)- parameter['adjustment_x']        #向前少走
            disy = -(x*math.sin(turn_angle) - parameter['adjustment_y'])        #向右少走

        return [disy,disx,turn_angle, 1]


def hit_ball(hbbook, robot_ip, port, parameter):
    hbmotion_proxy = ALProxy("ALMotion", robot_ip, port)

    hbmotion_proxy.angleInterpolationWithSpeed('RShoulderRoll', -math.pi / 4, 0.2)
    hbmotion_proxy.angleInterpolationWithSpeed('RShoulderPitch', math.pi / 6, 0.2)
    hbmotion_proxy.angleInterpolationWithSpeed('RElbowRoll', math.pi * 7 / 18, 0.2)
    hbmotion_proxy.angleInterpolationWithSpeed('RShoulderRoll', 0, 0.2)
    if hbbook:
        hbmotion_proxy.angleInterpolationWithSpeed('RWristYaw', -math.pi / 6, 0.2)
    else:
        hbmotion_proxy.angleInterpolationWithSpeed('RWristYaw', math.pi / 6, 0.2)
    hbmotion_proxy.angleInterpolationWithSpeed('RElbowYaw', math.pi / 2, 0.2)
    hbmotion_proxy.angleInterpolationWithSpeed('RShoulderPitch', math.pi * 7 / 18, 0.2)
    hbmotion_proxy.moveTo(-0.04, 0.0, 0.0, parameter['nao_walk'])
    time.sleep(2.0)
    #击球
    if hbbook:
        hbmotion_proxy.angleInterpolationWithSpeed('RWristYaw', math.pi / 6, parameter['all_hit_power'])
    else:
        hbmotion_proxy.angleInterpolationWithSpeed('RWristYaw', -math.pi / 6, parameter['all_hit_power'])
    time.sleep(2.0)
    #收杆
    recycle_club(robot_ip, port)


def ball_one(robot_ip, port):
    all_angle = math.pi/2       # 记录旋转的角度
    hit_power = 0.14     # 第一次击球力度，0 - 1越大力度越强
    all_hit_power = 0.13   # 以后击球力度，0 - 1越大力度越强
    no_nao_mark_power = 0.07 # 没找到球洞的击球力度
    first_walk = 0.15    # 击球后第一次行走的距离
    first_step = 1.2     # 第一次击球后第二次行走的距离
    nao_step = 0.30      # 每次前进的距离
    nao_red_high = 0.45      # 机器人识别红球高度
    nao_mark_size = 0.11     # NaoMark大小，单位米
    adjustment_x = 0.17       # 形成直角的微调距离（绝对值）
    adjustment_y = 0.09      # 形成直角的微调距离（绝对值）
    add_all = 0.15           # 红球测距原点与旋转点距离
    nao_walk = [["MaxStepFrequency", 0.65]]    #机器人行走速度调节

    straight_angle_a = 0     #击球后第一次行走矫正距离
    straight_y_a = 0

    straight_angle_b = 0     #找球时矫正距离
    straight_y_b = 0

    move_robot_x = 0     # 左边球洞偏移
    move_robot_y = 0     # 左边球洞偏移

    mmotionProxy = ALProxy("ALMotion", robot_ip, port)

    parameter = {'all_angle': all_angle,
                 'hit_power': hit_power,
                 'all_hit_power': all_hit_power,
                 'first_walk': first_walk,
                 'first_step': first_step,
                 'nao_step': nao_step,
                 'nao_red_high': nao_red_high,
                 'nao_mark_size': nao_mark_size,
                 'adjustment_x': adjustment_x,
                 'adjustment_y': adjustment_y,
                 'add_all': add_all,
                 'nao_walk': nao_walk,
                 'straight_angle_a': straight_angle_a,
                 'straight_y_a': straight_y_a,
                 'straight_angle_b': straight_angle_b,
                 'straight_y_b': straight_y_b,
                 'move_robot_x': move_robot_x,
                 'move_robot_y': move_robot_y}

    nao_start(robot_ip, port, parameter)

    my_all_ball_date = None
    while True:
        while True:
            my_all_ball_date = find_search_red_ball(robot_ip, port, parameter)   #红球定位 返回红球位置信息
            if (my_all_ball_date[2] == 0):
                break

        x = calculate_robot_to_redball(my_all_ball_date, robot_ip, port, parameter)
        my_nao_mark_date = first_search_nao_mark(robot_ip, port, parameter)
        adjustment = None

        no_nao_mark = None
        if my_nao_mark_date[2] == 0:
            adjustment = triangle_calculation(x, my_nao_mark_date[1], my_nao_mark_date[0], parameter)
            no_nao_mark = False
        else:
            adjustment = triangle_calculation(x, 9999, parameter['all_angle'], parameter)
            no_nao_mark = True

        #旋转合适角度，使球、洞、机器人构成一个直角
        mmotionProxy.moveTo(0.0, 0.0, adjustment[2], parameter['nao_walk'])
        parameter['all_angle'] += adjustment[2]
        #调整机器人x坐标
        mmotionProxy.moveTo(adjustment[1], 0.0, 0.0, parameter['nao_walk'])
        #调整机器人y坐标
        mmotionProxy.moveTo(0.0, adjustment[0], 0.0, parameter['nao_walk'])
        #击球
        if no_nao_mark is True:
            hit_ball(adjustment[3], robot_ip, port, {'all_hit_power': no_nao_mark_power, 'nao_walk': [["MaxStepFrequency", 0.65]]})
        else:
            hit_ball(adjustment[3], robot_ip, port, parameter)
        #击球后 旋转、前进
        #旋转90度
        if adjustment[3]:
            mmotionProxy.moveTo(0.0, 0.0, math.pi/2, parameter['nao_walk'])
            parameter['all_angle'] += math.pi/2
        else:
            mmotionProxy.moveTo(0.0, 0.0, -math.pi/2, parameter['nao_walk'])
            parameter['all_angle'] -= math.pi/2
        #前进
        mmotionProxy.moveTo(parameter['nao_step'], 0.0, 0.0, parameter['nao_walk'])          #调整参数设置前进距离
        mmotionProxy.moveTo(0, 0, parameter['straight_angle_b'], parameter['nao_walk'])   #矫正
        mmotionProxy.moveTo(0, parameter['straight_y_b'], 0, parameter['nao_walk'])  # 矫正


def ball_two(robot_ip, port):
    all_angle = math.pi / 6  # 记录旋转的角度
    hit_power = 0.17  # 第一次击球力度，0 - 1越大力度越强
    all_hit_power = 0.13  # 以后击球力度，0 - 1越大力度越强
    no_nao_mark_power = 0.07  # 没找到球洞的击球力度
    first_walk = 0.15  # 击球后第一次行走的距离
    first_step = 1.7  # 第一次击球后第二次行走的距离
    nao_step = 0.30  # 每次前进的距离
    nao_red_high = 0.45  # 机器人识别红球高度
    nao_mark_size = 0.11  # NaoMark大小，单位米
    adjustment_x = 0.17  # 形成直角的微调距离（绝对值）
    adjustment_y = 0.09  # 形成直角的微调距离（绝对值）
    add_all = 0.15  # 红球测距原点与旋转点距离
    nao_walk = [["MaxStepFrequency", 0.65]]  # 机器人行走速度调节

    straight_angle_a = 0  # 击球后第一次行走矫正距离
    straight_y_a = 0

    straight_angle_b = 0  # 找球时矫正距离
    straight_y_b = 0

    move_robot_x = 0  # 左边球洞偏移
    move_robot_y = 0  # 左边球洞偏移

    mmotionProxy = ALProxy("ALMotion", robot_ip, port)

    parameter = {'all_angle': all_angle,
                 'hit_power': hit_power,
                 'all_hit_power': all_hit_power,
                 'first_walk': first_walk,
                 'first_step': first_step,
                 'nao_step': nao_step,
                 'nao_red_high': nao_red_high,
                 'nao_mark_size': nao_mark_size,
                 'adjustment_x': adjustment_x,
                 'adjustment_y': adjustment_y,
                 'add_all': add_all,
                 'nao_walk': nao_walk,
                 'straight_angle_a': straight_angle_a,
                 'straight_y_a': straight_y_a,
                 'straight_angle_b': straight_angle_b,
                 'straight_y_b': straight_y_b,
                 'move_robot_x': move_robot_x,
                 'move_robot_y': move_robot_y}

    nao_start(robot_ip, port, parameter)

    my_all_ball_date = None
    while True:
        while True:
            my_all_ball_date = find_search_red_ball(robot_ip, port, parameter)  # 红球定位 返回红球位置信息
            if (my_all_ball_date[2] == 0):
                break

        x = calculate_robot_to_redball(my_all_ball_date, robot_ip, port, parameter)
        my_nao_mark_date = first_search_nao_mark(robot_ip, port, parameter)
        adjustment = None

        no_nao_mark = None
        if my_nao_mark_date[2] == 0:
            adjustment = triangle_calculation(x, my_nao_mark_date[1], my_nao_mark_date[0], parameter)
            no_nao_mark = False
        else:
            adjustment = triangle_calculation(x, 9999, parameter['all_angle'], parameter)
            no_nao_mark = True

        # 旋转合适角度，使球、洞、机器人构成一个直角
        mmotionProxy.moveTo(0.0, 0.0, adjustment[2], parameter['nao_walk'])
        parameter['all_angle'] += adjustment[2]
        # 调整机器人x坐标
        mmotionProxy.moveTo(adjustment[1], 0.0, 0.0, parameter['nao_walk'])
        # 调整机器人y坐标
        mmotionProxy.moveTo(0.0, adjustment[0], 0.0, parameter['nao_walk'])
        # 击球
        if no_nao_mark is True:
            hit_ball(adjustment[3], robot_ip, port, {'all_hit_power': no_nao_mark_power})
        else:
            hit_ball(adjustment[3], robot_ip, port, parameter)
        # 击球后 旋转、前进
        # 旋转90度
        if adjustment[3]:
            mmotionProxy.moveTo(0.0, 0.0, math.pi / 2, parameter['nao_walk'])
            parameter['all_angle'] += math.pi / 2
        else:
            mmotionProxy.moveTo(0.0, 0.0, -math.pi / 2, parameter['nao_walk'])
            parameter['all_angle'] -= math.pi / 2
        # 前进
        mmotionProxy.moveTo(parameter['nao_step'], 0.0, 0.0, parameter['nao_walk'])  # 调整参数设置前进距离
        mmotionProxy.moveTo(0, 0, parameter['straight_angle_b'], parameter['nao_walk'])  # 矫正
        mmotionProxy.moveTo(0, parameter['straight_y_b'], 0, parameter['nao_walk'])  # 矫正


def ball_three(robot_ip, port):
    all_angle = math.pi       #记录旋转的角度
    hit_power = 0.13      #第一次击球力度，0 - 1越大力度越强
    all_hit_power = 0.11   #以后击球力度，0 - 1越大力度越强
    first_walk = 0.15    #击球后第一次行走的距离
    first_step = 0.6     #第一次击球后第二次行走的距离
    nao_step = 0.15       #每次前进的距离
    nao_red_high = 0.45      #机器人识别红球高度
    nao_mark_size = 0.11     #NaoMark大小，单位米
    adjustment_x = 0.17       #形成直角的微调距离（绝对值）
    adjustment_y = 0.09      #形成直角的微调距离（绝对值）
    add_all = 0.15           #红球测距原点与旋转点距离
    nao_walk = [["MaxStepFrequency", 0.65]]    #机器人行走速度调节

    straight_angle_a = 0     #击球后第一次行走矫正距离
    straight_y_a = 0

    straight_angle_b = 0     #找球时矫正距离
    straight_y_b = 0

    move_robot_x = 0.02     #左边球洞偏移
    move_robot_y = 0.03     #左边球洞偏移

    mmotionProxy = ALProxy("ALMotion", robot_ip, port)

    parameter = {'all_angle': all_angle,
                 'hit_power': hit_power,
                 'all_hit_power': all_hit_power,
                 'first_walk': first_walk,
                 'first_step': first_step,
                 'nao_step': nao_step,
                 'nao_red_high': nao_red_high,
                 'nao_mark_size': nao_mark_size,
                 'adjustment_x': adjustment_x,
                 'adjustment_y': adjustment_y,
                 'add_all': add_all,
                 'nao_walk': nao_walk,
                 'straight_angle_a': straight_angle_a,
                 'straight_y_a': straight_y_a,
                 'straight_angle_b': straight_angle_b,
                 'straight_y_b': straight_y_b,
                 'move_robot_x': move_robot_x,
                 'move_robot_y': move_robot_y}

    nao_start(robot_ip, port, parameter)

    my_all_ball_date = None
    while True:
        while True:
            my_all_ball_date = find_search_red_ball(robot_ip, port, parameter)   #红球定位 返回红球位置信息
            if (my_all_ball_date[2] == 0):
                break

        x = calculate_robot_to_redball(my_all_ball_date, robot_ip, port, parameter)
        my_nao_mark_date = first_search_nao_mark(robot_ip, port, parameter)
        adjustment = None

        if my_nao_mark_date[2] == 0:
            adjustment = triangle_calculation(x, my_nao_mark_date[1], my_nao_mark_date[0], parameter)
        else:
            adjustment = triangle_calculation(x, 9999, parameter['all_angle'], parameter)

        #旋转合适角度，使球、洞、机器人构成一个直角
        mmotionProxy.moveTo(0.0, 0.0, adjustment[2], parameter['nao_walk'])
        parameter['all_angle'] += adjustment[2]
        #调整机器人x坐标
        mmotionProxy.moveTo(adjustment[1], 0.0, 0.0, parameter['nao_walk'])
        #调整机器人y坐标
        mmotionProxy.moveTo(0.0, adjustment[0], 0.0, parameter['nao_walk'])
        #击球
        hit_ball(adjustment[3], robot_ip, port, parameter)
        #击球后 旋转、前进
        #旋转90度
        if adjustment[3]:
            mmotionProxy.moveTo(0.0, 0.0, math.pi/2, parameter['nao_walk'])
            parameter['all_angle'] += math.pi/2
        else:
            mmotionProxy.moveTo(0.0, 0.0, -math.pi/2, parameter['nao_walk'])
            parameter['all_angle'] -= math.pi/2
        #前进
        mmotionProxy.moveTo(parameter['nao_step'], 0.0, 0.0, parameter['nao_walk'])          #调整参数设置前进距离
        mmotionProxy.moveTo(0, parameter['straight_y_b'], parameter['straight_angle_b'], parameter['nao_walk'])   #矫正


def main(robot_ip, port):
    choose_ball = ALProxy("ALMemory", robot_ip, port)

    while True:
        ball_a = choose_ball.getData("FrontTactilTouched")
        ball_b = choose_ball.getData("MiddleTactilTouched")
        ball_c = choose_ball.getData("HandRightBackTouched")
        if ball_a:
            ball_one(robot_ip, port)
        elif ball_b:
            ball_two(robot_ip, port)
        elif ball_c:
            ball_three(robot_ip, port)


if __name__ == '__main__':
    port = 9559  # crf 机器人端口
    robot_ip = "192.168.43.247"  # crf 机器人IP

    s = threading.Thread(name='stop_nao', target=stop_nao, args=(robot_ip, port))
    s.daemon = True
    s.start()
    while True:
        t = threading.Thread(target=main, args=(robot_ip, port))
        t.start()
        t.join()
