import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

# Global variables (current coordinates)
current_actual = [-1]
algorithm_queue = -1
enableStatus_robot = -1
robotErrorState = False
isBeaglebone = True
globalLockValue = threading.Lock()


def ConnectRobot():
    try:
        ip = "192.168.5.1"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("Establishing connection...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<Connection successful>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(Connection failed:(")
        raise e


def RunPoint(move: DobotApiMove, point_list: list,speedlparam):
    if isinstance(point_list, np.ndarray):
        # print("my_array is a numpy array")
        point_list = point_list.tolist()
    else:
        # print("my_array is not a numpy array")
    move.MovL(point_list[0], point_list[1], point_list[2],
              point_list[3], point_list[4], point_list[5],speedlparam)

def RunCircle(move: DobotApiMove, point_list1: list, point_list2: list, count,speedlparam,acclparam):
    if isinstance(point_list, np.ndarray):
        # print("my_array is a numpy array")
        point_list = point_list.tolist()
    else:
        # print("my_array is not a numpy array")
   move.Circle3(point_list1[0], point_list1[1], point_list1[2], point_list1[3], point_list1[4], point_list1[5],point_list2[0], point_list2[1], point_list2[2], point_list2[3], point_list2[4], point_list2[5],count,speedlparam,acclparam)     

def MarkCircle(move: DobotApiMove, point_list1: list, point_list2: list, count,speedlparam,acclparam):
    if isinstance(point_list, np.ndarray):
        # print("my_array is a numpy array")
        point_list = point_list.tolist()
    else:
        # print("my_array is not a numpy array")
   move.Circle3(point_list1[0], point_list1[1], point_list1[2], point_list1[3], point_list1[4], point_list1[5],point_list2[0], point_list2[1], point_list2[2], point_list2[3], point_list2[4], point_list2[5],count,speedlparam,acclparam)    



def GetFeed(feed: DobotApi):
    global current_actual
    global algorithm_queue
    global enableStatus_robot
    global robotErrorState
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            globalLockValue.acquire()
            # Refresh Properties
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['run_queued_cmd'][0]
            enableStatus_robot = feedInfo['enable_status'][0]
            robotErrorState = feedInfo['error_status'][0]
            globalLockValue.release()
        sleep(0.001)


def WaitArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)

def MarkTilArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)

def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState
    dataController, dataServo = alarmAlarmJsonFile()    # Read controller and servo alarm codes
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if (numbers[0] == 0):
                if (len(numbers) > 1):
                    for i in numbers[1:]:
                        alarmState = False
                        if i == -2:
                            print("Machine warning: Machine collision", i)
                            alarmState = True
                        if alarmState:
                            continue
                        for item in dataController:
                            if i == item["id"]:
                                print("Machine alarm Controller errorid", i,
                                      item["zh_CN"]["description"])
                                alarmState = True
                                break
                        if alarmState:
                            continue
                        for item in dataServo:
                            if i == item["id"]:
                                print("Machine alarm Servo errorid", i,
                                      item["zh_CN"]["description"])
                                break

                    choose = input("Enter 1 to clear the error and the machine will continue to run: ")
                    if int(choose) == 1:
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()

        else:
            if int(enableStatus_robot) == 1 and int(algorithm_queue) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)


if __name__ == '__main__':
    if isBeaglebone:
        import Adafruit_BBIO.PWM as PWM

    else:
        print("isBeaglebone setting is False. The laser will not mark")

    dashboard, move, feed = ConnectRobot()
    feed_thread = threading.Thread(target=GetFeed, args=(feed,))
    feed_thread.daemon = True
    feed_thread.start()
    feed_thread1 = threading.Thread(target=ClearRobotError, args=(dashboard,))
    feed_thread1.daemon = True
    feed_thread1.start()
    print("Start enabling...")
    dashboard.EnableRobot()
    
    print("Complete enabling:)")
    print("Loop execution...")

    dashboard.SpeedFactor(10)
    dashboard.SetCollisionLevel(5)
    
    
    markingHeight = 486.31

    # circle 1
    point_c11 = np.array([-22, 38, markingHeight, 90, 0, 30])
    point_c12 = point_c11 + [-2, 2, 0, 0, 0, 0]
    point_c13 = point_c11 + [0, 1, 0, 0, 0, 0]
    
    
    ## 0.5 from each side
    point_c21 = np.array([-22.5, 38.5, markingHeight, 90, 0, 30])
    point_c22 = point_c21 + [-1, 1, 0, 0, 0, 0]
    point_c23 = point_c21 + [0, 0.5, 0, 0, 0, 0]
    
    
    ## 0.25 from each side
    point_c31 = np.array([-22.75, 38.75, markingHeight, 90, 0, 30])
    point_c32 = point_c31 + [-0.5, 0.5, 0, 0, 0, 0]
    point_c33 = point_c31 + [0, 0.25, 0, 0, 0, 0]
    
    
    point_init = point_c11 + [0, 0, markingHeight - 26, 0, 0, 0]
    point_end = point_c31 + [0, 0, markingHeight - 26, 0, 0, 0]


    runCount = 0

    while True:
        if runCount < 1:
            RunPoint(move, point_init,"SpeedL=100")
            WaitArrive(point_init)

            ## circle 1
            RunPoint(move, point_c11,"SpeedL=100")
            WaitArrive(point_c11)
            if isBeaglebone:
                PWM.start("P9_14",30,1000)
            else:
                print('PWM.start("P9_14",30,1000)')
            
            RunCircle(move, point_c12,point_c13,1,"SpeedL=1","AccL=1")
            WaitArrive(point_c12)
            WaitArrive(point_c11)

            if isBeaglebone:
                PWM.stop("P9_14")
            else:
                print('PWM.stop("P9_14")')

            sleep(2)
            
            ## circle 2
            RunPoint(move, point_c21,"SpeedL=100")
            WaitArrive(point_c21)
            if isBeaglebone:
                PWM.start("P9_14",30,1000)
            else:
                print('PWM.start("P9_14",30,1000)')

            RunCircle(move, point_c22,point_c23,1,"SpeedL=1","AccL=1")
            WaitArrive(point_c22)
            WaitArrive(point_c21)

            if isBeaglebone:
                PWM.stop("P9_14")
            else:
                print('PWM.stop("P9_14")')

            ## circle 3
            """
            RunPoint(move, point_c31,"SpeedL=100")
            WaitArrive(point_c31)
            
            if isBeaglebone:
                PWM.start("P9_14",30,1000)
            else:
                print('PWM.start("P9_14",30,1000)')
            
            RunCircle(move, point_c32,point_c33,1,"SpeedL=1","AccL=1")
            WaitArrive(point_c32)
            WaitArrive(point_c31)
            if isBeaglebone:
                PWM.stop()
            else:
                print('PWM.stop()')
            """

            RunPoint(move,point_end,"SpeedL=100")
            PWM.cleanup()
            runCount = 2
