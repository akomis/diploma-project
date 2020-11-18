import threading
import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#将dll读取到内存中并获取对应的CDLL实例
#Load Dll and get the CDLL object
api = dType.load()
#建立与dobot的连接
#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    print("Pose: " + str(dType.GetPose(api)))
    print("Kinematics: " + str(dType.GetKinematics(api)))
    print("Alarms State: " + str(dType.GetAlarmsState(api)[1]))
    print("HHTTrigMode: " + str(dType.GetHHTTrigMode(api)[0]))
    print("Arm Orientation: " + str(dType.GetArmOrientation(api)[0]))
    print("JOG Joint Params: " + str(dType.GetJOGJointParams(api)))
    print("JOG Coord Params: " + str(dType.GetJOGCoordinateParams(api)))
    print("JOG Common Params: " + str(dType.GetJOGCommonParams(api)))
    print("PTP Joint Params: " + str(dType.GetPTPJointParams(api)))
    print("PTP Coord Params: " + str(dType.GetPTPCoordinateParams(api)))
    print("PTP Jump Params: " + str(dType.GetPTPJumpParams(api)))
    print("PTP Common Params: " + str(dType.GetPTPCommonParams(api)))
    test = 0
    print("IO Mult: " + " " + str(dType.GetIOMultiplexing(api, test)))

#断开连接
#Disconnect Dobot
dType.DisconnectDobot(api)
