import sys
import time
import serial
import webbrowser
import configparser
import DobotDllType as dType
from prometheus_client import start_http_server, Info, Gauge, Enum

config = configparser.ConfigParser()

class DobotMagician():
    def __init__(self, api, port):
        global config

        self.__api = api
        self.__port = port
        dobotSection = config['DOBOT' + ':' + self.__port]

        enabledDeviceInfo = {}
        if dobotSection.getboolean('DeviceSN', fallback=True):
            enabledDeviceInfo["serial_number"] = dType.GetDeviceSN(self.__api)[0]
        if dobotSection.getboolean('DeviceName', fallback=True):
            enabledDeviceInfo["device_name"] = dType.GetDeviceName(self.__api)[0]
        if dobotSection.getboolean('DeviceVersion', fallback=True):
            enabledDeviceInfo["version"] = '.'.join(list(map(str, dType.GetDeviceVersion(self.__api))))
        if len(enabledDeviceInfo) > 0:
            self.__deviceInfo = Info('dobot_magician', 'General information about monitored Dobot Magician device', ['device'])
            self.__deviceInfo.labels('dobot_'+self.__port).info(enabledDeviceInfo)

        enabledWifiInfo = {}
        if dobotSection.getboolean('WifiSSID', fallback=False):
            enabledWifiInfo["ssid"] = dType.GetWIFISSID(self.__api)[0]
        if dobotSection.getboolean('WifiPassword', fallback=False):
            enabledWifiInfo["password"] = dType.GetWIFIPassword(self.__api)[0]
        if dobotSection.getboolean('WifiIPAddress', fallback=False):
            enabledWifiInfo["ip_address"] = '.'.join(list(map(str, dType.GetWIFIIPAddress(self.__api)[1:])))
        if dobotSection.getboolean('WifiNetmask', fallback=False):
            enabledWifiInfo["netmask"] = '.'.join(list(map(str, dType.GetWIFINetmask(self.__api))))
        if dobotSection.getboolean('WifiGateway', fallback=False):
            enabledWifiInfo["gateway"] = '.'.join(list(map(str, dType.GetWIFIGateway(self.__api))))
        if dobotSection.getboolean('WifiDNS', fallback=False):
            enabledWifiInfo["dns"] = '.'.join(list(map(str, dType.GetWIFIDNS(self.__api))))
        if len(enabledWifiInfo) > 0:
            self.__wifiInfo = Info('wifi', 'Information regarding the device\'s wifi connection', ['device'])
            self.__wifiInfo.labels('dobot_'+self.__port).info(enabledWifiInfo)

        if dobotSection.getboolean('DeviceTime', fallback=False):
            self.__deviceTime = Gauge('device_time','Device\'s clock/time', ['device'])

        if dobotSection.getboolean('QueueIndex', fallback=False):
            self.__queueIndex = Gauge('queue_index','Current index in command queue', ['device'])

        if dobotSection.getboolean('PoseX', fallback=True):
            self.__poseX = Gauge('pose_x','Real-time cartesian coordinate of device\'s X axis', ['device'])

        if dobotSection.getboolean('PoseY', fallback=True):
            self.__poseY = Gauge('pose_y','Real-time cartesian coordinate of device\'s Y axis', ['device'])

        if dobotSection.getboolean('PoseZ', fallback=True):
            self.__poseZ = Gauge('pose_z','Real-time cartesian coordinate of device\'s Z axis', ['device'])

        if dobotSection.getboolean('PoseR', fallback=True):
            self.__poseR = Gauge('pose_r','Real-time cartesian coordinate of device\'s R axis', ['device'])

        if dobotSection.getboolean('AngleBase', fallback=True):
            self.__angleBase = Gauge('angle_base','Base joint angle', ['device'])

        if dobotSection.getboolean('AngleRearArm', fallback=True):
            self.__angleRearArm = Gauge('angle_rear_arm','Rear arm joint angle', ['device'])

        if dobotSection.getboolean('AngleForearm', fallback=True):
            self.__angleForearm = Gauge('angle_forearm','Forearm joint angle', ['device'])

        if dobotSection.getboolean('AngleEndEffector', fallback=True):
            self.__angleEndEffector = Gauge('angle_end_effector','End effector joint angle', ['device'])

        if dobotSection.getboolean('AlarmsState', fallback=True):
            self.__alarmsState = Enum('alarms', 'Device alarms', states=list(dType.alarms.values()), ['device'])

        if dobotSection.getboolean('HomeX', fallback=False):
            self.__homeX = Gauge('home_x','Home position for X axis', ['device'])

        if dobotSection.getboolean('HomeY', fallback=False):
            self.__homeY = Gauge('home_y','Home position for Y axis', ['device'])

        if dobotSection.getboolean('HomeZ', fallback=False):
            self.__homeZ = Gauge('home_z','Home position for Z axis', ['device'])

        if dobotSection.getboolean('HomeR', fallback=False):
            self.__homeR = Gauge('home_r','Home position for R axis', ['device'])

        if dobotSection.getboolean('AutoLevelingResult', fallback=False):
            self.__autoLevelingResult = Gauge('auto_leveling_result','Automatic leveling precision result', ['device'])

        if dobotSection.getboolean('EndEffectorX', fallback=False):
            self.__endEffectorX = Gauge('end_effector_x','X-axis offset of end effector', ['device'])

        if dobotSection.getboolean('EndEffectorY', fallback=False):
            self.__endEffectorY = Gauge('end_effector_y','Y-axis offset of end effector', ['device'])

        if dobotSection.getboolean('EndEffectorZ', fallback=False):
            self.__endEffectorZ = Gauge('end_effector_z','Z-axis offset of end effector', ['device'])

        if dobotSection.getboolean('LaserStatus', fallback=False):
            self.__laserStatus = Enum('laser_status','Status (enabled/disabled) of laser', states=['enabled','disabled'], ['device'])

        if dobotSection.getboolean('SuctionCupStatus', fallback=False):
            self.__suctionCupStatus = Enum('suction_cup_status','Status (enabled/disabled) of suction cup', states=['enabled','disabled'], ['device'])

        if dobotSection.getboolean('GripperStatus', fallback=False):
            self.__gripperStatus = Enum('gripper_status','Status (enabled/disabled) of gripper', states=['enabled','disabled'], ['device'])

        if dobotSection.getboolean('JogBaseVelocity', fallback=False):
            self.__jogBaseVelocity = Gauge('jog_base_velocity','Velocity (°/s) of base joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogRearArmVelocity', fallback=False):
            self.__jogRearArmVelocity = Gauge('jog_rear_arm_velocity','Velocity (°/s) of rear arm joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogForearmVelocity', fallback=False):
            self.__jogForearmVelocity = Gauge('jog_forearm_velocity','Velocity (°/s) of forearm joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogEndEffectorVelocity', fallback=False):
            self.__jogEndEffectorVelocity = Gauge('jog_end_effector_velocity','Velocity (°/s) of end effector joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogBaseAcceleration', fallback=False):
            self.__jogBaseAcceleration = Gauge('jog_base_acceleration','Acceleration (°/s^2) of base joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogRearArmAcceleration', fallback=False):
            self.__jogRearArmAcceleration = Gauge('jog_rear_arm_acceleration','Acceleration (°/s^2) of rear arm joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogForearmAcceleration', fallback=False):
            self.__jogForearmAcceleration = Gauge('jog_forearm_acceleration','Acceleration (°/s^2) of forearm joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogEndEffectorAcceleration', fallback=False):
            self.__jogEndEffectorAcceleration = Gauge('jog_end_effector_acceleration','Acceleration (°/s^2) of end effector joint in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisXVelocity', fallback=False):
            self.__jogAxisXVelocity = Gauge('jog_axis_x_velocity','Velocity (mm/s) of device\'s X axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisYVelocity', fallback=False):
            self.__jogAxisYVelocity = Gauge('jog_axis_y_velocity','Velocity (mm/s) of device\'s Y axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisZVelocity', fallback=False):
            self.__jogAxisZVelocity = Gauge('jog_axis_z_velocity','Velocity (mm/s) of device\'s Z axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisRVelocity', fallback=False):
            self.__jogAxisRVelocity = Gauge('jog_axis_r_velocity','Velocity (mm/s) of device\'s R axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisXAcceleration', fallback=False):
            self.__jogAxisXAcceleration = Gauge('jog_axis_x_acceleration','Acceleration (mm/s^2) of device\'s X axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisYAcceleration', fallback=False):
            self.__jogAxisYAcceleration = Gauge('jog_axis_y_acceleration','Acceleration (mm/s^2) of device\'s Y axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisZAcceleration', fallback=False):
            self.__jogAxisZAcceleration = Gauge('jog_axis_z_acceleration','Acceleration (mm/s^2) of device\'s Z axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAxisRAcceleration', fallback=False):
            self.__jogAxisRAcceleration = Gauge('jog_axis_r_acceleration','Acceleration (mm/s^2) of device\'s R axis (cartesian coordinate) in jogging mode', ['device'])

        if dobotSection.getboolean('JogVelocityRatio', fallback=False):
            self.__jogVelocityRatio = Gauge('jog_velocity_ratio','Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode', ['device'])

        if dobotSection.getboolean('JogAccelerationRatio', fallback=False):
            self.__jogAccelerationRatio = Gauge('jog_acceleration_ratio','Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode', ['device'])

        if dobotSection.getboolean('PtpBaseVelocity', fallback=False):
            self.__ptpBaseVelocity = Gauge('ptp_base_velocity','Velocity (°/s) of base joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpRearArmVelocity', fallback=False):
            self.__ptpRearArmVelocity = Gauge('ptp_rear_arm_velocity','Velocity (°/s) of rear arm joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpForearmVelocity', fallback=False):
            self.__ptpForearmVelocity = Gauge('ptp_forearm_velocity','Velocity (°/s) of forearm joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpEndEffectorVelocity', fallback=False):
            self.__ptpEndEffectorVelocity = Gauge('ptp_end_effector_velocity','Velocity (°/s) of end effector joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpBaseAcceleration', fallback=False):
            self.__ptpBaseAcceleration = Gauge('ptp_base_acceleration','Acceleration (°/s^2) of base joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpRearArmAcceleration', fallback=False):
            self.__ptpRearArmAcceleration = Gauge('ptp_rear_arm_acceleration','Acceleration (°/s^2) of rear arm joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpForearmAcceleration', fallback=False):
            self.__ptpForearmAcceleration = Gauge('ptp_forearm_acceleration','Acceleration (°/s^2) of forearm joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpEndEffectorAcceleration', fallback=False):
            self.__ptpEndEffectorAcceleration = Gauge('ptp_end_effector_acceleration','Acceleration (°/s^2) of end effector joint in point to point mode', ['device'])

        if dobotSection.getboolean('PtpXYZVelocity', fallback=False):
            self.__ptpXYZVelocity = Gauge('ptp_xyz_velocity','Velocity (mm/s) of device\'s X, Y, Z axis (cartesian coordinate) in point to point mode', ['device'])

        if dobotSection.getboolean('PtpRVelocity', fallback=False):
            self.__ptpRVelocity = Gauge('ptp_r_velocity','Velocity (mm/s) of device\'s R axis (cartesian coordinate) in point to point mode', ['device'])

        if dobotSection.getboolean('PtpXYZAcceleration', fallback=False):
            self.__ptpXYZAcceleration = Gauge('ptp_x_y_z_acceleration','Acceleration (mm/s^2) of device\'s X, Y, Z axis (cartesian coordinate) in point to point mode', ['device'])

        if dobotSection.getboolean('PtpRAcceleration', fallback=False):
            self.__ptpRAcceleration = Gauge('ptp_r_acceleration','Acceleration (mm/s^2) of device\'s R axis (cartesian coordinate) in point to point mode', ['device'])

        if dobotSection.getboolean('PtpVelocityRatio', fallback=False):
            self.__ptpVelocityRatio = Gauge('ptp_velocity_ratio','Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode', ['device'])

        if dobotSection.getboolean('PtpAccelerationRatio', fallback=False):
            self.__ptpAccelerationRatio = Gauge('ptp_acceleration_ratio','Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode', ['device'])

        if dobotSection.getboolean('LiftingHeight', fallback=False):
            self.__liftingHeight = Gauge('lifting_height','Lifting height in jump mode', ['device'])

        if dobotSection.getboolean('HeighLimit', fallback=False):
            self.__heightLimit = Gauge('height_limit','Max lifting height in jump mode', ['device'])

        if dobotSection.getboolean('CpVelocity', fallback=False):
            self.__cpVelocity = Gauge('cp_velocity','Velocity (mm/s) in cp mode', ['device'])

        if dobotSection.getboolean('CpAcceleration', fallback=False):
            self.__cpAcceleration = Gauge('cp_acceleration','Acceleration (mm/s^2) in cp mode', ['device'])

        if dobotSection.getboolean('ArcXYZVelocity', fallback=False):
            self.__arcXYZVelocity = Gauge('arc_x_y_z_velocity','Velocity (mm/s) of X, Y, Z axis in arc mode', ['device'])

        if dobotSection.getboolean('ArcRVelocity', fallback=False):
            self.__arcRVelocity = Gauge('arc_r_velocity','Velocity (mm/s) of R axis in arc mode', ['device'])

        if dobotSection.getboolean('ArcXYZAcceleration', fallback=False):
            self.__arcXYZAcceleration = Gauge('arc_x_y_z_acceleration','Acceleration (mm/s^2) of X, Y, Z axis in arc mode', ['device'])

        if dobotSection.getboolean('ArcRAcceleration', fallback=False):
            self.__arcRAcceleration = Gauge('arc_r_acceleration','Acceleration (mm/s^2) of R axis in arc mode', ['device'])

        if dobotSection.getboolean('AngleStaticErrRear', fallback=False):
            self.__angleStaticErrRear = Gauge('angle_static_err_rear','Rear arm angle sensor static error', ['device'])

        if dobotSection.getboolean('AngleStaticErrFront', fallback=False):
            self.__angleStaticErrFront = Gauge('arc_static_err_front','Forearm angle sensor static error', ['device'])

        if dobotSection.getboolean('AngleCoefRear', fallback=False):
            self.__angleCoefRear = Gauge('angle_coef_rear','Rear arm angle sensor linearization parameter', ['device'])

        if dobotSection.getboolean('AngleCoefFront', fallback=False):
            self.__angleCoefFront = Gauge('angle_coef_front','Forearm angle sensor linearization parameter', ['device'])

        if dobotSection.getboolean('SlidingRailStatus', fallback=False):
            self.__slidingRailStatus = Enum('sliding_rail_status','Sliding rail\'s status (enabled/disabled)', states=['enabled','disabled'], ['device'])

        if dobotSection.getboolean('SlidingRailPose', fallback=False):
            self.__slidingRailPose = Gauge('sliding_rail_pose','Sliding rail\'s real-time pose in mm', ['device'])

        if dobotSection.getboolean('SlidingRailJogVelocity', fallback=False):
            self.__slidingRailJogVelocity = Gauge('sliding_rail_jog_velocity','Velocity (mm/s) of sliding rail in jogging mode', ['device'])

        if dobotSection.getboolean('SlidingRailJogAcceleration', fallback=False):
            self.__slidingRailJogAcceleration = Gauge('sliding_rail_jog_acceleration','Acceleration (mm/s^2) of sliding rail in jogging mode', ['device'])

        if dobotSection.getboolean('SlidingRailPtpVelocity', fallback=False):
            self.__slidingRailPtpVelocity = Gauge('sliding_rail_ptp_velocity','Velocity (mm/s) of sliding rail in point to point mode', ['device'])

        if dobotSection.getboolean('SlidingRailPtpAcceleration', fallback=False):
            self.__slidingRailPtpAcceleration = Gauge('sliding_rail_ptp_acceleration','Acceleration (mm/s^2) of sliding rail in point to point mode', ['device'])

        if dobotSection.getboolean('WifiModuleStatus', fallback=False):
            self.__wifiModuleStatus = Enum('wifi_module_status','Wifi module status (enabled/disabled)', states=['enabled','disabled'], ['device'])

        if dobotSection.getboolean('WifiConnectionStatus', fallback=False):
            self.__wifiConnectionStatus = Enum('wifi_connection_status','Wifi connection status (connected/not connected)', states=['enabled','disabled'], ['device'])

    def fetchData(self):
        global config
        dobotSection = config['DOBOT' + ':' + self.__port]

        if dobotSection.getboolean('DeviceTime', fallback=False):
            self.__deviceTime.labels('dobot_'+self.__port).set(dType.GetDeviceTime(self.__api)[0])

        if dobotSection.getboolean('QueueIndex', fallback=False):
            self.__queueIndex.labels('dobot_'+self.__port).set(dType.GetQueuedCmdCurrentIndex(self.__api)[0])

        pose = dType.GetPose(self.__api)
        if dobotSection.getboolean('PoseX', fallback=True):
            self.__poseX.labels('dobot_'+self.__port).set(pose[0])

        if dobotSection.getboolean('PoseY', fallback=True):
            self.__poseY.labels('dobot_'+self.__port).set(pose[1])

        if dobotSection.getboolean('PoseZ', fallback=True):
            self.__poseZ.labels('dobot_'+self.__port).set(pose[2])

        if dobotSection.getboolean('PoseR', fallback=True):
            self.__poseR.labels('dobot_'+self.__port).set(pose[3])

        if dobotSection.getboolean('AngleBase', fallback=True):
            self.__angleBase.labels('dobot_'+self.__port).set(pose[4])

        if dobotSection.getboolean('AngleRearArm', fallback=True):
            self.__angleRearArm.labels('dobot_'+self.__port).set(pose[5])

        if dobotSection.getboolean('AngleForearm', fallback=True):
            self.__angleForearm.labels('dobot_'+self.__port).set(pose[6])

        if dobotSection.getboolean('AngleEndEffector', fallback=True):
            self.__angleEndEffector.labels('dobot_'+self.__port).set(pose[7])

        if dobotSection.getboolean('AlarmsState', fallback=True):
            for a in dType.GetAlarmsState(self.__api)
                self.__alarmsState.labels('dobot_'+self.__port).state(a)

        home = dType.GetHOMEParams(self.__api)
        if dobotSection.getboolean('HomeX', fallback=False):
            self.__homeX.labels('dobot_'+self.__port).set(home[0])

        if dobotSection.getboolean('HomeY', fallback=False):
            self.__homeY.labels('dobot_'+self.__port).set(home[1])

        if dobotSection.getboolean('HomeZ', fallback=False):
            self.__homeZ.labels('dobot_'+self.__port).set(home[2])

        if dobotSection.getboolean('HomeR', fallback=False):
            self.__homeR.labels('dobot_'+self.__port).set(home[3])

        if dobotSection.getboolean('AutoLevelingResult', fallback=False):
            self.__autoLevelingResult.labels('dobot_'+self.__port).set(dType.GetAutoLevelingResult(self.__api)[0])

        endEffector = dType.GetEndEffectorParams(self.__api)
        if dobotSection.getboolean('EndEffectorX', fallback=False):
            self.__endEffectorX.labels('dobot_'+self.__port).set(endEffector[0])

        if dobotSection.getboolean('EndEffectorY', fallback=False):
            self.__endEffectorY.labels('dobot_'+self.__port).set(endEffector[1])

        if dobotSection.getboolean('EndEffectorZ', fallback=False):
            self.__endEffectorZ.labels('dobot_'+self.__port).set(endEffector[2])

        if dobotSection.getboolean('LaserStatus', fallback=False):
            if bool(dType.GetEndEffectorLaser(self.__api)[0]):
                self.__laserStatus.labels('dobot_'+self.__port).state('enabled')
            else:
                self.__laserStatus.labels('dobot_'+self.__port).state('disabled')

        if dobotSection.getboolean('SuctionCupStatus', fallback=False):
            if bool(dType.GetEndEffectorSuctionCup(self.__api)[0]):
                self.__suctionCupStatus.labels('dobot_'+self.__port).state('enabled')
            else:
                self.__suctionCupStatus.labels('dobot_'+self.__port).state('disabled')

        if dobotSection.getboolean('GripperStatus', fallback=False):
            if bool(dType.GetEndEffectorGripper(self.__api)[0]):
                self.__gripperStatus.labels('dobot_'+self.__port).state('enabled')
            else:
                self.__gripperStatus.labels('dobot_'+self.__port).state('disabled')

        jogJoints = dType.GetJOGJointParams(self.__api)
        if dobotSection.getboolean('JogBaseVelocity', fallback=False):
            self.__jogBaseVelocity.labels('dobot_'+self.__port).set(jogJoints[0])

        if dobotSection.getboolean('JogRearArmVelocity', fallback=False):
            self.__jogRearArmVelocity.labels('dobot_'+self.__port).set(jogJoints[1])

        if dobotSection.getboolean('JogForearmVelocity', fallback=False):
            self.__jogForearmVelocity.labels('dobot_'+self.__port).set(jogJoints[2])

        if dobotSection.getboolean('JogEndEffectorVelocity', fallback=False):
            self.__jogEndEffectorVelocity.labels('dobot_'+self.__port).set(jogJoints[3])

        if dobotSection.getboolean('JogBaseAcceleration', fallback=False):
            self.__jogBaseAcceleration.labels('dobot_'+self.__port).set(jogJoints[4])

        if dobotSection.getboolean('JogRearArmAcceleration', fallback=False):
            self.__jogRearArmAcceleration.labels('dobot_'+self.__port).set(jogJoints[5])

        if dobotSection.getboolean('JogForearmAcceleration', fallback=False):
            self.__jogForearmAcceleration.labels('dobot_'+self.__port).set(jogJoints[6])

        if dobotSection.getboolean('JogEndEffectorAcceleration', fallback=False):
            self.__jogEndEffectorAcceleration.labels('dobot_'+self.__port).set(jogJoints[7])

        jogCoords = dType.GetJOGCoordinateParams(self.__api)
        if dobotSection.getboolean('JogAxisXVelocity', fallback=False):
            self.__jogAxisXVelocity.labels('dobot_'+self.__port).set(jogCoords[0])

        if dobotSection.getboolean('JogAxisYVelocity', fallback=False):
            self.__jogAxisYVelocity.labels('dobot_'+self.__port).set(jogCoords[1])

        if dobotSection.getboolean('JogAxisZVelocity', fallback=False):
            self.__jogAxisZVelocity.labels('dobot_'+self.__port).set(jogCoords[2])

        if dobotSection.getboolean('JogAxisRVelocity', fallback=False):
            self.__jogAxisRVelocity.labels('dobot_'+self.__port).set(jogCoords[3])

        if dobotSection.getboolean('JogAxisXAcceleration', fallback=False):
            self.__jogAxisXAcceleration.labels('dobot_'+self.__port).set(jogCoords[4])

        if dobotSection.getboolean('JogAxisYAcceleration', fallback=False):
            self.__jogAxisYAcceleration.labels('dobot_'+self.__port).set(jogCoords[5])

        if dobotSection.getboolean('JogAxisZAcceleration', fallback=False):
            self.__jogAxisZAcceleration.labels('dobot_'+self.__port).set(jogCoords[6])

        if dobotSection.getboolean('JogAxisRAcceleration', fallback=False):
            self.__jogAxisRAcceleration.labels('dobot_'+self.__port).set(jogCoords[7])

        jogCommon = dType.GetJOGCommonParams(self.__api)
        if dobotSection.getboolean('JogVelocityRatio', fallback=False):
            self.__jogVelocityRatio.labels('dobot_'+self.__port).set(jogCommon[0])

        if dobotSection.getboolean('JogAccelerationRatio', fallback=False):
            self.__jogAccelerationRatio.labels('dobot_'+self.__port).set(jogCommon[1])

        ptpJoints = dType.GetPTPJointParams(self.__api)
        if dobotSection.getboolean('PtpBaseVelocity', fallback=False):
            self.__ptpBaseVelocity.labels('dobot_'+self.__port).set(ptpJoints[0])

        if dobotSection.getboolean('PtpRearArmVelocity', fallback=False):
            self.__ptpRearArmVelocity.labels('dobot_'+self.__port).set(ptpJoints[1])

        if dobotSection.getboolean('PtpForearmVelocity', fallback=False):
            self.__ptpForearmVelocity.labels('dobot_'+self.__port).set(ptpJoints[2])

        if dobotSection.getboolean('PtpEndEffectorVelocity', fallback=False):
            self.__ptpEndEffectorVelocity.labels('dobot_'+self.__port).set(ptpJoints[3])

        if dobotSection.getboolean('PtpBaseAcceleration', fallback=False):
            self.__ptpBaseAcceleration.labels('dobot_'+self.__port).set(ptpJoints[4])

        if dobotSection.getboolean('PtpRearArmAcceleration', fallback=False):
            self.__ptpRearArmAcceleration.labels('dobot_'+self.__port).set(ptpJoints[5])

        if dobotSection.getboolean('PtpForearmAcceleration', fallback=False):
            self.__ptpForearmAcceleration.labels('dobot_'+self.__port).set(ptpJoints[6])

        if dobotSection.getboolean('PtpEndEffectorAcceleration', fallback=False):
            self.__ptpEndEffectorAcceleration.labels('dobot_'+self.__port).set(ptpJoints[7])

        ptpCoords = dType.GetPTPCoordinateParams(self.__api)
        if dobotSection.getboolean('PtpXYZVelocity', fallback=False):
            self.__ptpXYZVelocity.labels('dobot_'+self.__port).set(ptpCoords[0])

        if dobotSection.getboolean('PtpRVelocity', fallback=False):
            self.__ptpRVelocity.labels('dobot_'+self.__port).set(ptpCoords[1])

        if dobotSection.getboolean('PtpXYZAcceleration', fallback=False):
            self.__ptpXYZAcceleration.labels('dobot_'+self.__port).set(ptpCoords[2])

        if dobotSection.getboolean('PtpRAcceleration', fallback=False):
            self.__ptpRAcceleration.labels('dobot_'+self.__port).set(ptpCoords[3])

        ptpCommon = dType.GetPTPCommonParams(self.__api)
        if dobotSection.getboolean('PtpVelocityRatio', fallback=False):
            self.__ptpVelocityRatio.labels('dobot_'+self.__port).set(ptpCommon[0])

        if dobotSection.getboolean('PtpAccelerationRatio', fallback=False):
            self.__ptpAccelerationRatio.labels('dobot_'+self.__port).set(ptpCommon[1])

        ptpJump = dType.GetPTPJumpParams(self.__api)
        if dobotSection.getboolean('LiftingHeight', fallback=False):
            self.__liftingHeight.labels('dobot_'+self.__port).set(ptpJump[0])

        if dobotSection.getboolean('HeighLimit', fallback=False):
            self.__heightLimit.labels('dobot_'+self.__port).set(ptpJump[1])

        cp = dType.GetCPParams(self.__api)
        if dobotSection.getboolean('CpVelocity', fallback=False):
            self.__cpVelocity.labels('dobot_'+self.__port).set(cp[0])

        if dobotSection.getboolean('CpAcceleration', fallback=False):
            self.__cpAcceleration.labels('dobot_'+self.__port).set(cp[1])

        arc = dType.GetARCParams(self.__api)
        if dobotSection.getboolean('ArcXYZVelocity', fallback=False):
            self.__arcXYZVelocity.labels('dobot_'+self.__port).set(arc[0])

        if dobotSection.getboolean('ArcRVelocity', fallback=False):
            self.__arcRVelocity.labels('dobot_'+self.__port).set(arc[1])

        if dobotSection.getboolean('ArcXYZAcceleration', fallback=False):
            self.__arcXYZAcceleration.labels('dobot_'+self.__port).set(arc[2])

        if dobotSection.getboolean('ArcRAcceleration', fallback=False):
            self.__arcRAcceleration.labels('dobot_'+self.__port).set(arc[3])

        angleStaticErr = dType.GetAngleSensorStaticError(self.__api)
        if dobotSection.getboolean('AngleStaticErrRear', fallback=False):
            self.__angleStaticErrRear.labels('dobot_'+self.__port).set(angleStaticErr[0])

        if dobotSection.getboolean('AngleStaticErrFront', fallback=False):
            self.__angleStaticErrFront.labels('dobot_'+self.__port).set(angleStaticErr[1])

        angleCoef = dType.GetAngleSensorCoef(self.__api)
        if dobotSection.getboolean('AngleCoefRear', fallback=False):
            self.__angleCoefRear.labels('dobot_'+self.__port).set(angleCoef[0])

        if dobotSection.getboolean('AngleCoefFront', fallback=False):
            self.__angleCoefFront.labels('dobot_'+self.__port).set(angleCoef[1])

        if dobotSection.getboolean('SlidingRailStatus', fallback=False):
            if bool(dType.GetDeviceWithL(self.__api)[0]):
                self.__slidingRailStatus.labels('dobot_'+self.__port).state('enabled')
            else:
                self.__slidingRailStatus.labels('dobot_'+self.__port).state('disabled')

        if dobotSection.getboolean('SlidingRailPose', fallback=False):
            self.__slidingRailPose.labels('dobot_'+self.__port).set(dType.GetPoseL(self.__api)[0])

        jogRail = dType.GetJOGLParams(self.__api)
        if dobotSection.getboolean('SlidingRailJogVelocity', fallback=False):
            self.__slidingRailJogVelocity.labels('dobot_'+self.__port).set(jogRail[0])

        if dobotSection.getboolean('SlidingRailJogAcceleration', fallback=False):
            self.__slidingRailJogAcceleration.labels('dobot_'+self.__port).set(jogRail[1])

        ptpRail = dType.GetPTPLParams(self.__api)
        if dobotSection.getboolean('SlidingRailPtpVelocity', fallback=False):
            self.__slidingRailPtpVelocity.labels('dobot_'+self.__port).set(ptpRail[0])

        if dobotSection.getboolean('SlidingRailPtpAcceleration', fallback=False):
            self.__slidingRailPtpAcceleration.labels('dobot_'+self.__port).set(ptpRail[1])


        if dobotSection.getboolean('WifiModuleStatus', fallback=False):
            if bool(dType.GetWIFIConfigMode(self.__api)[0]):
                self.__wifiModuleStatus.labels('dobot_'+self.__port).state('enabled')
            else:
                self.__wifiModuleStatus.labels('dobot_'+self.__port).state('disabled')

        if dobotSection.getboolean('WifiConnectionStatus', fallback=False):
            if bool(dType.GetWIFIConnectStatus(self.__api)[0]):
                self.__wifiConnectionStatus.labels('dobot_'+self.__port).state('enabled')
            else:
                self.__wifiConnectionStatus.labels('dobot_'+self.__port).state('disabled')


class JevoisCamera():
    def __init__(self, serial, port, dimension):
        global config

        self.__serial = serial
        self.__port = port
        self.__dimension = dimension
        self.__options = None
        jevoisSection = config['JEVOIS' + ':' + self.__port]

        if jevoisSection.getboolean('ObjectIdentified', fallback=True):
            if jevoisSection.get('objects') is not None:
                self.__options = jevoisSection["options"].split()
                self.__objectIdentified = Enum('object_identified', 'Object Identified', states=self.__options, ['device'])
            else:
                print('The \"options\" list is necessary for monitoring identified objects')
                print('Skipping monitoring objects identified for JEVOIS:' + self.__port)

        if jevoisSection.getboolean('ObjectLocation', fallback=True):
            self.__objectLocationX = Gauge('object_location_x', 'Identified object\'s x position',['device'])
            if int(self.__dimension) > 1:
                self.__objectLocationY = Gauge('object_location_y', 'Identified object\'s y position',['device'])
            if int(self.__dimension) == 3:
                self.__objectLocationZ = Gauge('object_location_z', 'Identified object\'s Z position',['device'])

        if jevoisSection.getboolean('ObjectSize', fallback=False):
            self.__objectSize = Gauge('object_size','Identified object\'s size', ['device'])

    def fetchData(self):
        line = self.__serial.readline().rstrip()
        tok = line.split()
        # Abort fetching if timeout or malformed line
        if len(tok) < 1: return
        if self.__dimension == '1' and len(tok) != 4: return
        if self.__dimension == '2' and len(tok) != 6: return
        if self.__dimension == '3' and len(tok) != 8: return

        if jevoisSection.getboolean('ObjectIdentified', fallback=True):
            if self.__options is not None and tok[1] in self.__options:
                self.__objectIdentified.labels('jevois'+self.__port).state(tok[1])

        if jevoisSection.getboolean('ObjectLocation', fallback=True):
            self.__objectLocationX.lables('jevois'+self.__port).set(float(tok[2]))

            if int(self.__dimension) > 1:
                self.__objectLocationY.labels('jevois'+self.__port).set(float(tok[3]))

            if int(self.__dimension) == 3:
                self.__objectLOcationZ.labels('jevois'+self.__port).set(float(tok[4]))

        if jevoisSection.getboolean('ObjectSize', fallback=False):
            if self.__dimension == '1':
                self.__objectSize.labels('jevois'+self.__port).set(float(tok[3]))
            elif self.__dimension == '2':
                self.__objectSize.labels('jevois'+self.__port).set(float(tok[4])*float(tok[5]))
            elif self.__dimension == '3':
                self.__objectSize.labels('jevois'+self.__port).set(float(tok[5])*float(tok[6])*float(tok[7]))


class MonitoringAgent():
    def __init__(self):
        global config
        self.__agentName = config.get('AGENT', 'AgentName', fallback="Agent0")
        if self.__agentName == '':
            self.__agentName = "Agent0"

        try:
            self.__routineInterval = config.getint('AGENT', 'routineInterval', fallback=100)
            if (self.__routineInterval < 100):
                raise ValueError
        except ValueError:
            print('RoutineInterval must be a number greater or equal to 100')
            sys.exit(4)

        try:
            self.__prometheusPort = config.getint('AGENT', 'PrometheusPort', fallback=8080)
            if self.__prometheusPort > 65535 or self.__prometheusPort < 0:
                print(str(self.__prometheusPort) + " is not a valid port")
                raise ValueError
        except ValueError:
            print('PrometheusPort must be a number from 0 to 65535')
            sys.exit(5)

        self.__devices = []

    def __connectDobot(self, port):
        # Load Dll and get the CDLL object
        api = dType.load()
        state = dType.ConnectDobot(api, port, 115200)[0]

        if state == dType.DobotConnect.DobotConnect_NoError:
            print("Dobot Magician at port " + port + " connected succesfully!")
            return DobotMagician(api, port)
        else:
            print("Couldn't connect with Dobot Magician device at port " + port)
            return None

    def __connectJevois(self, port):
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            line = ser.readline().rstrip()
            tok = line.split()
            # Raise exception if timeout or malformed line or not supported serstyle
            if len(tok) < 1: raise Exception('The line is malformed.')
            if tok[0][0] != 'N': raise Exception('Unsupported serstyle. Can only interpret Normal messages.')
            print("Jevois Camera at port " + port + " connected succesfully!")
            return JevoisCamera(ser, port, tok[0][1])
        except:
            print("Couldn't connect with Jevois Camera device at port " + port)
            return None

    def __connectDevices(self):
        global config

        # Discover through the config which devices should be monitored
        for section in config:
            # Skip the Agent section as it does not represent a device
            if section == 'AGENT':
                continue

            try:
                part = section.split(':')
                name = part[0]
                port = part[1]
            except IndexError:
                print('All device entries should follow this format [DEVICE:PORT]')
                print('For more information use --help')
                exit(6)

            # Depending on the type of device try to connect to it
            if name == 'DOBOT':
                dobot = self.__connectDobot(port)
                if dobot is not None:
                    self.__devices.append(dobot)
            elif name == 'JEVOIS':
                jevois = self.__connectJevois(port)
                if jevois is not None:
                    self.__devices.append(jevois)
            else:
                print("Device with name " + name + " cannot be recognised")

    def __disconnectDevices(self):
        dType.DisconnectAll() # Disconnect Dobot devices and not leave any junk files on the disk

    def startRoutine(self):
        if len(self.__devices) == 0:
            print("No devices connected to the agent.")
            sys.exit(11)

        print('Connecting to devices listed in agent.conf..')
        self.__connectDevices()
        print('Starting prometheus server at port ' + str(self.__prometheusPort) + "..")
        start_http_server(self.__prometheusPort)

        print('Monitoring..')
        try:
            while (1):
                time.sleep(self.__routineInterval / 1000)
                for device in self.__devices:
                    device.fetchData()
        except KeyboardInterrupt:
            self.__disconnectDevices()


def argumentHandler(args):
    if len(args) == 2:
        if args[1] == "-h" or args[1] == "--help":
            # Open latest README.md documentation of diploma-project
            webbrowser.open('https://github.com/akomis/diploma-project/blob/master/README.md')
            exit(1)
        else:
            print('Unrecognised option ' + sys.argv[1])
            print('For more information: $ agent.py --help')
            exit(2)

def readConfig():
    global config
    try:
        check = config.read('agent.conf')[0]
        if check == '':
            raise Exception("Couldn't find agent.conf")
    except:
        print("Cant open configuration file. Make sure agent.conf is in the same directory as agent.py")
        exit(3)

def main():
    argumentHandler(sys.argv)
    readConfig()
    Agent = MonitoringAgent()
    Agent.startRoutine()

if __name__ == '__main__':
    main()
