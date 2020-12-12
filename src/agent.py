import sys
import time
import webbrowser
import configparser
import DobotDllType as dType
from prometheus_client import start_http_server, Info, Gauge, Enum

config = configparser.ConfigParser()


class DobotMagician():
    # {bit address index : alarm description}
    alarms = {"0x00": "Public Alarm: Reset Alarm", "0x01": "Public Alarm: Undefined Instruction", "0x02": "Public Alarm: File System Error", "0x03": "Public Alarm: Failured Communication between MCU and FPGA", "0x04": "Public Alarm: Angle Sensor Reading Error",
              "0x11": "Planning Alarm: Inverse Resolve Alarm", "0x12": "Planning Alarm: Inverse Resolve Limit", "0x13": "Planning Alarm: Data Repetition", "0x14": "Planning Alarm: Arc Input Parameter Alarm", "0x15": "Planning Alarm: JUMP Parameter Error",
              "0x21": "Kinematic Alarm: Inverse Resolve Alarm", "0x22": "Kinematic Alarm: Inverse Resolve Limit",
              "0x40": "Limit Alarm: Joint 1 Positive Limit Alarm", "0x41": "Limit Alarm: Joint 1 Negative Limit Alarm", "0x42": "Limit Alarm: Joint 2 Positive Limit Alarm", "0x43": "Limit Alarm: Joint 2 Negative Limit Alarm", "0x44": "Limit Alarm: Joint 3 Positive Limit Alarm", "0x45": "Limit Alarm: Joint 3 Negative Limit Alarm", "0x46": "Limit Alarm: Joint 4 Positive Limit Alarm", "0x47": "Limit Alarm: Joint 4 Negative Limit Alarm", "0x48": "Limit Alarm: Parallegram Positive Limit Alarm", "0x49": "Limit Alarm: Parallegram Negative Limit Alarm"}


    def __init__(self, api, port):
        global config

        self.__api = api
        self.__port = port
        dobotSection = config['DOBOT' + ':' + self.__port]

        enabledDeviceInfo = {}
        if dobotSection.getboolean('DeviceSN', fallback=False):
            enabledDeviceInfo["serial_number"] = dType.GetDeviceSN(self.__api)[0]
        if dobotSection.getboolean('DeviceName', fallback=False):
            enabledDeviceInfo["device_name"] = dType.GetDeviceName(self.__api)[0]
        if dobotSection.getboolean('DeviceVersion', fallback=False):
            enabledDeviceInfo["version"] = '.'.join(list(map(str, dType.GetDeviceVersion(self.__api))))
        if len(enabledDeviceInfo) > 0:
            self.__deviceInfo = Info(self.__port.lower()+'_'+'dobot_magician', 'General information about monitored Dobot Magician device')
            self.__deviceInfo.info(enabledDeviceInfo)

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
            self.__wifiInfo = Info(self.__port.lower()+'_'+'wifi', 'Information regarding the device\'s wifi connection')
            self.__wifiInfo.info(enabledWifiInfo)

        if dobotSection.getboolean('DeviceTime', fallback=False):
            self.__deviceTime = Gauge(self.__port.lower()+'_'+'device_time','Device\'s clock/time')

        if dobotSection.getboolean('QueueIndex', fallback=False):
            self.__queueIndex = Gauge(self.__port.lower()+'_'+'queue_index','Current index in command queue')

        if dobotSection.getboolean('PoseX', fallback=False):
            self.__poseX = Gauge(self.__port.lower()+'_'+'pose_x','Real-time cartesian coordinate of device\'s X axis')

        if dobotSection.getboolean('PoseY', fallback=False):
            self.__poseY = Gauge(self.__port.lower()+'_'+'pose_y','Real-time cartesian coordinate of device\'s Y axis')

        if dobotSection.getboolean('PoseZ', fallback=False):
            self.__poseZ = Gauge(self.__port.lower()+'_'+'pose_z','Real-time cartesian coordinate of device\'s Z axis')

        if dobotSection.getboolean('PoseR', fallback=False):
            self.__poseR = Gauge(self.__port.lower()+'_'+'pose_r','Real-time cartesian coordinate of device\'s R axis')

        if dobotSection.getboolean('AngleBase', fallback=False):
            self.__angleBase = Gauge(self.__port.lower()+'_'+'angle_base','Base joint angle')

        if dobotSection.getboolean('AngleRearArm', fallback=False):
            self.__angleRearArm = Gauge(self.__port.lower()+'_'+'angle_rear_arm','Rear arm joint angle')

        if dobotSection.getboolean('AngleForearm', fallback=False):
            self.__angleForearm = Gauge(self.__port.lower()+'_'+'angle_forearm','Forearm joint angle')

        if dobotSection.getboolean('AngleEndEffector', fallback=False):
            self.__angleEndEffector = Gauge(self.__port.lower()+'_'+'angle_end_effector','End effector joint angle')

        if dobotSection.getboolean('AlarmsState', fallback=False):
            self.__alarmsState = Enum(self.__port.lower()+'_'+'alarms', 'Device alarms', states=list(self.alarms.values()))

        if dobotSection.getboolean('HomeX', fallback=False):
            self.__homeX = Gauge(self.__port.lower()+'_'+'home_x','Home position for X axis')

        if dobotSection.getboolean('HomeY', fallback=False):
            self.__homeY = Gauge(self.__port.lower()+'_'+'home_y','Home position for Y axis')

        if dobotSection.getboolean('HomeZ', fallback=False):
            self.__homeZ = Gauge(self.__port.lower()+'_'+'home_z','Home position for Z axis')

        if dobotSection.getboolean('HomeR', fallback=False):
            self.__homeR = Gauge(self.__port.lower()+'_'+'home_r','Home position for R axis')

        if dobotSection.getboolean('AutoLevelingResult', fallback=False):
            self.__autoLevelingResult = Gauge(self.__port.lower()+'_'+'auto_leveling_result','Automatic leveling precision result')

        if dobotSection.getboolean('EndEffectorX', fallback=False):
            self.__endEffectorX = Gauge(self.__port.lower()+'_'+'end_effector_x','X-axis offset of end effector')

        if dobotSection.getboolean('EndEffectorY', fallback=False):
            self.__endEffectorY = Gauge(self.__port.lower()+'_'+'end_effector_y','Y-axis offset of end effector')

        if dobotSection.getboolean('EndEffectorZ', fallback=False):
            self.__endEffectorZ = Gauge(self.__port.lower()+'_'+'end_effector_z','Z-axis offset of end effector')

        if dobotSection.getboolean('LaserStatus', fallback=False):
            self.__laserStatus = Enum(self.__port.lower()+'_'+'laser_status','Status (enabled/disabled) of laser', states=['enabled','disabled'])

        if dobotSection.getboolean('SuctionCupStatus', fallback=False):
            self.__suctionCupStatus = Enum(self.__port.lower()+'_'+'suction_cup_status','Status (enabled/disabled) of suction cup', states=['enabled','disabled'])

        if dobotSection.getboolean('GripperStatus', fallback=False):
            self.__gripperStatus = Enum(self.__port.lower()+'_'+'gripper_status','Status (enabled/disabled) of gripper', states=['enabled','disabled'])

        if dobotSection.getboolean('JogBaseVelocity', fallback=False):
            self.__jogBaseVelocity = Gauge(self.__port.lower()+'_'+'jog_base_velocity','Velocity (°/s) of base joint in jogging mode')

        if dobotSection.getboolean('JogRearArmVelocity', fallback=False):
            self.__jogRearArmVelocity = Gauge(self.__port.lower()+'_'+'jog_rear_arm_velocity','Velocity (°/s) of rear arm joint in jogging mode')

        if dobotSection.getboolean('JogForearmVelocity', fallback=False):
            self.__jogForearmVelocity = Gauge(self.__port.lower()+'_'+'jog_forearm_velocity','Velocity (°/s) of forearm joint in jogging mode')

        if dobotSection.getboolean('JogEndEffectorVelocity', fallback=False):
            self.__jogEndEffectorVelocity = Gauge(self.__port.lower()+'_'+'jog_end_effector_velocity','Velocity (°/s) of end effector joint in jogging mode')

        if dobotSection.getboolean('JogBaseAcceleration', fallback=False):
            self.__jogBaseAcceleration = Gauge(self.__port.lower()+'_'+'jog_base_acceleration','Acceleration (°/s^2) of base joint in jogging mode')

        if dobotSection.getboolean('JogRearArmAcceleration', fallback=False):
            self.__jogRearArmAcceleration = Gauge(self.__port.lower()+'_'+'jog_rear_arm_acceleration','Acceleration (°/s^2) of rear arm joint in jogging mode')

        if dobotSection.getboolean('JogForearmAcceleration', fallback=False):
            self.__jogForearmAcceleration = Gauge(self.__port.lower()+'_'+'jog_forearm_acceleration','Acceleration (°/s^2) of forearm joint in jogging mode')

        if dobotSection.getboolean('JogEndEffectorAcceleration', fallback=False):
            self.__jogEndEffectorAcceleration = Gauge(self.__port.lower()+'_'+'jog_end_effector_acceleration','Acceleration (°/s^2) of end effector joint in jogging mode')

        if dobotSection.getboolean('JogAxisXVelocity', fallback=False):
            self.__jogAxisXVelocity = Gauge(self.__port.lower()+'_'+'jog_axis_x_velocity','Velocity (mm/s) of device\'s X axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisYVelocity', fallback=False):
            self.__jogAxisYVelocity = Gauge(self.__port.lower()+'_'+'jog_axis_y_velocity','Velocity (mm/s) of device\'s Y axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisZVelocity', fallback=False):
            self.__jogAxisZVelocity = Gauge(self.__port.lower()+'_'+'jog_axis_z_velocity','Velocity (mm/s) of device\'s Z axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisRVelocity', fallback=False):
            self.__jogAxisRVelocity = Gauge(self.__port.lower()+'_'+'jog_axis_r_velocity','Velocity (mm/s) of device\'s R axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisXAcceleration', fallback=False):
            self.__jogAxisXAcceleration = Gauge(self.__port.lower()+'_'+'jog_axis_x_acceleration','Acceleration (mm/s^2) of device\'s X axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisYAcceleration', fallback=False):
            self.__jogAxisYAcceleration = Gauge(self.__port.lower()+'_'+'jog_axis_y_acceleration','Acceleration (mm/s^2) of device\'s Y axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisZAcceleration', fallback=False):
            self.__jogAxisZAcceleration = Gauge(self.__port.lower()+'_'+'jog_axis_z_acceleration','Acceleration (mm/s^2) of device\'s Z axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogAxisRAcceleration', fallback=False):
            self.__jogAxisRAcceleration = Gauge(self.__port.lower()+'_'+'jog_axis_r_acceleration','Acceleration (mm/s^2) of device\'s R axis (cartesian coordinate) in jogging mode')

        if dobotSection.getboolean('JogVelocityRatio', fallback=False):
            self.__jogVelocityRatio = Gauge(self.__port.lower()+'_'+'jog_velocity_ratio','Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode')

        if dobotSection.getboolean('JogAccelerationRatio', fallback=False):
            self.__jogAccelerationRatio = Gauge(self.__port.lower()+'_'+'jog_acceleration_ratio','Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode')

        if dobotSection.getboolean('PtpBaseVelocity', fallback=False):
            self.__ptpBaseVelocity = Gauge(self.__port.lower()+'_'+'ptp_base_velocity','Velocity (°/s) of base joint in point to point mode')

        if dobotSection.getboolean('PtpRearArmVelocity', fallback=False):
            self.__ptpRearArmVelocity = Gauge(self.__port.lower()+'_'+'ptp_rear_arm_velocity','Velocity (°/s) of rear arm joint in point to point mode')

        if dobotSection.getboolean('PtpForearmVelocity', fallback=False):
            self.__ptpForearmVelocity = Gauge(self.__port.lower()+'_'+'ptp_forearm_velocity','Velocity (°/s) of forearm joint in point to point mode')

        if dobotSection.getboolean('PtpEndEffectorVelocity', fallback=False):
            self.__ptpEndEffectorVelocity = Gauge(self.__port.lower()+'_'+'ptp_end_effector_velocity','Velocity (°/s) of end effector joint in point to point mode')

        if dobotSection.getboolean('PtpBaseAcceleration', fallback=False):
            self.__ptpBaseAcceleration = Gauge(self.__port.lower()+'_'+'ptp_base_acceleration','Acceleration (°/s^2) of base joint in point to point mode')

        if dobotSection.getboolean('PtpRearArmAcceleration', fallback=False):
            self.__ptpRearArmAcceleration = Gauge(self.__port.lower()+'_'+'ptp_rear_arm_acceleration','Acceleration (°/s^2) of rear arm joint in point to point mode')

        if dobotSection.getboolean('PtpForearmAcceleration', fallback=False):
            self.__ptpForearmAcceleration = Gauge(self.__port.lower()+'_'+'ptp_forearm_acceleration','Acceleration (°/s^2) of forearm joint in point to point mode')

        if dobotSection.getboolean('PtpEndEffectorAcceleration', fallback=False):
            self.__ptpEndEffectorAcceleration = Gauge(self.__port.lower()+'_'+'ptp_end_effector_acceleration','Acceleration (°/s^2) of end effector joint in point to point mode')

        if dobotSection.getboolean('PtpXYZVelocity', fallback=False):
            self.__ptpXYZVelocity = Gauge(self.__port.lower()+'_'+'ptp_xyz_velocity','Velocity (mm/s) of device\'s X, Y, Z axis (cartesian coordinate) in point to point mode')

        if dobotSection.getboolean('PtpRVelocity', fallback=False):
            self.__ptpRVelocity = Gauge(self.__port.lower()+'_'+'ptp_r_velocity','Velocity (mm/s) of device\'s R axis (cartesian coordinate) in point to point mode')

        if dobotSection.getboolean('PtpXYZAcceleration', fallback=False):
            self.__ptpXYZAcceleration = Gauge(self.__port.lower()+'_'+'ptp_x_y_z_acceleration','Acceleration (mm/s^2) of device\'s X, Y, Z axis (cartesian coordinate) in point to point mode')

        if dobotSection.getboolean('PtpRAcceleration', fallback=False):
            self.__ptpRAcceleration = Gauge(self.__port.lower()+'_'+'ptp_r_acceleration','Acceleration (mm/s^2) of device\'s R axis (cartesian coordinate) in point to point mode')

        if dobotSection.getboolean('PtpVelocityRatio', fallback=False):
            self.__ptpVelocityRatio = Gauge(self.__port.lower()+'_'+'ptp_velocity_ratio','Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode')

        if dobotSection.getboolean('PtpAccelerationRatio', fallback=False):
            self.__ptpAccelerationRatio = Gauge(self.__port.lower()+'_'+'ptp_acceleration_ratio','Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode')

        if dobotSection.getboolean('LiftingHeight', fallback=False):
            self.__liftingHeight = Gauge(self.__port.lower()+'_'+'lifting_height','Lifting height in jump mode')

        if dobotSection.getboolean('HeighLimit', fallback=False):
            self.__heightLimit = Gauge(self.__port.lower()+'_'+'height_limit','Max lifting height in jump mode')

        if dobotSection.getboolean('CpVelocity', fallback=False):
            self.__cpVelocity = Gauge(self.__port.lower()+'_'+'cp_velocity','Velocity (mm/s) in cp mode')

        if dobotSection.getboolean('CpAcceleration', fallback=False):
            self.__cpAcceleration = Gauge(self.__port.lower()+'_'+'cp_acceleration','Acceleration (mm/s^2) in cp mode')

        if dobotSection.getboolean('ArcXYZVelocity', fallback=False):
            self.__arcXYZVelocity = Gauge(self.__port.lower()+'_'+'arc_x_y_z_velocity','Velocity (mm/s) of X, Y, Z axis in arc mode')

        if dobotSection.getboolean('ArcRVelocity', fallback=False):
            self.__arcRVelocity = Gauge(self.__port.lower()+'_'+'arc_r_velocity','Velocity (mm/s) of R axis in arc mode')

        if dobotSection.getboolean('ArcXYZAcceleration', fallback=False):
            self.__arcXYZAcceleration = Gauge(self.__port.lower()+'_'+'arc_x_y_z_acceleration','Acceleration (mm/s^2) of X, Y, Z axis in arc mode')

        if dobotSection.getboolean('ArcRAcceleration', fallback=False):
            self.__arcRAcceleration = Gauge(self.__port.lower()+'_'+'arc_r_acceleration','Acceleration (mm/s^2) of R axis in arc mode')

        if dobotSection.getboolean('AngleStaticErrRear', fallback=False):
            self.__angleStaticErrRear = Gauge(self.__port.lower()+'_'+'angle_static_err_rear','Rear arm angle sensor static error')

        if dobotSection.getboolean('AngleStaticErrFront', fallback=False):
            self.__angleStaticErrFront = Gauge(self.__port.lower()+'_'+'arc_static_err_front','Forearm angle sensor static error')

        if dobotSection.getboolean('AngleCoefRear', fallback=False):
            self.__angleCoefRear = Gauge(self.__port.lower()+'_'+'angle_coef_rear','Rear arm angle sensor linearization parameter')

        if dobotSection.getboolean('AngleCoefFront', fallback=False):
            self.__angleCoefFront = Gauge(self.__port.lower()+'_'+'angle_coef_front','Forearm angle sensor linearization parameter')

        if dobotSection.getboolean('SlidingRailStatus', fallback=False):
            self.__slidingRailStatus = Enum(self.__port.lower()+'_'+'sliding_rail_status','Sliding rail\'s status (enabled/disabled)', states=['enabled','disabled'])

        if dobotSection.getboolean('SlidingRailPose', fallback=False):
            self.__slidingRailPose = Gauge(self.__port.lower()+'_'+'sliding_rail_pose','Sliding rail\'s real-time pose in mm')

        if dobotSection.getboolean('SlidingRailJogVelocity', fallback=False):
            self.__slidingRailJogVelocity = Gauge(self.__port.lower()+'_'+'sliding_rail_jog_velocity','Velocity (mm/s) of sliding rail in jogging mode')

        if dobotSection.getboolean('SlidingRailJogAcceleration', fallback=False):
            self.__slidingRailJogAcceleration = Gauge(self.__port.lower()+'_'+'sliding_rail_jog_acceleration','Acceleration (mm/s^2) of sliding rail in jogging mode')

        if dobotSection.getboolean('SlidingRailPtpVelocity', fallback=False):
            self.__slidingRailPtpVelocity = Gauge(self.__port.lower()+'_'+'sliding_rail_ptp_velocity','Velocity (mm/s) of sliding rail in point to point mode')

        if dobotSection.getboolean('SlidingRailPtpAcceleration', fallback=False):
            self.__slidingRailPtpAcceleration = Gauge(self.__port.lower()+'_'+'sliding_rail_ptp_acceleration','Acceleration (mm/s^2) of sliding rail in point to point mode')

        if dobotSection.getboolean('WifiModuleStatus', fallback=False):
            self.__wifiModuleStatus = Enum(self.__port.lower()+'_'+'wifi_module_status','Wifi module status (enabled/disabled)', states=['enabled','disabled'])

        if dobotSection.getboolean('WifiConnectionStatus', fallback=False):
            self.__wifiConnectionStatus = Enum(self.__port.lower()+'_'+'wifi_connection_status','Wifi connection status (connected/not connected)', states=['enabled','disabled'])

    def __getAlarms(self):
        alarmBytes = dType.GetAlarmsState(self.__api, 10)[0]

        # Convert Bytes to bits (as string for reading)
        bits = ''
        for byte in alarmBytes:
            bits += f'{byte:0>8b}'
            # print(f'{byte:0>8b}', end=' ')

        # If all bits are 0 then device state is clean/safe
        if bits.strip("0") != '':
            for alarm in self.alarms:
                # Get index in 10-base form to check the corresponding bit
                index = int(alarm, 16)
                if bits[index] == '1':
                    self.__alarmsState.state(self.alarms[alarm])

    def fetchData(self):
        global config
        # Switch to desired Dobot device
        dType.ConnectDobot(self.__api, self.__port, 115200)
        dobotSection = config['DOBOT' + ':' + self.__port]

        if dobotSection.getboolean('DeviceTime', fallback=False):
            self.__deviceTime.set(dType.GetDeviceTime(self.__api)[0])

        if dobotSection.getboolean('QueueIndex', fallback=False):
            self.__queueIndex.set(dType.GetQueuedCmdCurrentIndex(self.__api)[0])

        pose = dType.GetPose(self.__api)
        if dobotSection.getboolean('PoseX', fallback=False):
            self.__poseX.set(pose[0])

        if dobotSection.getboolean('PoseY', fallback=False):
            self.__poseY.set(pose[1])

        if dobotSection.getboolean('PoseZ', fallback=False):
            self.__poseZ.set(pose[2])

        if dobotSection.getboolean('PoseR', fallback=False):
            self.__poseR.set(pose[3])

        if dobotSection.getboolean('AngleBase', fallback=False):
            self.__angleBase.set(pose[4])

        if dobotSection.getboolean('AngleRearArm', fallback=False):
            self.__angleRearArm.set(pose[5])

        if dobotSection.getboolean('AngleForearm', fallback=False):
            self.__angleForearm.set(pose[6])

        if dobotSection.getboolean('AngleEndEffector', fallback=False):
            self.__angleEndEffector.set(pose[7])

        if dobotSection.getboolean('AlarmsState', fallback=False):
            self.__getAlarms()

        home = dType.GetHOMEParams(self.__api)
        if dobotSection.getboolean('HomeX', fallback=False):
            self.__homeX.set(home[0])

        if dobotSection.getboolean('HomeY', fallback=False):
            self.__homeY.set(home[1])

        if dobotSection.getboolean('HomeZ', fallback=False):
            self.__homeZ.set(home[2])

        if dobotSection.getboolean('HomeR', fallback=False):
            self.__homeR.set(home[3])

        if dobotSection.getboolean('AutoLevelingResult', fallback=False):
            self.__autoLevelingResult.set(dType.GetAutoLevelingResult(self.__api)[0])

        endEffector = dType.GetEndEffectorParams(self.__api)
        if dobotSection.getboolean('EndEffectorX', fallback=False):
            self.__endEffectorX.set(endEffector[0])

        if dobotSection.getboolean('EndEffectorY', fallback=False):
            self.__endEffectorY.set(endEffector[1])

        if dobotSection.getboolean('EndEffectorZ', fallback=False):
            self.__endEffectorZ.set(endEffector[2])

        if dobotSection.getboolean('LaserStatus', fallback=False):
            if bool(dType.GetEndEffectorLaser(self.__api)[0]):
                self.__laserStatus.state('enabled')
            else:
                self.__laserStatus.state('disabled')

        if dobotSection.getboolean('SuctionCupStatus', fallback=False):
            if bool(dType.GetEndEffectorSuctionCup(self.__api)[0]):
                self.__suctionCupStatus.state('enabled')
            else:
                self.__suctionCupStatus.state('disabled')

        if dobotSection.getboolean('GripperStatus', fallback=False):
            if bool(dType.GetEndEffectorGripper(self.__api)[0]):
                self.__gripperStatus.state('enabled')
            else:
                self.__gripperStatus.state('disabled')

        jogJoints = dType.GetJOGJointParams(self.__api)
        if dobotSection.getboolean('JogBaseVelocity', fallback=False):
            self.__jogBaseVelocity.set(jogJoints[0])

        if dobotSection.getboolean('JogRearArmVelocity', fallback=False):
            self.__jogRearArmVelocity.set(jogJoints[1])

        if dobotSection.getboolean('JogForearmVelocity', fallback=False):
            self.__jogForearmVelocity.set(jogJoints[2])

        if dobotSection.getboolean('JogEndEffectorVelocity', fallback=False):
            self.__jogEndEffectorVelocity.set(jogJoints[3])

        if dobotSection.getboolean('JogBaseAcceleration', fallback=False):
            self.__jogBaseAcceleration.set(jogJoints[4])

        if dobotSection.getboolean('JogRearArmAcceleration', fallback=False):
            self.__jogRearArmAcceleration.set(jogJoints[5])

        if dobotSection.getboolean('JogForearmAcceleration', fallback=False):
            self.__jogForearmAcceleration.set(jogJoints[6])

        if dobotSection.getboolean('JogEndEffectorAcceleration', fallback=False):
            self.__jogEndEffectorAcceleration.set(jogJoints[7])

        jogCoords = dType.GetJOGCoordinateParams(self.__api)
        if dobotSection.getboolean('JogAxisXVelocity', fallback=False):
            self.__jogAxisXVelocity.set(jogCoords[0])

        if dobotSection.getboolean('JogAxisYVelocity', fallback=False):
            self.__jogAxisYVelocity.set(jogCoords[1])

        if dobotSection.getboolean('JogAxisZVelocity', fallback=False):
            self.__jogAxisZVelocity.set(jogCoords[2])

        if dobotSection.getboolean('JogAxisRVelocity', fallback=False):
            self.__jogAxisRVelocity.set(jogCoords[3])

        if dobotSection.getboolean('JogAxisXAcceleration', fallback=False):
            self.__jogAxisXAcceleration.set(jogCoords[4])

        if dobotSection.getboolean('JogAxisYAcceleration', fallback=False):
            self.__jogAxisYAcceleration.set(jogCoords[5])

        if dobotSection.getboolean('JogAxisZAcceleration', fallback=False):
            self.__jogAxisZAcceleration.set(jogCoords[6])

        if dobotSection.getboolean('JogAxisRAcceleration', fallback=False):
            self.__jogAxisRAcceleration.set(jogCoords[7])

        jogCommon = dType.GetJOGCommonParams(self.__api)
        if dobotSection.getboolean('JogVelocityRatio', fallback=False):
            self.__jogVelocityRatio.set(jogCommon[0])

        if dobotSection.getboolean('JogAccelerationRatio', fallback=False):
            self.__jogAccelerationRatio.set(jogCommon[1])

        ptpJoints = dType.GetPTPJointParams(self.__api)
        if dobotSection.getboolean('PtpBaseVelocity', fallback=False):
            self.__ptpBaseVelocity.set(ptpJoints[0])

        if dobotSection.getboolean('PtpRearArmVelocity', fallback=False):
            self.__ptpRearArmVelocity.set(ptpJoints[1])

        if dobotSection.getboolean('PtpForearmVelocity', fallback=False):
            self.__ptpForearmVelocity.set(ptpJoints[2])

        if dobotSection.getboolean('PtpEndEffectorVelocity', fallback=False):
            self.__ptpEndEffectorVelocity.set(ptpJoints[3])

        if dobotSection.getboolean('PtpBaseAcceleration', fallback=False):
            self.__ptpBaseAcceleration.set(ptpJoints[4])

        if dobotSection.getboolean('PtpRearArmAcceleration', fallback=False):
            self.__ptpRearArmAcceleration.set(ptpJoints[5])

        if dobotSection.getboolean('PtpForearmAcceleration', fallback=False):
            self.__ptpForearmAcceleration.set(ptpJoints[6])

        if dobotSection.getboolean('PtpEndEffectorAcceleration', fallback=False):
            self.__ptpEndEffectorAcceleration.set(ptpJoints[7])

        ptpCoords = dType.GetPTPCoordinateParams(self.__api)
        if dobotSection.getboolean('PtpXYZVelocity', fallback=False):
            self.__ptpXYZVelocity.set(ptpCoords[0])

        if dobotSection.getboolean('PtpRVelocity', fallback=False):
            self.__ptpRVelocity.set(ptpCoords[1])

        if dobotSection.getboolean('PtpXYZAcceleration', fallback=False):
            self.__ptpXYZAcceleration.set(ptpCoords[2])

        if dobotSection.getboolean('PtpRAcceleration', fallback=False):
            self.__ptpRAcceleration.set(ptpCoords[3])

        ptpCommon = dType.GetPTPCommonParams(self.__api)
        if dobotSection.getboolean('PtpVelocityRatio', fallback=False):
            self.__ptpVelocityRatio.set(ptpCommon[0])

        if dobotSection.getboolean('PtpAccelerationRatio', fallback=False):
            self.__ptpAccelerationRatio.set(ptpCommon[1])

        ptpJump = dType.GetPTPJumpParams(self.__api)
        if dobotSection.getboolean('LiftingHeight', fallback=False):
            self.__liftingHeight.set(ptpJump[0])

        if dobotSection.getboolean('HeighLimit', fallback=False):
            self.__heightLimit.set(ptpJump[1])

        cp = dType.GetCPParams(self.__api)
        if dobotSection.getboolean('CpVelocity', fallback=False):
            self.__cpVelocity.set(cp[0])

        if dobotSection.getboolean('CpAcceleration', fallback=False):
            self.__cpAcceleration.set(cp[1])

        arc = dType.GetARCParams(self.__api)
        if dobotSection.getboolean('ArcXYZVelocity', fallback=False):
            self.__arcXYZVelocity.set(arc[0])

        if dobotSection.getboolean('ArcRVelocity', fallback=False):
            self.__arcRVelocity.set(arc[1])

        if dobotSection.getboolean('ArcXYZAcceleration', fallback=False):
            self.__arcXYZAcceleration.set(arc[2])

        if dobotSection.getboolean('ArcRAcceleration', fallback=False):
            self.__arcRAcceleration.set(arc[3])

        angleStaticErr = dType.GetAngleSensorStaticError(self.__api)
        if dobotSection.getboolean('AngleStaticErrRear', fallback=False):
            self.__angleStaticErrRear.set(angleStaticErr[0])

        if dobotSection.getboolean('AngleStaticErrFront', fallback=False):
            self.__angleStaticErrFront.set(angleStaticErr[1])

        angleCoef = dType.GetAngleSensorCoef(self.__api)
        if dobotSection.getboolean('AngleCoefRear', fallback=False):
            self.__angleCoefRear.set(angleCoef[0])

        if dobotSection.getboolean('AngleCoefFront', fallback=False):
            self.__angleCoefFront.set(angleCoef[1])

        if dobotSection.getboolean('SlidingRailStatus', fallback=False):
            if bool(dType.GetDeviceWithL(self.__api)[0]):
                self.__slidingRailStatus.state('enabled')
            else:
                self.__slidingRailStatus.state('disabled')

        if dobotSection.getboolean('SlidingRailPose', fallback=False):
            self.__slidingRailPose.set(dType.GetPoseL(self.__api)[0])

        jogRail = dType.GetJOGLParams(self.__api)
        if dobotSection.getboolean('SlidingRailJogVelocity', fallback=False):
            self.__slidingRailJogVelocity.set(jogRail[0])

        if dobotSection.getboolean('SlidingRailJogAcceleration', fallback=False):
            self.__slidingRailJogAcceleration.set(jogRail[1])

        ptpRail = dType.GetPTPLParams(self.__api)
        if dobotSection.getboolean('SlidingRailPtpVelocity', fallback=False):
            self.__slidingRailPtpVelocity.set(ptpRail[0])

        if dobotSection.getboolean('SlidingRailPtpAcceleration', fallback=False):
            self.__slidingRailPtpAcceleration.set(ptpRail[1])


        if dobotSection.getboolean('WifiModuleStatus', fallback=False):
            if bool(dType.GetWIFIConfigMode(self.__api)[0]):
                self.__wifiModuleStatus.state('enabled')
            else:
                self.__wifiModuleStatus.state('disabled')

        if dobotSection.getboolean('WifiConnectionStatus', fallback=False):
            if bool(dType.GetWIFIConnectStatus(self.__api)[0]):
                self.__wifiConnectionStatus.state('enabled')
            else:
                self.__wifiConnectionStatus.state('disabled')


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
        # Connect Dobot
        state = dType.ConnectDobot(api, port, 115200)[0]

        if state == dType.DobotConnect.DobotConnect_NoError:
            print("Dobot Magician at port " + port + " connected succesfully!")
            return DobotMagician(api, port)
        else:
            print("Couldn't connect with a Dobot Magician device at port " + port)
            return None

    def __connectToDevices(self):
        global config

        # Discover through the config which devices should be monitored
        for section in config:
            try:
                part = str(section).split(':')
                name = part[0]
                port = part[1]
            except IndexError:
                print('All device entries should follow this format [DEVICE:PORT]')
                print('For more information use --help')
                exit(6)

            # Depending on the type of device try to connect to it
            if name.lower() == 'dobot':
                dobot = self.__connectDobot(port)
                if dobot is not None:
                    self.__devices.append(dobot)


    def startRoutine(self):
        if len(self.__devices) == 0:
            print("No devices connected to the agent.")
            sys.exit(11)

        print('Connecting to devices listed in agent.conf..')
        self.__connectToDevices()
        print('Starting prometheus server at port ' + str(self.__prometheusPort) + "..")
        start_http_server(self.__prometheusPort)

        print('Monitoring..')
        while (1):
            time.sleep(self.__routineInterval / 1000)
            for device in self.__devices:
                device.fetchData()

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
        config.read('agent.conf')
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
