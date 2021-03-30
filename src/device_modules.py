from prometheus_client import Info, Gauge, Enum
import runtime.DobotDllTypeX as dType
import serial

class Device():
    configValidOptions = []
    configIgnoreValueCheck = []

    def __init__(self, config, port):
        self.port = port
        self.section = config[type(self).__name__ + ':' + self.port]

    #@abstractmethod
    def _connect(self):
        return None

    #@abstractmethod
    def _fetch(self):
        return None

    #@abstractmethod
    def _disconnect(self):
        return None

    def getDeviceName(self):
        return type(self).__name__ + ":" + self.port


class Dobot(Device):
    configValidOptions = ["devicesn","devicename","deviceversion","devicetime","queueindex",
    "posex","posey","posez","poser","anglebase","anglereararm","angleforearm",
    "angleendeffector","alarmsstate","homex","homey","homez","homer","autolevelingresult",
    "endeffectorx","endeffectory","endeffectorz","laserstatus","suctioncupstatus","gripperstatus","jogbasevelocity",
    "jogreararmvelocity","jogforearmvelocity","jogendeffectorvelocity","jogbaseacceleration","jogreararmacceleration",
    "jogforearmacceleration","jogendeffectoracceleration","jogaxisxvelocity","jogaxisyvelocity","jogaxiszvelocity","jogaxisrvelocity","jogaxisxacceleration",
    "jogaxisyacceleration","jogaxiszacceleration","jogaxisracceleration","jogvelocityratio","jogaccelerationratio","ptpbasevelocity","ptpreararmvelocity",
    "ptpforearmvelocity","ptpendeffectorvelocity","ptpbaseacceleration","ptpreararmacceleration","ptpforearmacceleration","ptpendeffectoracceleration","ptpaxisxyzvelocity",
    "ptpaxisrvelocity","ptpaxisxyzacceleration","ptpaxisracceleration","ptpvelocityratio","ptpaccelerationratio","liftingheight","heightlimit",
    "cpvelocity","cpacceleration","arcxyzvelocity","arcrvelocity","arcxyzacceleration","arcracceleration","anglestaticerrrear",
    "anglestaticerrfront","anglecoefrear","anglecoeffront","slidingrailstatus","slidingrailpose","slidingrailjogvelocity","slidingrailjogacceleration",
    "slidingrailptpvelocity","slidingrailptpacceleration","wifimodulestatus","wificonnectionstatus","wifissid","wifipassword","wifiipaddress",
    "wifinetmask","wifigateway","wifidns"]
    configIgnoreValueCheck = []

    deviceInfo = Info('dobot_magician', 'General information about monitored Dobot Magician device', ['device'])
    wifiInfo = Info('wifi', 'Information regarding the device\'s wifi connection', ['device'])
    deviceTime = Gauge('device_time','Device\'s clock/time', ['device'])
    queueIndex = Gauge('queue_index','Current index in command queue', ['device'])
    poseX = Gauge('pose_x','Real-time cartesian coordinate of device\'s X axis', ['device'])
    poseY = Gauge('pose_y','Real-time cartesian coordinate of device\'s Y axis', ['device'])
    poseZ = Gauge('pose_z','Real-time cartesian coordinate of device\'s Z axis', ['device'])
    poseR = Gauge('pose_r','Real-time cartesian coordinate of device\'s R axis', ['device'])
    angleBase = Gauge('angle_base','Base joint angle', ['device'])
    angleRearArm = Gauge('angle_rear_arm','Rear arm joint angle', ['device'])
    angleForearm = Gauge('angle_forearm','Forearm joint angle', ['device'])
    angleEndEffector = Gauge('angle_end_effector','End effector joint angle', ['device'])
    alarmsState = Enum('alarms', 'Device alarms', ['device'], states=list(dType.alarms.values()))
    homeX = Gauge('home_x','Home position for X axis', ['device'])
    homeY = Gauge('home_y','Home position for Y axis', ['device'])
    homeZ = Gauge('home_z','Home position for Z axis', ['device'])
    homeR = Gauge('home_r','Home position for R axis', ['device'])
    autoLevelingResult = Gauge('auto_leveling_result','Automatic leveling precision result', ['device'])
    endEffectorX = Gauge('end_effector_x','X-axis offset of end effector', ['device'])
    endEffectorY = Gauge('end_effector_y','Y-axis offset of end effector', ['device'])
    endEffectorZ = Gauge('end_effector_z','Z-axis offset of end effector', ['device'])
    laserStatus = Enum('laser_status','Status (enabled/disabled) of laser', ['device'], states=['enabled','disabled'])
    suctionCupStatus = Enum('suction_cup_status','Status (enabled/disabled) of suction cup', ['device'], states=['enabled','disabled'])
    gripperStatus = Enum('gripper_status','Status (enabled/disabled) of gripper', ['device'], states=['enabled','disabled'])
    jogBaseVelocity = Gauge('jog_base_velocity','Velocity (°/s) of base joint in jogging mode', ['device'])
    jogRearArmVelocity = Gauge('jog_rear_arm_velocity','Velocity (°/s) of rear arm joint in jogging mode', ['device'])
    jogForearmVelocity = Gauge('jog_forearm_velocity','Velocity (°/s) of forearm joint in jogging mode', ['device'])
    jogEndEffectorVelocity = Gauge('jog_end_effector_velocity','Velocity (°/s) of end effector joint in jogging mode', ['device'])
    jogBaseAcceleration = Gauge('jog_base_acceleration','Acceleration (°/s^2) of base joint in jogging mode', ['device'])
    jogRearArmAcceleration = Gauge('jog_rear_arm_acceleration','Acceleration (°/s^2) of rear arm joint in jogging mode', ['device'])
    jogForearmAcceleration = Gauge('jog_forearm_acceleration','Acceleration (°/s^2) of forearm joint in jogging mode', ['device'])
    jogEndEffectorAcceleration = Gauge('jog_end_effector_acceleration','Acceleration (°/s^2) of end effector joint in jogging mode', ['device'])
    jogAxisXVelocity = Gauge('jog_axis_x_velocity','Velocity (mm/s) of device\'s X axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisYVelocity = Gauge('jog_axis_y_velocity','Velocity (mm/s) of device\'s Y axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisZVelocity = Gauge('jog_axis_z_velocity','Velocity (mm/s) of device\'s Z axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisRVelocity = Gauge('jog_axis_r_velocity','Velocity (mm/s) of device\'s R axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisXAcceleration = Gauge('jog_axis_x_acceleration','Acceleration (mm/s^2) of device\'s X axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisYAcceleration = Gauge('jog_axis_y_acceleration','Acceleration (mm/s^2) of device\'s Y axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisZAcceleration = Gauge('jog_axis_z_acceleration','Acceleration (mm/s^2) of device\'s Z axis (cartesian coordinate) in jogging mode', ['device'])
    jogAxisRAcceleration = Gauge('jog_axis_r_acceleration','Acceleration (mm/s^2) of device\'s R axis (cartesian coordinate) in jogging mode', ['device'])
    jogVelocityRatio = Gauge('jog_velocity_ratio','Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode', ['device'])
    jogAccelerationRatio = Gauge('jog_acceleration_ratio','Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode', ['device'])
    ptpBaseVelocity = Gauge('ptp_base_velocity','Velocity (°/s) of base joint in point to point mode', ['device'])
    ptpRearArmVelocity = Gauge('ptp_rear_arm_velocity','Velocity (°/s) of rear arm joint in point to point mode', ['device'])
    ptpForearmVelocity = Gauge('ptp_forearm_velocity','Velocity (°/s) of forearm joint in point to point mode', ['device'])
    ptpEndEffectorVelocity = Gauge('ptp_end_effector_velocity','Velocity (°/s) of end effector joint in point to point mode', ['device'])
    ptpBaseAcceleration = Gauge('ptp_base_acceleration','Acceleration (°/s^2) of base joint in point to point mode', ['device'])
    ptpRearArmAcceleration = Gauge('ptp_rear_arm_acceleration','Acceleration (°/s^2) of rear arm joint in point to point mode', ['device'])
    ptpForearmAcceleration = Gauge('ptp_forearm_acceleration','Acceleration (°/s^2) of forearm joint in point to point mode', ['device'])
    ptpEndEffectorAcceleration = Gauge('ptp_end_effector_acceleration','Acceleration (°/s^2) of end effector joint in point to point mode', ['device'])
    ptpXYZVelocity = Gauge('ptp_xyz_velocity','Velocity (mm/s) of device\'s X, Y, Z axis (cartesian coordinate) in point to point mode', ['device'])
    ptpRVelocity = Gauge('ptp_r_velocity','Velocity (mm/s) of device\'s R axis (cartesian coordinate) in point to point mode', ['device'])
    ptpXYZAcceleration = Gauge('ptp_x_y_z_acceleration','Acceleration (mm/s^2) of device\'s X, Y, Z axis (cartesian coordinate) in point to point mode', ['device'])
    ptpRAcceleration = Gauge('ptp_r_acceleration','Acceleration (mm/s^2) of device\'s R axis (cartesian coordinate) in point to point mode', ['device'])
    ptpVelocityRatio = Gauge('ptp_velocity_ratio','Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode', ['device'])
    ptpAccelerationRatio = Gauge('ptp_acceleration_ratio','Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode', ['device'])
    liftingHeight = Gauge('lifting_height','Lifting height in jump mode', ['device'])
    heightLimit = Gauge('height_limit','Max lifting height in jump mode', ['device'])
    cpVelocity = Gauge('cp_velocity','Velocity (mm/s) in cp mode', ['device'])
    cpAcceleration = Gauge('cp_acceleration','Acceleration (mm/s^2) in cp mode', ['device'])
    arcXYZVelocity = Gauge('arc_x_y_z_velocity','Velocity (mm/s) of X, Y, Z axis in arc mode', ['device'])
    arcRVelocity = Gauge('arc_r_velocity','Velocity (mm/s) of R axis in arc mode', ['device'])
    arcXYZAcceleration = Gauge('arc_x_y_z_acceleration','Acceleration (mm/s^2) of X, Y, Z axis in arc mode', ['device'])
    arcRAcceleration = Gauge('arc_r_acceleration','Acceleration (mm/s^2) of R axis in arc mode', ['device'])
    angleStaticErrRear = Gauge('angle_static_err_rear','Rear arm angle sensor static error', ['device'])
    angleStaticErrFront = Gauge('arc_static_err_front','Forearm angle sensor static error', ['device'])
    angleCoefRear = Gauge('angle_coef_rear','Rear arm angle sensor linearization parameter', ['device'])
    angleCoefFront = Gauge('angle_coef_front','Forearm angle sensor linearization parameter', ['device'])
    slidingRailStatus = Enum('sliding_rail_status','Sliding rail\'s status (enabled/disabled)', ['device'], states=['enabled','disabled'])
    slidingRailPose = Gauge('sliding_rail_pose','Sliding rail\'s real-time pose in mm', ['device'])
    slidingRailJogVelocity = Gauge('sliding_rail_jog_velocity','Velocity (mm/s) of sliding rail in jogging mode', ['device'])
    slidingRailJogAcceleration = Gauge('sliding_rail_jog_acceleration','Acceleration (mm/s^2) of sliding rail in jogging mode', ['device'])
    slidingRailPtpVelocity = Gauge('sliding_rail_ptp_velocity','Velocity (mm/s) of sliding rail in point to point mode', ['device'])
    slidingRailPtpAcceleration = Gauge('sliding_rail_ptp_acceleration','Acceleration (mm/s^2) of sliding rail in point to point mode', ['device'])
    wifiModuleStatus = Enum('wifi_module_status','Wifi module status (enabled/disabled)', ['device'], states=['enabled','disabled'])
    wifiConnectionStatus = Enum('wifi_connection_status','Wifi connection status (connected/not connected)', ['device'], states=['enabled','disabled'])

    def _connect(self):
        self.api, state = dType.ConnectDobotX(self.port)

        if state[0] == dType.DobotConnect.DobotConnect_NoError:
            self.__prominit()
            return True
        else:
            return False

    def __prominit(self):
        enabledDeviceInfo = {}
        if self.section.getboolean('DeviceSN', fallback=True):
            enabledDeviceInfo["serial_number"] = dType.GetDeviceSN(self.api)[0]
        if self.section.getboolean('DeviceName', fallback=True):
            enabledDeviceInfo["device_name"] = dType.GetDeviceName(self.api)[0]
        if self.section.getboolean('DeviceVersion', fallback=True):
            enabledDeviceInfo["version"] = '.'.join(list(map(str, dType.GetDeviceVersion(self.api))))
        if len(enabledDeviceInfo) > 0:
            Dobot.deviceInfo.labels('dobot_'+self.port).info(enabledDeviceInfo)

        enabledWifiInfo = {}
        if self.section.getboolean('WifiSSID', fallback=False):
            enabledWifiInfo["ssid"] = dType.GetWIFISSID(self.api)[0]
        if self.section.getboolean('WifiPassword', fallback=False):
            enabledWifiInfo["password"] = dType.GetWIFIPassword(self.api)[0]
        if self.section.getboolean('WifiIPAddress', fallback=False):
            enabledWifiInfo["ip_address"] = '.'.join(list(map(str, dType.GetWIFIIPAddress(self.api)[1:])))
        if self.section.getboolean('WifiNetmask', fallback=False):
            enabledWifiInfo["netmask"] = '.'.join(list(map(str, dType.GetWIFINetmask(self.api))))
        if self.section.getboolean('WifiGateway', fallback=False):
            enabledWifiInfo["gateway"] = '.'.join(list(map(str, dType.GetWIFIGateway(self.api))))
        if self.section.getboolean('WifiDNS', fallback=False):
            enabledWifiInfo["dns"] = '.'.join(list(map(str, dType.GetWIFIDNS(self.api))))
        if len(enabledWifiInfo) > 0:
            Dobot.wifiInfo.labels('dobot_'+self.port).info(enabledWifiInfo)

    def _fetch(self):
        if self.section.getboolean('DeviceTime', fallback=False):
            Dobot.deviceTime.labels('dobot_'+self.port).set(dType.GetDeviceTime(self.api)[0])

        if self.section.getboolean('QueueIndex', fallback=False):
            Dobot.queueIndex.labels('dobot_'+self.port).set(dType.GetQueuedCmdCurrentIndex(self.api)[0])

        pose = dType.GetPose(self.api)
        if self.section.getboolean('PoseX', fallback=True):
            Dobot.poseX.labels('dobot_'+self.port).set(pose[0])

        if self.section.getboolean('PoseY', fallback=True):
            Dobot.poseY.labels('dobot_'+self.port).set(pose[1])

        if self.section.getboolean('PoseZ', fallback=True):
            Dobot.poseZ.labels('dobot_'+self.port).set(pose[2])

        if self.section.getboolean('PoseR', fallback=True):
            Dobot.poseR.labels('dobot_'+self.port).set(pose[3])

        if self.section.getboolean('AngleBase', fallback=True):
            Dobot.angleBase.labels('dobot_'+self.port).set(pose[4])

        if self.section.getboolean('AngleRearArm', fallback=True):
            Dobot.angleRearArm.labels('dobot_'+self.port).set(pose[5])

        if self.section.getboolean('AngleForearm', fallback=True):
            Dobot.angleForearm.labels('dobot_'+self.port).set(pose[6])

        if self.section.getboolean('AngleEndEffector', fallback=True):
            Dobot.angleEndEffector.labels('dobot_'+self.port).set(pose[7])

        if self.section.getboolean('AlarmsState', fallback=True):
            for a in dType.GetAlarmsStateX(self.api):
                Dobot.alarmsState.labels('dobot_'+self.port).state(a)

        home = dType.GetHOMEParams(self.api)
        if self.section.getboolean('HomeX', fallback=False):
            Dobot.homeX.labels('dobot_'+self.port).set(home[0])

        if self.section.getboolean('HomeY', fallback=False):
            Dobot.homeY.labels('dobot_'+self.port).set(home[1])

        if self.section.getboolean('HomeZ', fallback=False):
            Dobot.homeZ.labels('dobot_'+self.port).set(home[2])

        if self.section.getboolean('HomeR', fallback=False):
            Dobot.homeR.labels('dobot_'+self.port).set(home[3])

        if self.section.getboolean('AutoLevelingResult', fallback=False):
            Dobot.autoLevelingResult.labels('dobot_'+self.port).set(dType.GetAutoLevelingResult(self.api)[0])

        endEffector = dType.GetEndEffectorParams(self.api)
        if self.section.getboolean('EndEffectorX', fallback=False):
            Dobot.endEffectorX.labels('dobot_'+self.port).set(endEffector[0])

        if self.section.getboolean('EndEffectorY', fallback=False):
            Dobot.endEffectorY.labels('dobot_'+self.port).set(endEffector[1])

        if self.section.getboolean('EndEffectorZ', fallback=False):
            Dobot.endEffectorZ.labels('dobot_'+self.port).set(endEffector[2])

        if self.section.getboolean('LaserStatus', fallback=False):
            if bool(dType.GetEndEffectorLaser(self.api)[0]):
                Dobot.laserStatus.labels('dobot_'+self.port).state('enabled')
            else:
                Dobot.laserStatus.labels('dobot_'+self.port).state('disabled')

        if self.section.getboolean('SuctionCupStatus', fallback=False):
            if bool(dType.GetEndEffectorSuctionCup(self.api)[0]):
                Dobot.suctionCupStatus.labels('dobot_'+self.port).state('enabled')
            else:
                Dobot.suctionCupStatus.labels('dobot_'+self.port).state('disabled')

        if self.section.getboolean('GripperStatus', fallback=False):
            if bool(dType.GetEndEffectorGripper(self.api)[0]):
                Dobot.gripperStatus.labels('dobot_'+self.port).state('enabled')
            else:
                Dobot.gripperStatus.labels('dobot_'+self.port).state('disabled')

        jogJoints = dType.GetJOGJointParams(self.api)
        if self.section.getboolean('JogBaseVelocity', fallback=False):
            Dobot.jogBaseVelocity.labels('dobot_'+self.port).set(jogJoints[0])

        if self.section.getboolean('JogRearArmVelocity', fallback=False):
            Dobot.jogRearArmVelocity.labels('dobot_'+self.port).set(jogJoints[1])

        if self.section.getboolean('JogForearmVelocity', fallback=False):
            Dobot.jogForearmVelocity.labels('dobot_'+self.port).set(jogJoints[2])

        if self.section.getboolean('JogEndEffectorVelocity', fallback=False):
            Dobot.jogEndEffectorVelocity.labels('dobot_'+self.port).set(jogJoints[3])

        if self.section.getboolean('JogBaseAcceleration', fallback=False):
            Dobot.jogBaseAcceleration.labels('dobot_'+self.port).set(jogJoints[4])

        if self.section.getboolean('JogRearArmAcceleration', fallback=False):
            Dobot.jogRearArmAcceleration.labels('dobot_'+self.port).set(jogJoints[5])

        if self.section.getboolean('JogForearmAcceleration', fallback=False):
            Dobot.jogForearmAcceleration.labels('dobot_'+self.port).set(jogJoints[6])

        if self.section.getboolean('JogEndEffectorAcceleration', fallback=False):
            Dobot.jogEndEffectorAcceleration.labels('dobot_'+self.port).set(jogJoints[7])

        jogCoords = dType.GetJOGCoordinateParams(self.api)
        if self.section.getboolean('JogAxisXVelocity', fallback=False):
            Dobot.jogAxisXVelocity.labels('dobot_'+self.port).set(jogCoords[0])

        if self.section.getboolean('JogAxisYVelocity', fallback=False):
            Dobot.jogAxisYVelocity.labels('dobot_'+self.port).set(jogCoords[1])

        if self.section.getboolean('JogAxisZVelocity', fallback=False):
            Dobot.jogAxisZVelocity.labels('dobot_'+self.port).set(jogCoords[2])

        if self.section.getboolean('JogAxisRVelocity', fallback=False):
            Dobot.jogAxisRVelocity.labels('dobot_'+self.port).set(jogCoords[3])

        if self.section.getboolean('JogAxisXAcceleration', fallback=False):
            Dobot.jogAxisXAcceleration.labels('dobot_'+self.port).set(jogCoords[4])

        if self.section.getboolean('JogAxisYAcceleration', fallback=False):
            Dobot.jogAxisYAcceleration.labels('dobot_'+self.port).set(jogCoords[5])

        if self.section.getboolean('JogAxisZAcceleration', fallback=False):
            Dobot.jogAxisZAcceleration.labels('dobot_'+self.port).set(jogCoords[6])

        if self.section.getboolean('JogAxisRAcceleration', fallback=False):
            Dobot.jogAxisRAcceleration.labels('dobot_'+self.port).set(jogCoords[7])

        jogCommon = dType.GetJOGCommonParams(self.api)
        if self.section.getboolean('JogVelocityRatio', fallback=False):
            Dobot.jogVelocityRatio.labels('dobot_'+self.port).set(jogCommon[0])

        if self.section.getboolean('JogAccelerationRatio', fallback=False):
            Dobot.jogAccelerationRatio.labels('dobot_'+self.port).set(jogCommon[1])

        ptpJoints = dType.GetPTPJointParams(self.api)
        if self.section.getboolean('PtpBaseVelocity', fallback=False):
            Dobot.ptpBaseVelocity.labels('dobot_'+self.port).set(ptpJoints[0])

        if self.section.getboolean('PtpRearArmVelocity', fallback=False):
            Dobot.ptpRearArmVelocity.labels('dobot_'+self.port).set(ptpJoints[1])

        if self.section.getboolean('PtpForearmVelocity', fallback=False):
            Dobot.ptpForearmVelocity.labels('dobot_'+self.port).set(ptpJoints[2])

        if self.section.getboolean('PtpEndEffectorVelocity', fallback=False):
            Dobot.ptpEndEffectorVelocity.labels('dobot_'+self.port).set(ptpJoints[3])

        if self.section.getboolean('PtpBaseAcceleration', fallback=False):
            Dobot.ptpBaseAcceleration.labels('dobot_'+self.port).set(ptpJoints[4])

        if self.section.getboolean('PtpRearArmAcceleration', fallback=False):
            Dobot.ptpRearArmAcceleration.labels('dobot_'+self.port).set(ptpJoints[5])

        if self.section.getboolean('PtpForearmAcceleration', fallback=False):
            Dobot.ptpForearmAcceleration.labels('dobot_'+self.port).set(ptpJoints[6])

        if self.section.getboolean('PtpEndEffectorAcceleration', fallback=False):
            Dobot.ptpEndEffectorAcceleration.labels('dobot_'+self.port).set(ptpJoints[7])

        ptpCoords = dType.GetPTPCoordinateParams(self.api)
        if self.section.getboolean('PtpXYZVelocity', fallback=False):
            Dobot.ptpXYZVelocity.labels('dobot_'+self.port).set(ptpCoords[0])

        if self.section.getboolean('PtpRVelocity', fallback=False):
            Dobot.ptpRVelocity.labels('dobot_'+self.port).set(ptpCoords[1])

        if self.section.getboolean('PtpXYZAcceleration', fallback=False):
            Dobot.ptpXYZAcceleration.labels('dobot_'+self.port).set(ptpCoords[2])

        if self.section.getboolean('PtpRAcceleration', fallback=False):
            Dobot.ptpRAcceleration.labels('dobot_'+self.port).set(ptpCoords[3])

        ptpCommon = dType.GetPTPCommonParams(self.api)
        if self.section.getboolean('PtpVelocityRatio', fallback=False):
            Dobot.ptpVelocityRatio.labels('dobot_'+self.port).set(ptpCommon[0])

        if self.section.getboolean('PtpAccelerationRatio', fallback=False):
            Dobot.ptpAccelerationRatio.labels('dobot_'+self.port).set(ptpCommon[1])

        ptpJump = dType.GetPTPJumpParams(self.api)
        if self.section.getboolean('LiftingHeight', fallback=False):
            Dobot.liftingHeight.labels('dobot_'+self.port).set(ptpJump[0])

        if self.section.getboolean('HeighLimit', fallback=False):
            Dobot.heightLimit.labels('dobot_'+self.port).set(ptpJump[1])

        cp = dType.GetCPParams(self.api)
        if self.section.getboolean('CpVelocity', fallback=False):
            Dobot.cpVelocity.labels('dobot_'+self.port).set(cp[0])

        if self.section.getboolean('CpAcceleration', fallback=False):
            Dobot.cpAcceleration.labels('dobot_'+self.port).set(cp[1])

        arc = dType.GetARCParams(self.api)
        if self.section.getboolean('ArcXYZVelocity', fallback=False):
            Dobot.arcXYZVelocity.labels('dobot_'+self.port).set(arc[0])

        if self.section.getboolean('ArcRVelocity', fallback=False):
            Dobot.arcRVelocity.labels('dobot_'+self.port).set(arc[1])

        if self.section.getboolean('ArcXYZAcceleration', fallback=False):
            Dobot.arcXYZAcceleration.labels('dobot_'+self.port).set(arc[2])

        if self.section.getboolean('ArcRAcceleration', fallback=False):
            Dobot.arcRAcceleration.labels('dobot_'+self.port).set(arc[3])

        angleStaticErr = dType.GetAngleSensorStaticError(self.api)
        if self.section.getboolean('AngleStaticErrRear', fallback=False):
            Dobot.angleStaticErrRear.labels('dobot_'+self.port).set(angleStaticErr[0])

        if self.section.getboolean('AngleStaticErrFront', fallback=False):
            Dobot.angleStaticErrFront.labels('dobot_'+self.port).set(angleStaticErr[1])

        angleCoef = dType.GetAngleSensorCoef(self.api)
        if self.section.getboolean('AngleCoefRear', fallback=False):
            Dobot.angleCoefRear.labels('dobot_'+self.port).set(angleCoef[0])

        if self.section.getboolean('AngleCoefFront', fallback=False):
            Dobot.angleCoefFront.labels('dobot_'+self.port).set(angleCoef[1])

        if self.section.getboolean('SlidingRailStatus', fallback=False):
            if bool(dType.GetDeviceWithL(self.api)[0]):
                Dobot.slidingRailStatus.labels('dobot_'+self.port).state('enabled')
            else:
                Dobot.slidingRailStatus.labels('dobot_'+self.port).state('disabled')

        if self.section.getboolean('SlidingRailPose', fallback=False):
            Dobot.slidingRailPose.labels('dobot_'+self.port).set(dType.GetPoseL(self.api)[0])

        jogRail = dType.GetJOGLParams(self.api)
        if self.section.getboolean('SlidingRailJogVelocity', fallback=False):
            Dobot.slidingRailJogVelocity.labels('dobot_'+self.port).set(jogRail[0])

        if self.section.getboolean('SlidingRailJogAcceleration', fallback=False):
            Dobot.slidingRailJogAcceleration.labels('dobot_'+self.port).set(jogRail[1])

        ptpRail = dType.GetPTPLParams(self.api)
        if self.section.getboolean('SlidingRailPtpVelocity', fallback=False):
            Dobot.slidingRailPtpVelocity.labels('dobot_'+self.port).set(ptpRail[0])

        if self.section.getboolean('SlidingRailPtpAcceleration', fallback=False):
            Dobot.slidingRailPtpAcceleration.labels('dobot_'+self.port).set(ptpRail[1])


        if self.section.getboolean('WifiModuleStatus', fallback=False):
            if bool(dType.GetWIFIConfigMode(self.api)[0]):
                Dobot.wifiModuleStatus.labels('dobot_'+self.port).state('enabled')
            else:
                Dobot.wifiModuleStatus.labels('dobot_'+self.port).state('disabled')

        if self.section.getboolean('WifiConnectionStatus', fallback=False):
            if bool(dType.GetWIFIConnectStatus(self.api)[0]):
                Dobot.wifiConnectionStatus.labels('dobot_'+self.port).state('enabled')
            else:
                Dobot.wifiConnectionStatus.labels('dobot_'+self.port).state('disabled')

    def _disconnect(self):
        dType.DisconnectDobotX(self.api)

class Jevois(Device):
    configValidOptions = ["objects","objectidentified","objectlocation","objectsize"]
    configIgnoreValueCheck = ["objects"]

    objectLocationX = Gauge('object_location_x', 'Identified object\'s x position', ['device'])
    objectLocationY = Gauge('object_location_y', 'Identified object\'s y position', ['device'])
    objectLocationZ = Gauge('object_location_z', 'Identified object\'s Z position', ['device'])
    objectSize = Gauge('object_size','Identified object\'s size', ['device'])

    def _connect(self):
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=1)
            self.__prominit()
            return True
        except Exception as e:
            print("Couldn't connect with Jevois Camera device at port " + self.port + " (" + str(e) + ")")
            return False

    def __prominit(self):
        if self.section.getboolean('ObjectIdentified', fallback=True):
            if self.section.get('objects') is not None:
                self.objects = self.section["objects"].split()
                self.objectIdentified = Enum('object_identified_by_'+self.port, 'Object Identified', states=self.objects)
            else:
                print('The \"objects\" list is necessary for monitoring identified objects')
                print('Skipping monitoring objects identified for Jevois:' + self.port)

    def _fetch(self):
        line = self.serial.readline().rstrip().decode()
        tok = line.split()

        # Abort fetching if timeout or malformed line
        if len(tok) < 1: return

        serstyle = tok[0][0]
        dimension = tok[0][1]

        # If the serstyle is not Normal (thus it is unsupported by the module)
        if (serstyle != 'N'): return

        if dimension == '1' and len(tok) != 4: return
        if dimension == '2' and len(tok) != 6: return
        if dimension == '3' and len(tok) != 8: return

        if self.section.getboolean('ObjectIdentified', fallback=True):
            if self.objects is not None and tok[1] in self.objects:
                self.objectIdentified.state(tok[1])

        if self.section.getboolean('ObjectLocation', fallback=True):
            Jevois.objectLocationX.labels('jevois_'+self.port).set(float(tok[2]))

            if int(dimension) > 1:
                Jevois.objectLocationY.labels('jevois_'+self.port).set(float(tok[3]))

            if int(dimension) == 3:
                Jevois.objectLOcationZ.labels('jevois_'+self.port).set(float(tok[4]))

        if self.section.getboolean('ObjectSize', fallback=False):
            if dimension == '1':
                Jevois.objectSize.labels('jevois_'+self.port).set(float(tok[3]))
            elif dimension == '2':
                Jevois.objectSize.labels('jevois_'+self.port).set(float(tok[4])*float(tok[5]))
            elif dimension == '3':
                Jevois.objectSize.labels('jevois_'+self.port).set(float(tok[5])*float(tok[6])*float(tok[7]))

    def _disconnect(self):
        self.serial.close()
