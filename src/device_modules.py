from abc import ABC, abstractmethod
from prometheus_client import Info, Gauge, Enum
import runtime.DobotDllTypeX as dTypeX
import serial

class Device(ABC):
    options = {"timeout":0} # Default device options/attributes

    def __init__(self, config_section, port, host):
        self.section = config_section
        self.port = port
        self.host = host
        self.type = type(self).__name__
        self.id = self.type + ":" + self.port

        self.timeout = self.section.getint("timeout", fallback=Device.options["timeout"])
        if self.timeout < Device.options["timeout"]: self.timeout = Device.options["timeout"]

        activeCounter = 0
        for key in type(self).options:
            if isinstance(type(self).options[key], bool) and self.isEnabled(key):
                activeCounter += 1

        self.activeAttr = activeCounter

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def fetch(self):
        pass

    @abstractmethod
    def disconnect(self):
        pass

    def isEnabled(self, attr):
        if not isinstance(type(self).options[attr], bool):
            raise Exception("\"" + attr + "\" attribute is not a monitoring option.")
        return self.section.getboolean(attr, fallback=type(self).options[attr])

    def isCallEnabled(self, attrList):
        for attr in attrList:
            if self.isEnabled(attr):
                return True

        return False

class Dobot(Device):
    options = {"devicesn":True,"devicename":True,"deviceversion":True,"devicetime":False,"queueindex":False,
    "posex":True,"posey":True,"posez":True,"poser":True,"anglebase":True,"anglereararm":True,"angleforearm":True,
    "angleendeffector":True,"alarmsstate":True,"homex":False,"homey":False,"homez":False,"homer":False,
    "endeffectorx":False,"endeffectory":False,"endeffectorz":False,"laserstatus":False,"suctioncupstatus":False,"gripperstatus":False,"jogbasevelocity":False,
    "jogreararmvelocity":False,"jogforearmvelocity":False,"jogendeffectorvelocity":False,"jogbaseacceleration":False,"jogreararmacceleration":False,
    "jogforearmacceleration":False,"jogendeffectoracceleration":False,"jogaxisxvelocity":False,"jogaxisyvelocity":False,"jogaxiszvelocity":False,
    "jogaxisrvelocity":False,"jogaxisxacceleration":False,"jogaxisyacceleration":False,"jogaxiszacceleration":False,"jogaxisracceleration":False,
    "jogvelocityratio":False,"jogaccelerationratio":False,"ptpbasevelocity":False,"ptpreararmvelocity":False,
    "ptpforearmvelocity":False,"ptpendeffectorvelocity":False,"ptpbaseacceleration":False,"ptpreararmacceleration":False,
    "ptpforearmacceleration":False,"ptpendeffectoracceleration":False,"ptpaxisxyzvelocity":False,
    "ptpaxisrvelocity":False,"ptpaxisxyzacceleration":False,"ptpaxisracceleration":False,"ptpvelocityratio":False,
    "ptpaccelerationratio":False,"liftingheight":False,"heightlimit":False,
    "cpvelocity":False,"cpacceleration":False,"arcxyzvelocity":False,"arcrvelocity":False,
    "arcxyzacceleration":False,"arcracceleration":False,"anglestaticerrrear":False,
    "anglestaticerrfront":False,"anglecoefrear":False,"anglecoeffront":False,"slidingrailstatus":False,
    "slidingrailpose":False,"slidingrailjogvelocity":False,"slidingrailjogacceleration":False,
    "slidingrailptpvelocity":False,"slidingrailptpacceleration":False,"wifimodulestatus":False,
    "wificonnectionstatus":False,"wifissid":False,"wifipassword":False,"wifiipaddress":False,
    "wifinetmask":False,"wifigateway":False,"wifidns":False}

    deviceInfo = Info("dobot_magician", "General information about monitored Dobot Magician device", ["device_id","device_type","station"])
    wifiInfo = Info("wifi", "Information regarding the device's wifi connection", ["device_id","device_type","station"])
    deviceTime = Gauge("device_time","Device's clock/time", ["device_id","device_type","station"])
    queueIndex = Gauge("queue_index","Current index in command queue", ["device_id","device_type","station"])
    poseX = Gauge("pose_x","Real-time cartesian coordinate of device's X axis", ["device_id","device_type","station"])
    poseY = Gauge("pose_y","Real-time cartesian coordinate of device's Y axis", ["device_id","device_type","station"])
    poseZ = Gauge("pose_z","Real-time cartesian coordinate of device's Z axis", ["device_id","device_type","station"])
    poseR = Gauge("pose_r","Real-time cartesian coordinate of device's R axis", ["device_id","device_type","station"])
    angleBase = Gauge("angle_base","Base joint angle", ["device_id","device_type","station"])
    angleRearArm = Gauge("angle_rear_arm","Rear arm joint angle", ["device_id","device_type","station"])
    angleForearm = Gauge("angle_forearm","Forearm joint angle", ["device_id","device_type","station"])
    angleEndEffector = Gauge("angle_end_effector","End effector joint angle", ["device_id","device_type","station"])
    alarmsState = Enum("alarms_state", "Device alarms state", ["device_id","device_type","station"], states=dTypeX.alarmStates)
    homeX = Gauge("home_x","Home position for X axis", ["device_id","device_type","station"])
    homeY = Gauge("home_y","Home position for Y axis", ["device_id","device_type","station"])
    homeZ = Gauge("home_z","Home position for Z axis", ["device_id","device_type","station"])
    homeR = Gauge("home_r","Home position for R axis", ["device_id","device_type","station"])
    endEffectorX = Gauge("end_effector_x","X-axis offset of end effector", ["device_id","device_type","station"])
    endEffectorY = Gauge("end_effector_y","Y-axis offset of end effector", ["device_id","device_type","station"])
    endEffectorZ = Gauge("end_effector_z","Z-axis offset of end effector", ["device_id","device_type","station"])
    laserStatus = Enum("laser_status","Status (enabled/disabled) of laser", ["device_id","device_type","station"], states=["enabled","disabled"])
    suctionCupStatus = Enum("suction_cup_status","Status (enabled/disabled) of suction cup", ["device_id","device_type","station"], states=["enabled","disabled"])
    gripperStatus = Enum("gripper_status","Status (enabled/disabled) of gripper", ["device_id","device_type","station"], states=["enabled","disabled"])
    jogBaseVelocity = Gauge("jog_base_velocity","Velocity (°/s) of base joint in jogging mode", ["device_id","device_type","station"])
    jogRearArmVelocity = Gauge("jog_rear_arm_velocity","Velocity (°/s) of rear arm joint in jogging mode", ["device_id","device_type","station"])
    jogForearmVelocity = Gauge("jog_forearm_velocity","Velocity (°/s) of forearm joint in jogging mode", ["device_id","device_type","station"])
    jogEndEffectorVelocity = Gauge("jog_end_effector_velocity","Velocity (°/s) of end effector joint in jogging mode", ["device_id","device_type","station"])
    jogBaseAcceleration = Gauge("jog_base_acceleration","Acceleration (°/s^2) of base joint in jogging mode", ["device_id","device_type","station"])
    jogRearArmAcceleration = Gauge("jog_rear_arm_acceleration","Acceleration (°/s^2) of rear arm joint in jogging mode", ["device_id","device_type","station"])
    jogForearmAcceleration = Gauge("jog_forearm_acceleration","Acceleration (°/s^2) of forearm joint in jogging mode", ["device_id","device_type","station"])
    jogEndEffectorAcceleration = Gauge("jog_end_effector_acceleration","Acceleration (°/s^2) of end effector joint in jogging mode", ["device_id","device_type","station"])
    jogAxisXVelocity = Gauge("jog_axis_x_velocity","Velocity (mm/s) of device's X axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisYVelocity = Gauge("jog_axis_y_velocity","Velocity (mm/s) of device's Y axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisZVelocity = Gauge("jog_axis_z_velocity","Velocity (mm/s) of device's Z axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisRVelocity = Gauge("jog_axis_r_velocity","Velocity (mm/s) of device's R axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisXAcceleration = Gauge("jog_axis_x_acceleration","Acceleration (mm/s^2) of device's X axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisYAcceleration = Gauge("jog_axis_y_acceleration","Acceleration (mm/s^2) of device's Y axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisZAcceleration = Gauge("jog_axis_z_acceleration","Acceleration (mm/s^2) of device's Z axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogAxisRAcceleration = Gauge("jog_axis_r_acceleration","Acceleration (mm/s^2) of device's R axis (cartesian coordinate) in jogging mode", ["device_id","device_type","station"])
    jogVelocityRatio = Gauge("jog_velocity_ratio","Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode", ["device_id","device_type","station"])
    jogAccelerationRatio = Gauge("jog_acceleration_ratio","Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode", ["device_id","device_type","station"])
    ptpBaseVelocity = Gauge("ptp_base_velocity","Velocity (°/s) of base joint in point to point mode", ["device_id","device_type","station"])
    ptpRearArmVelocity = Gauge("ptp_rear_arm_velocity","Velocity (°/s) of rear arm joint in point to point mode", ["device_id","device_type","station"])
    ptpForearmVelocity = Gauge("ptp_forearm_velocity","Velocity (°/s) of forearm joint in point to point mode", ["device_id","device_type","station"])
    ptpEndEffectorVelocity = Gauge("ptp_end_effector_velocity","Velocity (°/s) of end effector joint in point to point mode", ["device_id","device_type","station"])
    ptpBaseAcceleration = Gauge("ptp_base_acceleration","Acceleration (°/s^2) of base joint in point to point mode", ["device_id","device_type","station"])
    ptpRearArmAcceleration = Gauge("ptp_rear_arm_acceleration","Acceleration (°/s^2) of rear arm joint in point to point mode", ["device_id","device_type","station"])
    ptpForearmAcceleration = Gauge("ptp_forearm_acceleration","Acceleration (°/s^2) of forearm joint in point to point mode", ["device_id","device_type","station"])
    ptpEndEffectorAcceleration = Gauge("ptp_end_effector_acceleration","Acceleration (°/s^2) of end effector joint in point to point mode", ["device_id","device_type","station"])
    ptpAxisXYZVelocity = Gauge("ptp_axis_xyz_velocity","Velocity (mm/s) of device's X, Y, Z axis (cartesian coordinate) in point to point mode", ["device_id","device_type","station"])
    ptpAxisRVelocity = Gauge("ptp_axis_r_velocity","Velocity (mm/s) of device's R axis (cartesian coordinate) in point to point mode", ["device_id","device_type","station"])
    ptpAxisXYZAcceleration = Gauge("ptp_axis_x_y_z_acceleration","Acceleration (mm/s^2) of device's X, Y, Z axis (cartesian coordinate) in point to point mode", ["device_id","device_type","station"])
    ptpAxisRAcceleration = Gauge("ptp_axis_r_acceleration","Acceleration (mm/s^2) of device's R axis (cartesian coordinate) in point to point mode", ["device_id","device_type","station"])
    ptpVelocityRatio = Gauge("ptp_velocity_ratio","Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode", ["device_id","device_type","station"])
    ptpAccelerationRatio = Gauge("ptp_acceleration_ratio","Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode", ["device_id","device_type","station"])
    liftingHeight = Gauge("lifting_height","Lifting height in jump mode", ["device_id","device_type","station"])
    heightLimit = Gauge("height_limit","Max lifting height in jump mode", ["device_id","device_type","station"])
    cpVelocity = Gauge("cp_velocity","Velocity (mm/s) in cp mode", ["device_id","device_type","station"])
    cpAcceleration = Gauge("cp_acceleration","Acceleration (mm/s^2) in cp mode", ["device_id","device_type","station"])
    arcXYZVelocity = Gauge("arc_x_y_z_velocity","Velocity (mm/s) of X, Y, Z axis in arc mode", ["device_id","device_type","station"])
    arcRVelocity = Gauge("arc_r_velocity","Velocity (mm/s) of R axis in arc mode", ["device_id","device_type","station"])
    arcXYZAcceleration = Gauge("arc_x_y_z_acceleration","Acceleration (mm/s^2) of X, Y, Z axis in arc mode", ["device_id","device_type","station"])
    arcRAcceleration = Gauge("arc_r_acceleration","Acceleration (mm/s^2) of R axis in arc mode", ["device_id","device_type","station"])
    angleStaticErrRear = Gauge("angle_static_err_rear","Rear arm angle sensor static error", ["device_id","device_type","station"])
    angleStaticErrFront = Gauge("arc_static_err_front","Forearm angle sensor static error", ["device_id","device_type","station"])
    angleCoefRear = Gauge("angle_coef_rear","Rear arm angle sensor linearization parameter", ["device_id","device_type","station"])
    angleCoefFront = Gauge("angle_coef_front","Forearm angle sensor linearization parameter", ["device_id","device_type","station"])
    slidingRailStatus = Enum("sliding_rail_status","Sliding rail's status (enabled/disabled)", ["device_id","device_type","station"], states=["enabled","disabled"])
    slidingRailPose = Gauge("sliding_rail_pose","Sliding rail's real-time pose in mm", ["device_id","device_type","station"])
    slidingRailJogVelocity = Gauge("sliding_rail_jog_velocity","Velocity (mm/s) of sliding rail in jogging mode", ["device_id","device_type","station"])
    slidingRailJogAcceleration = Gauge("sliding_rail_jog_acceleration","Acceleration (mm/s^2) of sliding rail in jogging mode", ["device_id","device_type","station"])
    slidingRailPtpVelocity = Gauge("sliding_rail_ptp_velocity","Velocity (mm/s) of sliding rail in point to point mode", ["device_id","device_type","station"])
    slidingRailPtpAcceleration = Gauge("sliding_rail_ptp_acceleration","Acceleration (mm/s^2) of sliding rail in point to point mode", ["device_id","device_type","station"])
    wifiModuleStatus = Enum("wifi_module_status","Wifi module status (enabled/disabled)", ["device_id","device_type","station"], states=["enabled","disabled"])
    wifiConnectionStatus = Enum("wifi_connection_status","Wifi connection status (connected/not connected)", ["device_id","device_type","station"], states=["enabled","disabled"])

    def connect(self):
        stateInfo = {1:"Not Found", 2:"Occupied"}

        try:
            self.api, state = dTypeX.ConnectDobotX(self.port)
            if state[0] == dTypeX.DobotConnect.DobotConnect_NoError:
                self.__initialize()
            else:
                raise Exception(stateInfo[state[0]])
        except Exception as e:
            raise Exception(str(e))

    def __initialize(self):
        enabledDeviceInfo = {}
        if self.isEnabled("devicesn"):
            enabledDeviceInfo["serial"] = dTypeX.GetDeviceSN(self.api)[0]
        if self.isEnabled("devicename"):
            enabledDeviceInfo["name"] = dTypeX.GetDeviceName(self.api)[0]
        if self.isEnabled("deviceversion"):
            enabledDeviceInfo["version"] = ".".join(list(map(str, dTypeX.GetDeviceVersion(self.api))))
        if len(enabledDeviceInfo) > 0:
            Dobot.deviceInfo.labels(device_id=self.id, device_type=self.type, station=self.host).info(enabledDeviceInfo)

        enabledWifiInfo = {}
        if self.isEnabled("wifissid"):
            enabledWifiInfo["ssid"] = dTypeX.GetWIFISSID(self.api)[0]
        if self.isEnabled("wifipassword"):
            enabledWifiInfo["password"] = dTypeX.GetWIFIPassword(self.api)[0]
        if self.isEnabled("wifiipaddress"):
            enabledWifiInfo["ip_address"] = ".".join(list(map(str, dTypeX.GetWIFIIPAddress(self.api)[1:])))
        if self.isEnabled("wifinetmask"):
            enabledWifiInfo["netmask"] = ".".join(list(map(str, dTypeX.GetWIFINetmask(self.api))))
        if self.isEnabled("wifigateway"):
            enabledWifiInfo["gateway"] = ".".join(list(map(str, dTypeX.GetWIFIGateway(self.api))))
        if self.isEnabled("wifidns"):
            enabledWifiInfo["dns"] = ".".join(list(map(str, dTypeX.GetWIFIDNS(self.api))))
        if len(enabledWifiInfo) > 0:
            Dobot.wifiInfo.labels(device_id=self.id, device_type=self.type, station=self.host).info(enabledWifiInfo)

        self.GetPose = self.isCallEnabled(["posex","posey","posez","poser","anglebase","anglereararm","angleforearm","angleendeffector"])
        self.GetHomeParams = self.isCallEnabled(["homex","homey","homez","homer"])
        self.GetEndEffectorParams = self.isCallEnabled(["endeffectorx","endeffectory","endeffectorz"])
        self.GetJOGGointParams = self.isCallEnabled(["jogbasevelocity","jogreararmvelocity","jogforearmvelocity","jogendeffectorvelocity",
        "jogbaseacceleration","jogreararmacceleration","jogforearmacceleration","jogendeffectoracceleration"])
        self.GetJOGCoordinateParams = self.isCallEnabled(["jogaxisxvelocity","jogaxisyvelocity","jogaxiszvelocity","jogaxisrvelocity",
        "jogaxisxacceleration","jogaxisyacceleration","jogaxiszacceleration","jogaxisracceleration"])
        self.GetJOGCommonParams = self.isCallEnabled(["jogvelocityratio","jogaccelerationratio"])
        self.GetPTPJointParams = self.isCallEnabled(["ptpbasevelocity","ptpreararmvelocity","ptpforearmvelocity","ptpendeffectorvelocity",
        "ptpbaseacceleration","ptpreararmacceleration","ptpforearmacceleration","ptpendeffectoracceleration"])
        self.GetPTPCoordinateParams = self.isCallEnabled(["ptpaxisxyzvelocity","ptpaxisrvelocity","ptpaxisxyzacceleration","ptpaxisracceleration"])
        self.GetPTPCommonParams = self.isCallEnabled(["ptpvelocityratio","ptpaccelerationratio"])
        self.GetPTPJumpParams = self.isCallEnabled(["liftingheight","heightlimit"])
        self.GetCPParams = self.isCallEnabled(["cpvelocity","cpacceleration"])
        self.GetARCParams = self.isCallEnabled(["arcxyzvelocity","arcrvelocity","arcxyzacceleration","arcracceleration"])
        self.GetAngleSensorStaticError = self.isCallEnabled(["anglestaticerrrear","anglestaticerrfront"])
        self.GetAngleSensorCoef = self.isCallEnabled(["anglecoefrear","anglecoeffront"])
        self.GetJOGLParams = self.isCallEnabled(["slidingrailjogvelocity","slidingrailjogacceleration"])
        self.GetPTPLParams = self.isCallEnabled(["slidingrailptpvelocity","slidingrailptpacceleration"])

    def fetch(self):
        if self.isEnabled("devicetime"):
            Dobot.deviceTime.labels(device_id=self.id, device_type=self.type, station=self.host).set(dTypeX.GetDeviceTime(self.api)[0])

        if self.isEnabled("queueindex"):
            Dobot.queueIndex.labels(device_id=self.id, device_type=self.type, station=self.host).set(dTypeX.GetQueuedCmdCurrentIndex(self.api)[0])

        if self.GetPose:
            pose = dTypeX.GetPose(self.api)
            if self.isEnabled("posex"):
                Dobot.poseX.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[0])

            if self.isEnabled("posey"):
                Dobot.poseY.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[1])

            if self.isEnabled("posez"):
                Dobot.poseZ.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[2])

            if self.isEnabled("poser"):
                Dobot.poseR.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[3])

            if self.isEnabled("anglebase"):
                Dobot.angleBase.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[4])

            if self.isEnabled("anglereararm"):
                Dobot.angleRearArm.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[5])

            if self.isEnabled("angleforearm"):
                Dobot.angleForearm.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[6])

            if self.isEnabled("angleendeffector"):
                Dobot.angleEndEffector.labels(device_id=self.id, device_type=self.type, station=self.host).set(pose[7])

        if self.isEnabled("alarmsstate"):
            alarmsList = dTypeX.GetAlarmsStateX(self.api)
            if len(alarmsList) == 0:
                Dobot.alarmsState.labels(device_id=self.id, device_type=self.type, station=self.host).state("clear")
            else:
                for a in alarmsList:
                    Dobot.alarmsState.labels(device_id=self.id, device_type=self.type, station=self.host).state(a)

        if self.GetHomeParams:
            home = dTypeX.GetHOMEParams(self.api)
            if self.isEnabled("homex"):
                Dobot.homeX.labels(device_id=self.id, device_type=self.type, station=self.host).set(home[0])

            if self.isEnabled("homey"):
                Dobot.homeY.labels(device_id=self.id, device_type=self.type, station=self.host).set(home[1])

            if self.isEnabled("homez"):
                Dobot.homeZ.labels(device_id=self.id, device_type=self.type, station=self.host).set(home[2])

            if self.isEnabled("homer"):
                Dobot.homeR.labels(device_id=self.id, device_type=self.type, station=self.host).set(home[3])

        if self.GetEndEffectorParams:
            endEffector = dTypeX.GetEndEffectorParams(self.api)
            if self.isEnabled("endeffectorx"):
                Dobot.endEffectorX.labels(device_id=self.id, device_type=self.type, station=self.host).set(endEffector[0])

            if self.isEnabled("endeffectory"):
                Dobot.endEffectorY.labels(device_id=self.id, device_type=self.type, station=self.host).set(endEffector[1])

            if self.isEnabled("endeffectorz"):
                Dobot.endEffectorZ.labels(device_id=self.id, device_type=self.type, station=self.host).set(endEffector[2])

        if self.isEnabled("laserstatus"):
            if bool(dTypeX.GetEndEffectorLaser(self.api)[0]):
                Dobot.laserStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("enabled")
            else:
                Dobot.laserStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("disabled")

        if self.isEnabled("suctioncupstatus"):
            if bool(dTypeX.GetEndEffectorSuctionCup(self.api)[0]):
                Dobot.suctionCupStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("enabled")
            else:
                Dobot.suctionCupStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("disabled")

        if self.isEnabled("gripperstatus"):
            if bool(dTypeX.GetEndEffectorGripper(self.api)[0]):
                Dobot.gripperStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("enabled")
            else:
                Dobot.gripperStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("disabled")

        if self.GetJOGGointParams:
            jogJoints = dTypeX.GetJOGJointParams(self.api)
            if self.isEnabled("jogbasevelocity"):
                Dobot.jogBaseVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[0])

            if self.isEnabled("jogreararmvelocity"):
                Dobot.jogRearArmVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[1])

            if self.isEnabled("jogforearmvelocity"):
                Dobot.jogForearmVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[2])

            if self.isEnabled("jogendeffectorvelocity"):
                Dobot.jogEndEffectorVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[3])

            if self.isEnabled("jogbaseacceleration"):
                Dobot.jogBaseAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[4])

            if self.isEnabled("jogreararmacceleration"):
                Dobot.jogRearArmAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[5])

            if self.isEnabled("jogforearmacceleration"):
                Dobot.jogForearmAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[6])

            if self.isEnabled("jogendeffectoracceleration"):
                Dobot.jogEndEffectorAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogJoints[7])

        if self.GetJOGCoordinateParams:
            jogCoords = dTypeX.GetJOGCoordinateParams(self.api)
            if self.isEnabled("jogaxisxvelocity"):
                Dobot.jogAxisXVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[0])

            if self.isEnabled("jogaxisyvelocity"):
                Dobot.jogAxisYVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[1])

            if self.isEnabled("jogaxiszvelocity"):
                Dobot.jogAxisZVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[2])

            if self.isEnabled("jogaxisrvelocity"):
                Dobot.jogAxisRVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[3])

            if self.isEnabled("jogaxisxacceleration"):
                Dobot.jogAxisXAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[4])

            if self.isEnabled("jogaxisyacceleration"):
                Dobot.jogAxisYAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[5])

            if self.isEnabled("jogaxiszacceleration"):
                Dobot.jogAxisZAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[6])

            if self.isEnabled("jogaxisracceleration"):
                Dobot.jogAxisRAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCoords[7])

        if self.GetJOGCommonParams:
            jogCommon = dTypeX.GetJOGCommonParams(self.api)
            if self.isEnabled("jogvelocityratio"):
                Dobot.jogVelocityRatio.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCommon[0])

            if self.isEnabled("jogaccelerationratio"):
                Dobot.jogAccelerationRatio.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogCommon[1])

        if self.GetPTPJointParams:
            ptpJoints = dTypeX.GetPTPJointParams(self.api)
            if self.isEnabled("ptpbasevelocity"):
                Dobot.ptpBaseVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[0])

            if self.isEnabled("ptpreararmvelocity"):
                Dobot.ptpRearArmVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[1])

            if self.isEnabled("ptpforearmvelocity"):
                Dobot.ptpForearmVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[2])

            if self.isEnabled("ptpendeffectorvelocity"):
                Dobot.ptpEndEffectorVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[3])

            if self.isEnabled("ptpbaseacceleration"):
                Dobot.ptpBaseAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[4])

            if self.isEnabled("ptpreararmacceleration"):
                Dobot.ptpRearArmAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[5])

            if self.isEnabled("ptpforearmacceleration"):
                Dobot.ptpForearmAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[6])

            if self.isEnabled("ptpendeffectoracceleration"):
                Dobot.ptpEndEffectorAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJoints[7])

        if self.GetPTPCoordinateParams:
            ptpCoords = dTypeX.GetPTPCoordinateParams(self.api)
            if self.isEnabled("ptpaxisxyzvelocity"):
                Dobot.ptpAxisXYZVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpCoords[0])

            if self.isEnabled("ptpaxisrvelocity"):
                Dobot.ptpAxisRVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpCoords[1])

            if self.isEnabled("ptpaxisxyzacceleration"):
                Dobot.ptpAxisXYZAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpCoords[2])

            if self.isEnabled("ptpaxisracceleration"):
                Dobot.ptpAxisRAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpCoords[3])

        if self.GetPTPCommonParams:
            ptpCommon = dTypeX.GetPTPCommonParams(self.api)
            if self.isEnabled("ptpvelocityratio"):
                Dobot.ptpVelocityRatio.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpCommon[0])

            if self.isEnabled("ptpaccelerationratio"):
                Dobot.ptpAccelerationRatio.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpCommon[1])

        if self.GetPTPJumpParams:
            ptpJump = dTypeX.GetPTPJumpParams(self.api)
            if self.isEnabled("liftingheight"):
                Dobot.liftingHeight.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJump[0])

            if self.isEnabled("heightlimit"):
                Dobot.heightLimit.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpJump[1])

        if self.GetCPParams:
            cp = dTypeX.GetCPParams(self.api)
            if self.isEnabled("cpvelocity"):
                Dobot.cpVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(cp[0])

            if self.isEnabled("cpacceleration"):
                Dobot.cpAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(cp[1])

        if self.GetARCParams:
            arc = dTypeX.GetARCParams(self.api)
            if self.isEnabled("arcxyzvelocity"):
                Dobot.arcXYZVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(arc[0])

            if self.isEnabled("arcrvelocity"):
                Dobot.arcRVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(arc[1])

            if self.isEnabled("arcxyzacceleration"):
                Dobot.arcXYZAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(arc[2])

            if self.isEnabled("arcracceleration"):
                Dobot.arcRAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(arc[3])

        if self.GetAngleSensorStaticError:
            angleStaticErr = dTypeX.GetAngleSensorStaticError(self.api)
            if self.isEnabled("anglestaticerrrear"):
                Dobot.angleStaticErrRear.labels(device_id=self.id, device_type=self.type, station=self.host).set(angleStaticErr[0])

            if self.isEnabled("anglestaticerrfront"):
                Dobot.angleStaticErrFront.labels(device_id=self.id, device_type=self.type, station=self.host).set(angleStaticErr[1])

        if self.GetAngleSensorCoef:
            angleCoef = dTypeX.GetAngleSensorCoef(self.api)
            if self.isEnabled("anglecoefrear"):
                Dobot.angleCoefRear.labels(device_id=self.id, device_type=self.type, station=self.host).set(angleCoef[0])

            if self.isEnabled("anglecoeffront"):
                Dobot.angleCoefFront.labels(device_id=self.id, device_type=self.type, station=self.host).set(angleCoef[1])

        if self.isEnabled("slidingrailstatus"):
            if bool(dTypeX.GetDeviceWithL(self.api)[0]):
                Dobot.slidingRailStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("enabled")
            else:
                Dobot.slidingRailStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("disabled")

        if self.isEnabled("slidingrailpose"):
            Dobot.slidingRailPose.labels(device_id=self.id, device_type=self.type, station=self.host).set(dTypeX.GetPoseL(self.api)[0])

        if self.GetJOGLParams:
            jogRail = dTypeX.GetJOGLParams(self.api)
            if self.isEnabled("slidingrailjogvelocity"):
                Dobot.slidingRailJogVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogRail[0])

            if self.isEnabled("slidingrailjogacceleration"):
                Dobot.slidingRailJogAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(jogRail[1])

        if self.GetPTPLParams:
            ptpRail = dTypeX.GetPTPLParams(self.api)
            if self.isEnabled("slidingrailptpvelocity"):
                Dobot.slidingRailPtpVelocity.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpRail[0])

            if self.isEnabled("slidingrailptpacceleration"):
                Dobot.slidingRailPtpAcceleration.labels(device_id=self.id, device_type=self.type, station=self.host).set(ptpRail[1])

        if self.isEnabled("wifimodulestatus"):
            if bool(dTypeX.GetWIFIConfigMode(self.api)[0]):
                Dobot.wifiModuleStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("enabled")
            else:
                Dobot.wifiModuleStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("disabled")

        if self.isEnabled("wificonnectionstatus"):
            if bool(dTypeX.GetWIFIConnectStatus(self.api)[0]):
                Dobot.wifiConnectionStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("enabled")
            else:
                Dobot.wifiConnectionStatus.labels(device_id=self.id, device_type=self.type, station=self.host).state("disabled")

    def disconnect(self):
        dTypeX.DisconnectDobotX(self.api)

class Jevois(Device):
    options = {"objects":"","objectidentified":True,"objectlocation":True,"objectsize":False}

    objectLocationX = Gauge("object_location_x", "Identified object's x position", ["device_id","device_type","station"])
    objectLocationY = Gauge("object_location_y", "Identified object's y position", ["device_id","device_type","station"])
    objectLocationZ = Gauge("object_location_z", "Identified object's Z position", ["device_id","device_type","station"])
    objectSize = Gauge("object_size","Identified object's size", ["device_id","device_type","station"])

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=0)
            self.__initialize()
        except Exception as e:
            raise Exception(str(e))

    def __initialize(self):
        if self.isEnabled("objectidentified"):
            if self.section["objects"] is not None:
                self.objects = self.section["objects"].split()
                self.objectIdentified = Enum("object_identified_by_"+self.id+"_"+self.host, "Object Identified", states=self.objects)
            else:
                raise Exception("The \"objects\" list is necessary for monitoring identified objects")

    def fetch(self):
        line = self.serial.readline().rstrip().decode()
        tok = line.split()

        # in case of no identified object (empty message) skip fetching
        if len(tok) < 1: return

        serstyle = tok[0][0]
        dimension = tok[0][1]

        # If the serstyle is not Normal (thus it is unsupported by the module)
        if (serstyle != "N"): raise Exception("Unsupported serstyle (" + serstyle + ")")

        if dimension == "1" and len(tok) != 4: raise Exception("Malformed line (expected 4 fields but received " + str(len(tok)) + ")")
        if dimension == "2" and len(tok) != 6: raise Exception("Malformed line (expected 6 fields but received " + str(len(tok)) + ")")
        if dimension == "3" and len(tok) != 8: raise Exception("Malformed line (expected 8 fields but received " + str(len(tok)) + ")")

        if self.isEnabled("objectidentified"):
            if self.objects is not None and tok[1] in self.objects:
                self.objectIdentified.state(tok[1])

        if self.isEnabled("objectlocation"):
            Jevois.objectLocationX.labels(device_id=self.id, device_type=self.type, station=self.host).set(float(tok[2]))

            if int(dimension) > 1:
                Jevois.objectLocationY.labels(device_id=self.id, device_type=self.type, station=self.host).set(float(tok[3]))

            if int(dimension) == 3:
                Jevois.objectLocationZ.labels(device_id=self.id, device_type=self.type, station=self.host).set(float(tok[4]))

        if self.isEnabled("objectsize"):
            if dimension == "1":
                Jevois.objectSize.labels(device_id=self.id, device_type=self.type, station=self.host).set(float(tok[3]))
            elif dimension == "2":
                Jevois.objectSize.labels(device_id=self.id, device_type=self.type, station=self.host).set(abs(float(tok[4])*float(tok[5])))
            elif dimension == "3":
                Jevois.objectSize.labels(device_id=self.id, device_type=self.type, station=self.host).set(abs(float(tok[5])*float(tok[6])*float(tok[7])))

    def disconnect(self):
        self.serial.close()
