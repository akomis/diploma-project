from abc import ABC, abstractmethod
from prometheus_client import Info, Gauge, Enum
import runtime.DobotDllTypeX as dTypeX
import serial

class Device(ABC):
    options = {"timeout":100} # Default device options/attributes

    def __init__(self, config_section, port, host):
        self.section = config_section
        self.type = type(self).__name__
        self.port = port
        self.host = host
        self.id = self.type + ":" + self.port

        self.timeout = self.section.getint("timeout", fallback=Device.options["timeout"])
        if (self.timeout < 100):
            self.timeout = 100

        activeCounter = 0
        for key in type(self).options:
            if isinstance(type(self).options[key], bool) and self.section.getboolean(key, fallback=type(self).options[key]):
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

    deviceInfo = Info("dobot_magician", "General information about monitored Dobot Magician device", ["device","station"])
    wifiInfo = Info("wifi_info", "Information regarding the device\"s wifi connection", ["device","station"])
    deviceTime = Gauge("device_time","Device\"s clock/time", ["device","station"])
    queueIndex = Gauge("queue_index","Current index in command queue", ["device","station"])
    poseX = Gauge("pose_x","Real-time cartesian coordinate of device\"s X axis", ["device","station"])
    poseY = Gauge("pose_y","Real-time cartesian coordinate of device\"s Y axis", ["device","station"])
    poseZ = Gauge("pose_z","Real-time cartesian coordinate of device\"s Z axis", ["device","station"])
    poseR = Gauge("pose_r","Real-time cartesian coordinate of device\"s R axis", ["device","station"])
    angleBase = Gauge("angle_base","Base joint angle", ["device","station"])
    angleRearArm = Gauge("angle_rear_arm","Rear arm joint angle", ["device","station"])
    angleForearm = Gauge("angle_forearm","Forearm joint angle", ["device","station"])
    angleEndEffector = Gauge("angle_end_effector","End effector joint angle", ["device","station"])
    alarmsState = Enum("alarms", "Device alarms", ["device","station"], states=list(dTypeX.alarms.values()))
    homeX = Gauge("home_x","Home position for X axis", ["device","station"])
    homeY = Gauge("home_y","Home position for Y axis", ["device","station"])
    homeZ = Gauge("home_z","Home position for Z axis", ["device","station"])
    homeR = Gauge("home_r","Home position for R axis", ["device","station"])
    endEffectorX = Gauge("end_effector_x","X-axis offset of end effector", ["device","station"])
    endEffectorY = Gauge("end_effector_y","Y-axis offset of end effector", ["device","station"])
    endEffectorZ = Gauge("end_effector_z","Z-axis offset of end effector", ["device","station"])
    laserStatus = Enum("laser_status","Status (enabled/disabled) of laser", ["device","station"], states=["enabled","disabled"])
    suctionCupStatus = Enum("suction_cup_status","Status (enabled/disabled) of suction cup", ["device","station"], states=["enabled","disabled"])
    gripperStatus = Enum("gripper_status","Status (enabled/disabled) of gripper", ["device","station"], states=["enabled","disabled"])
    jogBaseVelocity = Gauge("jog_base_velocity","Velocity (°/s) of base joint in jogging mode", ["device","station"])
    jogRearArmVelocity = Gauge("jog_rear_arm_velocity","Velocity (°/s) of rear arm joint in jogging mode", ["device","station"])
    jogForearmVelocity = Gauge("jog_forearm_velocity","Velocity (°/s) of forearm joint in jogging mode", ["device","station"])
    jogEndEffectorVelocity = Gauge("jog_end_effector_velocity","Velocity (°/s) of end effector joint in jogging mode", ["device","station"])
    jogBaseAcceleration = Gauge("jog_base_acceleration","Acceleration (°/s^2) of base joint in jogging mode", ["device","station"])
    jogRearArmAcceleration = Gauge("jog_rear_arm_acceleration","Acceleration (°/s^2) of rear arm joint in jogging mode", ["device","station"])
    jogForearmAcceleration = Gauge("jog_forearm_acceleration","Acceleration (°/s^2) of forearm joint in jogging mode", ["device","station"])
    jogEndEffectorAcceleration = Gauge("jog_end_effector_acceleration","Acceleration (°/s^2) of end effector joint in jogging mode", ["device","station"])
    jogAxisXVelocity = Gauge("jog_axis_x_velocity","Velocity (mm/s) of device\"s X axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisYVelocity = Gauge("jog_axis_y_velocity","Velocity (mm/s) of device\"s Y axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisZVelocity = Gauge("jog_axis_z_velocity","Velocity (mm/s) of device\"s Z axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisRVelocity = Gauge("jog_axis_r_velocity","Velocity (mm/s) of device\"s R axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisXAcceleration = Gauge("jog_axis_x_acceleration","Acceleration (mm/s^2) of device\"s X axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisYAcceleration = Gauge("jog_axis_y_acceleration","Acceleration (mm/s^2) of device\"s Y axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisZAcceleration = Gauge("jog_axis_z_acceleration","Acceleration (mm/s^2) of device\"s Z axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogAxisRAcceleration = Gauge("jog_axis_r_acceleration","Acceleration (mm/s^2) of device\"s R axis (cartesian coordinate) in jogging mode", ["device","station"])
    jogVelocityRatio = Gauge("jog_velocity_ratio","Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode", ["device","station"])
    jogAccelerationRatio = Gauge("jog_acceleration_ratio","Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode", ["device","station"])
    ptpBaseVelocity = Gauge("ptp_base_velocity","Velocity (°/s) of base joint in point to point mode", ["device","station"])
    ptpRearArmVelocity = Gauge("ptp_rear_arm_velocity","Velocity (°/s) of rear arm joint in point to point mode", ["device","station"])
    ptpForearmVelocity = Gauge("ptp_forearm_velocity","Velocity (°/s) of forearm joint in point to point mode", ["device","station"])
    ptpEndEffectorVelocity = Gauge("ptp_end_effector_velocity","Velocity (°/s) of end effector joint in point to point mode", ["device","station"])
    ptpBaseAcceleration = Gauge("ptp_base_acceleration","Acceleration (°/s^2) of base joint in point to point mode", ["device","station"])
    ptpRearArmAcceleration = Gauge("ptp_rear_arm_acceleration","Acceleration (°/s^2) of rear arm joint in point to point mode", ["device","station"])
    ptpForearmAcceleration = Gauge("ptp_forearm_acceleration","Acceleration (°/s^2) of forearm joint in point to point mode", ["device","station"])
    ptpEndEffectorAcceleration = Gauge("ptp_end_effector_acceleration","Acceleration (°/s^2) of end effector joint in point to point mode", ["device","station"])
    ptpAxisXYZVelocity = Gauge("ptp_axis_xyz_velocity","Velocity (mm/s) of device\"s X, Y, Z axis (cartesian coordinate) in point to point mode", ["device","station"])
    ptpAxisRVelocity = Gauge("ptp_axis_r_velocity","Velocity (mm/s) of device\"s R axis (cartesian coordinate) in point to point mode", ["device","station"])
    ptpAxisXYZAcceleration = Gauge("ptp_axis_x_y_z_acceleration","Acceleration (mm/s^2) of device\"s X, Y, Z axis (cartesian coordinate) in point to point mode", ["device","station"])
    ptpAxisRAcceleration = Gauge("ptp_axis_r_acceleration","Acceleration (mm/s^2) of device\"s R axis (cartesian coordinate) in point to point mode", ["device","station"])
    ptpVelocityRatio = Gauge("ptp_velocity_ratio","Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode", ["device","station"])
    ptpAccelerationRatio = Gauge("ptp_acceleration_ratio","Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode", ["device","station"])
    liftingHeight = Gauge("lifting_height","Lifting height in jump mode", ["device","station"])
    heightLimit = Gauge("height_limit","Max lifting height in jump mode", ["device","station"])
    cpVelocity = Gauge("cp_velocity","Velocity (mm/s) in cp mode", ["device","station"])
    cpAcceleration = Gauge("cp_acceleration","Acceleration (mm/s^2) in cp mode", ["device","station"])
    arcXYZVelocity = Gauge("arc_x_y_z_velocity","Velocity (mm/s) of X, Y, Z axis in arc mode", ["device","station"])
    arcRVelocity = Gauge("arc_r_velocity","Velocity (mm/s) of R axis in arc mode", ["device","station"])
    arcXYZAcceleration = Gauge("arc_x_y_z_acceleration","Acceleration (mm/s^2) of X, Y, Z axis in arc mode", ["device","station"])
    arcRAcceleration = Gauge("arc_r_acceleration","Acceleration (mm/s^2) of R axis in arc mode", ["device","station"])
    angleStaticErrRear = Gauge("angle_static_err_rear","Rear arm angle sensor static error", ["device","station"])
    angleStaticErrFront = Gauge("arc_static_err_front","Forearm angle sensor static error", ["device","station"])
    angleCoefRear = Gauge("angle_coef_rear","Rear arm angle sensor linearization parameter", ["device","station"])
    angleCoefFront = Gauge("angle_coef_front","Forearm angle sensor linearization parameter", ["device","station"])
    slidingRailStatus = Enum("sliding_rail_status","Sliding rail\"s status (enabled/disabled)", ["device","station"], states=["enabled","disabled"])
    slidingRailPose = Gauge("sliding_rail_pose","Sliding rail\"s real-time pose in mm", ["device","station"])
    slidingRailJogVelocity = Gauge("sliding_rail_jog_velocity","Velocity (mm/s) of sliding rail in jogging mode", ["device","station"])
    slidingRailJogAcceleration = Gauge("sliding_rail_jog_acceleration","Acceleration (mm/s^2) of sliding rail in jogging mode", ["device","station"])
    slidingRailPtpVelocity = Gauge("sliding_rail_ptp_velocity","Velocity (mm/s) of sliding rail in point to point mode", ["device","station"])
    slidingRailPtpAcceleration = Gauge("sliding_rail_ptp_acceleration","Acceleration (mm/s^2) of sliding rail in point to point mode", ["device","station"])
    wifiModuleStatus = Enum("wifi_module_status","Wifi module status (enabled/disabled)", ["device","station"], states=["enabled","disabled"])
    wifiConnectionStatus = Enum("wificonnection_status","Wifi connection status (connected/not connected)", ["device","station"], states=["enabled","disabled"])

    def connect(self):
        stateInfo = {1:"Not Found", 2:"Occupied"}

        try:
            self.api, state = dTypeX.ConnectDobotX(self.port)
            if state[0] == dTypeX.DobotConnect.DobotConnect_NoError:
                self.__prominit()
            else:
                raise Exception(stateInfo[state[0]])
        except Exception as e:
            raise Exception(str(e))

    def __prominit(self):
        enabledDeviceInfo = {}
        if self.section.getboolean("devicesn", fallback=Dobot.options["devicesn"]):
            enabledDeviceInfo["serial_number"] = dTypeX.GetDeviceSN(self.api)[0]
        if self.section.getboolean("devicename", fallback=Dobot.options["devicename"]):
            enabledDeviceInfo["device_name"] = dTypeX.GetDeviceName(self.api)[0]
        if self.section.getboolean("deviceversion", fallback=Dobot.options["deviceversion"]):
            enabledDeviceInfo["version"] = ".".join(list(map(str, dTypeX.GetDeviceVersion(self.api))))
        if len(enabledDeviceInfo) > 0:
            Dobot.deviceInfo.labels(device=self.id, station=self.host).info(enabledDeviceInfo)

        enabledWifiInfo = {}
        if self.section.getboolean("wifissid", fallback=Dobot.options["wifissid"]):
            enabledWifiInfo["ssid"] = dTypeX.GetWIFISSID(self.api)[0]
        if self.section.getboolean("wifipassword", fallback=Dobot.options["wifipassword"]):
            enabledWifiInfo["password"] = dTypeX.GetWIFIPassword(self.api)[0]
        if self.section.getboolean("wifiipaddress", fallback=Dobot.options["wifiipaddress"]):
            enabledWifiInfo["ip_address"] = ".".join(list(map(str, dTypeX.GetWIFIIPAddress(self.api)[1:])))
        if self.section.getboolean("wifinetmask", fallback=Dobot.options["wifinetmask"]):
            enabledWifiInfo["netmask"] = ".".join(list(map(str, dTypeX.GetWIFINetmask(self.api))))
        if self.section.getboolean("wifigateway", fallback=Dobot.options["wifigateway"]):
            enabledWifiInfo["gateway"] = ".".join(list(map(str, dTypeX.GetWIFIGateway(self.api))))
        if self.section.getboolean("wifidns", fallback=Dobot.options["wifidns"]):
            enabledWifiInfo["dns"] = ".".join(list(map(str, dTypeX.GetWIFIDNS(self.api))))
        if len(enabledWifiInfo) > 0:
            Dobot.wifiInfo.labels(device=self.id, station=self.host).info(enabledWifiInfo)

    def fetch(self):
        if self.section.getboolean("devicetime", fallback=Dobot.options["devicetime"]):
            Dobot.deviceTime.labels(device=self.id, station=self.host).set(dTypeX.GetDeviceTime(self.api)[0])

        if self.section.getboolean("queueindex", fallback=Dobot.options["queueindex"]):
            Dobot.queueIndex.labels(device=self.id, station=self.host).set(dTypeX.GetQueuedCmdCurrentIndex(self.api)[0])

        pose = dTypeX.GetPose(self.api)
        if self.section.getboolean("posex", fallback=Dobot.options["posex"]):
            Dobot.poseX.labels(device=self.id, station=self.host).set(pose[0])

        if self.section.getboolean("posey", fallback=Dobot.options["posey"]):
            Dobot.poseY.labels(device=self.id, station=self.host).set(pose[1])

        if self.section.getboolean("posez", fallback=Dobot.options["posez"]):
            Dobot.poseZ.labels(device=self.id, station=self.host).set(pose[2])

        if self.section.getboolean("poser", fallback=Dobot.options["poser"]):
            Dobot.poseR.labels(device=self.id, station=self.host).set(pose[3])

        if self.section.getboolean("anglebase", fallback=Dobot.options["anglebase"]):
            Dobot.angleBase.labels(device=self.id, station=self.host).set(pose[4])

        if self.section.getboolean("anglereararm", fallback=Dobot.options["anglereararm"]):
            Dobot.angleRearArm.labels(device=self.id, station=self.host).set(pose[5])

        if self.section.getboolean("angleforearm", fallback=Dobot.options["angleforearm"]):
            Dobot.angleForearm.labels(device=self.id, station=self.host).set(pose[6])

        if self.section.getboolean("angleendeffector", fallback=Dobot.options["angleendeffector"]):
            Dobot.angleEndEffector.labels(device=self.id, station=self.host).set(pose[7])

        if self.section.getboolean("alarmsstate", fallback=Dobot.options["alarmsstate"]):
            for a in dTypeX.GetAlarmsStateX(self.api):
                Dobot.alarmsState.labels(device=self.id, station=self.host).state(a)

        home = dTypeX.GetHOMEParams(self.api)
        if self.section.getboolean("homex", fallback=Dobot.options["homex"]):
            Dobot.homeX.labels(device=self.id, station=self.host).set(home[0])

        if self.section.getboolean("homey", fallback=Dobot.options["homey"]):
            Dobot.homeY.labels(device=self.id, station=self.host).set(home[1])

        if self.section.getboolean("homez", fallback=Dobot.options["homez"]):
            Dobot.homeZ.labels(device=self.id, station=self.host).set(home[2])

        if self.section.getboolean("homer", fallback=Dobot.options["homer"]):
            Dobot.homeR.labels(device=self.id, station=self.host).set(home[3])

        endEffector = dTypeX.GetEndEffectorParams(self.api)
        if self.section.getboolean("endeffectorx", fallback=Dobot.options["endeffectorx"]):
            Dobot.endEffectorX.labels(device=self.id, station=self.host).set(endEffector[0])

        if self.section.getboolean("endeffectory", fallback=Dobot.options["endeffectory"]):
            Dobot.endEffectorY.labels(device=self.id, station=self.host).set(endEffector[1])

        if self.section.getboolean("endeffectorz", fallback=Dobot.options["endeffectorz"]):
            Dobot.endEffectorZ.labels(device=self.id, station=self.host).set(endEffector[2])

        if self.section.getboolean("laserstatus", fallback=Dobot.options["laserstatus"]):
            if bool(dTypeX.GetEndEffectorLaser(self.api)[0]):
                Dobot.laserStatus.labels(device=self.id, station=self.host).state("enabled")
            else:
                Dobot.laserStatus.labels(device=self.id, station=self.host).state("disabled")

        if self.section.getboolean("suctioncupstatus", fallback=Dobot.options["suctioncupstatus"]):
            if bool(dTypeX.GetEndEffectorSuctionCup(self.api)[0]):
                Dobot.suctionCupStatus.labels(device=self.id, station=self.host).state("enabled")
            else:
                Dobot.suctionCupStatus.labels(device=self.id, station=self.host).state("disabled")

        if self.section.getboolean("gripperstatus", fallback=Dobot.options["gripperstatus"]):
            if bool(dTypeX.GetEndEffectorGripper(self.api)[0]):
                Dobot.gripperStatus.labels(device=self.id, station=self.host).state("enabled")
            else:
                Dobot.gripperStatus.labels(device=self.id, station=self.host).state("disabled")

        jogJoints = dTypeX.GetJOGJointParams(self.api)
        if self.section.getboolean("jogbasevelocity", fallback=Dobot.options["jogbasevelocity"]):
            Dobot.jogBaseVelocity.labels(device=self.id, station=self.host).set(jogJoints[0])

        if self.section.getboolean("jogreararmvelocity", fallback=Dobot.options["jogreararmvelocity"]):
            Dobot.jogRearArmVelocity.labels(device=self.id, station=self.host).set(jogJoints[1])

        if self.section.getboolean("jogforearmvelocity", fallback=Dobot.options["jogforearmvelocity"]):
            Dobot.jogForearmVelocity.labels(device=self.id, station=self.host).set(jogJoints[2])

        if self.section.getboolean("jogendeffectorvelocity", fallback=Dobot.options["jogendeffectorvelocity"]):
            Dobot.jogEndEffectorVelocity.labels(device=self.id, station=self.host).set(jogJoints[3])

        if self.section.getboolean("jogbaseacceleration", fallback=Dobot.options["jogbaseacceleration"]):
            Dobot.jogBaseAcceleration.labels(device=self.id, station=self.host).set(jogJoints[4])

        if self.section.getboolean("jogreararmacceleration", fallback=Dobot.options["jogreararmacceleration"]):
            Dobot.jogRearArmAcceleration.labels(device=self.id, station=self.host).set(jogJoints[5])

        if self.section.getboolean("jogforearmacceleration", fallback=Dobot.options["jogforearmacceleration"]):
            Dobot.jogForearmAcceleration.labels(device=self.id, station=self.host).set(jogJoints[6])

        if self.section.getboolean("jogendeffectoracceleration", fallback=Dobot.options["jogendeffectoracceleration"]):
            Dobot.jogEndEffectorAcceleration.labels(device=self.id, station=self.host).set(jogJoints[7])

        jogCoords = dTypeX.GetJOGCoordinateParams(self.api)
        if self.section.getboolean("jogaxisxvelocity", fallback=Dobot.options["jogaxisxvelocity"]):
            Dobot.jogAxisXVelocity.labels(device=self.id, station=self.host).set(jogCoords[0])

        if self.section.getboolean("jogaxisyvelocity", fallback=Dobot.options["jogaxisyvelocity"]):
            Dobot.jogAxisYVelocity.labels(device=self.id, station=self.host).set(jogCoords[1])

        if self.section.getboolean("jogaxiszvelocity", fallback=Dobot.options["jogaxiszvelocity"]):
            Dobot.jogAxisZVelocity.labels(device=self.id, station=self.host).set(jogCoords[2])

        if self.section.getboolean("jogaxisrvelocity", fallback=Dobot.options["jogaxisrvelocity"]):
            Dobot.jogAxisRVelocity.labels(device=self.id, station=self.host).set(jogCoords[3])

        if self.section.getboolean("jogaxisxacceleration", fallback=Dobot.options["jogaxisxacceleration"]):
            Dobot.jogAxisXAcceleration.labels(device=self.id, station=self.host).set(jogCoords[4])

        if self.section.getboolean("jogaxisyacceleration", fallback=Dobot.options["jogaxisyacceleration"]):
            Dobot.jogAxisYAcceleration.labels(device=self.id, station=self.host).set(jogCoords[5])

        if self.section.getboolean("jogaxiszacceleration", fallback=Dobot.options["jogaxiszacceleration"]):
            Dobot.jogAxisZAcceleration.labels(device=self.id, station=self.host).set(jogCoords[6])

        if self.section.getboolean("jogaxisracceleration", fallback=Dobot.options["jogaxisracceleration"]):
            Dobot.jogAxisRAcceleration.labels(device=self.id, station=self.host).set(jogCoords[7])

        jogCommon = dTypeX.GetJOGCommonParams(self.api)
        if self.section.getboolean("jogvelocityratio", fallback=Dobot.options["jogvelocityratio"]):
            Dobot.jogVelocityRatio.labels(device=self.id, station=self.host).set(jogCommon[0])

        if self.section.getboolean("jogaccelerationratio", fallback=Dobot.options["jogaccelerationratio"]):
            Dobot.jogAccelerationRatio.labels(device=self.id, station=self.host).set(jogCommon[1])

        ptpJoints = dTypeX.GetPTPJointParams(self.api)
        if self.section.getboolean("ptpbasevelocity", fallback=Dobot.options["ptpbasevelocity"]):
            Dobot.ptpBaseVelocity.labels(device=self.id, station=self.host).set(ptpJoints[0])

        if self.section.getboolean("ptpreararmvelocity", fallback=Dobot.options["ptpreararmvelocity"]):
            Dobot.ptpRearArmVelocity.labels(device=self.id, station=self.host).set(ptpJoints[1])

        if self.section.getboolean("ptpforearmvelocity", fallback=Dobot.options["ptpforearmvelocity"]):
            Dobot.ptpForearmVelocity.labels(device=self.id, station=self.host).set(ptpJoints[2])

        if self.section.getboolean("ptpendeffectorvelocity", fallback=Dobot.options["ptpendeffectorvelocity"]):
            Dobot.ptpEndEffectorVelocity.labels(device=self.id, station=self.host).set(ptpJoints[3])

        if self.section.getboolean("ptpbaseacceleration", fallback=Dobot.options["ptpbaseacceleration"]):
            Dobot.ptpBaseAcceleration.labels(device=self.id, station=self.host).set(ptpJoints[4])

        if self.section.getboolean("ptpreararmacceleration", fallback=Dobot.options["ptpreararmacceleration"]):
            Dobot.ptpRearArmAcceleration.labels(device=self.id, station=self.host).set(ptpJoints[5])

        if self.section.getboolean("ptpforearmacceleration", fallback=Dobot.options["ptpforearmacceleration"]):
            Dobot.ptpForearmAcceleration.labels(device=self.id, station=self.host).set(ptpJoints[6])

        if self.section.getboolean("ptpendeffectoracceleration", fallback=Dobot.options["ptpendeffectoracceleration"]):
            Dobot.ptpEndEffectorAcceleration.labels(device=self.id, station=self.host).set(ptpJoints[7])

        ptpCoords = dTypeX.GetPTPCoordinateParams(self.api)
        if self.section.getboolean("ptpaxisxyzvelocity", fallback=Dobot.options["ptpaxisxyzvelocity"]):
            Dobot.ptpAxisXYZVelocity.labels(device=self.id, station=self.host).set(ptpCoords[0])

        if self.section.getboolean("ptpaxisrvelocity", fallback=Dobot.options["ptpaxisrvelocity"]):
            Dobot.ptpAxisRVelocity.labels(device=self.id, station=self.host).set(ptpCoords[1])

        if self.section.getboolean("ptpaxisxyzacceleration", fallback=Dobot.options["ptpaxisxyzacceleration"]):
            Dobot.ptpAxisXYZAcceleration.labels(device=self.id, station=self.host).set(ptpCoords[2])

        if self.section.getboolean("ptpaxisracceleration", fallback=Dobot.options["ptpaxisracceleration"]):
            Dobot.ptpAxisRAcceleration.labels(device=self.id, station=self.host).set(ptpCoords[3])

        ptpCommon = dTypeX.GetPTPCommonParams(self.api)
        if self.section.getboolean("ptpvelocityratio", fallback=Dobot.options["ptpvelocityratio"]):
            Dobot.ptpVelocityRatio.labels(device=self.id, station=self.host).set(ptpCommon[0])

        if self.section.getboolean("ptpaccelerationratio", fallback=Dobot.options["ptpaccelerationratio"]):
            Dobot.ptpAccelerationRatio.labels(device=self.id, station=self.host).set(ptpCommon[1])

        ptpJump = dTypeX.GetPTPJumpParams(self.api)
        if self.section.getboolean("liftingheight", fallback=Dobot.options["liftingheight"]):
            Dobot.liftingHeight.labels(device=self.id, station=self.host).set(ptpJump[0])

        if self.section.getboolean("heightlimit", fallback=Dobot.options["heightlimit"]):
            Dobot.heightLimit.labels(device=self.id, station=self.host).set(ptpJump[1])

        cp = dTypeX.GetCPParams(self.api)
        if self.section.getboolean("cpvelocity", fallback=Dobot.options["cpvelocity"]):
            Dobot.cpVelocity.labels(device=self.id, station=self.host).set(cp[0])

        if self.section.getboolean("cpacceleration", fallback=Dobot.options["cpacceleration"]):
            Dobot.cpAcceleration.labels(device=self.id, station=self.host).set(cp[1])

        arc = dTypeX.GetARCParams(self.api)
        if self.section.getboolean("arcxyzvelocity", fallback=Dobot.options["arcxyzvelocity"]):
            Dobot.arcXYZVelocity.labels(device=self.id, station=self.host).set(arc[0])

        if self.section.getboolean("arcrvelocity", fallback=Dobot.options["arcrvelocity"]):
            Dobot.arcRVelocity.labels(device=self.id, station=self.host).set(arc[1])

        if self.section.getboolean("arcxyzacceleration", fallback=Dobot.options["arcxyzacceleration"]):
            Dobot.arcXYZAcceleration.labels(device=self.id, station=self.host).set(arc[2])

        if self.section.getboolean("arcracceleration", fallback=Dobot.options["arcracceleration"]):
            Dobot.arcRAcceleration.labels(device=self.id, station=self.host).set(arc[3])

        angleStaticErr = dTypeX.GetAngleSensorStaticError(self.api)
        if self.section.getboolean("anglestaticerrrear", fallback=Dobot.options["anglestaticerrrear"]):
            Dobot.angleStaticErrRear.labels(device=self.id, station=self.host).set(angleStaticErr[0])

        if self.section.getboolean("anglestaticerrfront", fallback=Dobot.options["anglestaticerrfront"]):
            Dobot.angleStaticErrFront.labels(device=self.id, station=self.host).set(angleStaticErr[1])

        angleCoef = dTypeX.GetAngleSensorCoef(self.api)
        if self.section.getboolean("anglecoefrear", fallback=Dobot.options["anglecoefrear"]):
            Dobot.angleCoefRear.labels(device=self.id, station=self.host).set(angleCoef[0])

        if self.section.getboolean("anglecoeffront", fallback=Dobot.options["anglecoeffront"]):
            Dobot.angleCoefFront.labels(device=self.id, station=self.host).set(angleCoef[1])

        if self.section.getboolean("slidingrailstatus", fallback=Dobot.options["slidingrailstatus"]):
            if bool(dTypeX.GetDeviceWithL(self.api)[0]):
                Dobot.slidingRailStatus.labels(device=self.id, station=self.host).state("enabled")
            else:
                Dobot.slidingRailStatus.labels(device=self.id, station=self.host).state("disabled")

        if self.section.getboolean("slidingrailpose", fallback=Dobot.options["slidingrailpose"]):
            Dobot.slidingRailPose.labels(device=self.id, station=self.host).set(dTypeX.GetPoseL(self.api)[0])

        jogRail = dTypeX.GetJOGLParams(self.api)
        if self.section.getboolean("slidingrailjogvelocity", fallback=Dobot.options["slidingrailjogvelocity"]):
            Dobot.slidingRailJogVelocity.labels(device=self.id, station=self.host).set(jogRail[0])

        if self.section.getboolean("slidingrailjogacceleration", fallback=Dobot.options["slidingrailjogacceleration"]):
            Dobot.slidingRailJogAcceleration.labels(device=self.id, station=self.host).set(jogRail[1])

        ptpRail = dTypeX.GetPTPLParams(self.api)
        if self.section.getboolean("slidingrailptpvelocity", fallback=Dobot.options["slidingrailptpvelocity"]):
            Dobot.slidingRailPtpVelocity.labels(device=self.id, station=self.host).set(ptpRail[0])

        if self.section.getboolean("slidingrailptpacceleration", fallback=Dobot.options["slidingrailptpacceleration"]):
            Dobot.slidingRailPtpAcceleration.labels(device=self.id, station=self.host).set(ptpRail[1])


        if self.section.getboolean("wifimodulestatus", fallback=Dobot.options["wifimodulestatus"]):
            if bool(dTypeX.GetWIFIConfigMode(self.api)[0]):
                Dobot.wifiModuleStatus.labels(device=self.id, station=self.host).state("enabled")
            else:
                Dobot.wifiModuleStatus.labels(device=self.id, station=self.host).state("disabled")

        if self.section.getboolean("wificonnectionstatus", fallback=Dobot.options["wificonnectionstatus"]):
            if bool(dTypeX.GetWIFIConnectStatus(self.api)[0]):
                Dobot.wifiConnectionStatus.labels(device=self.id, station=self.host).state("enabled")
            else:
                Dobot.wifiConnectionStatus.labels(device=self.id, station=self.host).state("disabled")

    def disconnect(self):
        dTypeX.DisconnectDobotX(self.api)

class Jevois(Device):
    options = {"objects":"","objectidentified":True,"objectlocation":True,"objectsize":False}

    objectLocationX = Gauge("object_location_x", "Identified object\"s x position", ["device","station"])
    objectLocationY = Gauge("object_location_y", "Identified object\"s y position", ["device","station"])
    objectLocationZ = Gauge("object_location_z", "Identified object\"s Z position", ["device","station"])
    objectSize = Gauge("object_size","Identified object\"s size", ["device","station"])

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=0)
            self.__prominit()
        except Exception as e:
            raise Exception(str(e))

    def __prominit(self):
        if self.section.getboolean("objectidentified", fallback=Jevois.options["objectidentified"]):
            if self.section["objects"] is not None:
                self.objects = self.section["objects"].split()
                self.objectIdentified = Enum("object_identified_by_"+self.port, "Object Identified", states=self.objects)
            else:
                raise Exception("The \"objects\" list is necessary for monitoring identified objects")

    def fetch(self):
        line = self.serial.readline().rstrip().decode()
        tok = line.split()

        # in case of no identified object (empty message)
        if len(tok) < 1: return

        serstyle = tok[0][0]
        dimension = tok[0][1]

        # If the serstyle is not Normal (thus it is unsupported by the module)
        if (serstyle != "N"): raise Exception("Unsupported serstyle (" + serstyle + ")")

        if dimension == "1" and len(tok) != 4: raise Exception("Malformed line (expected 4 fields but received " + str(len(tok)) + ")")
        if dimension == "2" and len(tok) != 6: raise Exception("Malformed line (expected 6 fields but received " + str(len(tok)) + ")")
        if dimension == "3" and len(tok) != 8: raise Exception("Malformed line (expected 8 fields but received " + str(len(tok)) + ")")

        if self.section.getboolean("objectidentified", fallback=Jevois.options["objectidentified"]):
            if self.objects is not None and tok[1] in self.objects:
                self.objectIdentified.state(tok[1])

        if self.section.getboolean("objectlocation", fallback=Jevois.options["objectlocation"]):
            Jevois.objectLocationX.labels(device=self.id, station=self.host).set(float(tok[2]))

            if int(dimension) > 1:
                Jevois.objectLocationY.labels(device=self.id, station=self.host).set(float(tok[3]))

            if int(dimension) == 3:
                Jevois.objectLocationZ.labels(device=self.id, station=self.host).set(float(tok[4]))

        if self.section.getboolean("objectsize", fallback=Jevois.options["objectsize"]):
            if dimension == "1":
                Jevois.objectSize.labels(device=self.id, station=self.host).set(float(tok[3]))
            elif dimension == "2":
                Jevois.objectSize.labels(device=self.id, station=self.host).set(abs(float(tok[4])*float(tok[5])))
            elif dimension == "3":
                Jevois.objectSize.labels(device=self.id, station=self.host).set(abs(float(tok[5])*float(tok[6])*float(tok[7])))

    def disconnect(self):
        self.serial.close()
