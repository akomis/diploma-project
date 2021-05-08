from prometheus_client import Info, Gauge, Enum
import runtime.DobotDllTypeX as dTypeX
import serial

class Device():
    options = {"timeout":100} # Default device options/attributes

    def __init__(self, config, port):
        self.port = port
        self.id = type(self).__name__ + ":" + self.port
        self.section = config[self.id]
        self.timeout = self.section.getint("timeout", fallback=Device.options["timeout"])
        if (self.timeout < 100):
            self.timeout = 100

    #@abstractmethod
    def _connect(self):
        return None

    #@abstractmethod
    def _fetch(self):
        return None

    #@abstractmethod
    def _disconnect(self):
        return None


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

    deviceInfo = Info("dobot_magician", "General information about monitored Dobot Magician device", ["device"])
    wifiInfo = Info("wifi_info", "Information regarding the device\"s wifi connection", ["device"])
    deviceTime = Gauge("device_time","Device\"s clock/time", ["device"])
    queueIndex = Gauge("queue_index","Current index in command queue", ["device"])
    poseX = Gauge("pose_x","Real-time cartesian coordinate of device\"s X axis", ["device"])
    poseY = Gauge("pose_y","Real-time cartesian coordinate of device\"s Y axis", ["device"])
    poseZ = Gauge("pose_z","Real-time cartesian coordinate of device\"s Z axis", ["device"])
    poseR = Gauge("pose_r","Real-time cartesian coordinate of device\"s R axis", ["device"])
    angleBase = Gauge("angle_base","Base joint angle", ["device"])
    angleRearArm = Gauge("angle_rear_arm","Rear arm joint angle", ["device"])
    angleForearm = Gauge("angle_forearm","Forearm joint angle", ["device"])
    angleEndEffector = Gauge("angle_end_effector","End effector joint angle", ["device"])
    alarmsState = Enum("alarms", "Device alarms", ["device"], states=list(dTypeX.alarms.values()))
    homeX = Gauge("home_x","Home position for X axis", ["device"])
    homeY = Gauge("home_y","Home position for Y axis", ["device"])
    homeZ = Gauge("home_z","Home position for Z axis", ["device"])
    homeR = Gauge("home_r","Home position for R axis", ["device"])
    endEffectorX = Gauge("end_effector_x","X-axis offset of end effector", ["device"])
    endEffectorY = Gauge("end_effector_y","Y-axis offset of end effector", ["device"])
    endEffectorZ = Gauge("end_effector_z","Z-axis offset of end effector", ["device"])
    laserStatus = Enum("laser_status","Status (enabled/disabled) of laser", ["device"], states=["enabled","disabled"])
    suctionCupStatus = Enum("suction_cup_status","Status (enabled/disabled) of suction cup", ["device"], states=["enabled","disabled"])
    gripperStatus = Enum("gripper_status","Status (enabled/disabled) of gripper", ["device"], states=["enabled","disabled"])
    jogBaseVelocity = Gauge("jog_base_velocity","Velocity (°/s) of base joint in jogging mode", ["device"])
    jogRearArmVelocity = Gauge("jog_rear_arm_velocity","Velocity (°/s) of rear arm joint in jogging mode", ["device"])
    jogForearmVelocity = Gauge("jog_forearm_velocity","Velocity (°/s) of forearm joint in jogging mode", ["device"])
    jogEndEffectorVelocity = Gauge("jog_end_effector_velocity","Velocity (°/s) of end effector joint in jogging mode", ["device"])
    jogBaseAcceleration = Gauge("jog_base_acceleration","Acceleration (°/s^2) of base joint in jogging mode", ["device"])
    jogRearArmAcceleration = Gauge("jog_rear_arm_acceleration","Acceleration (°/s^2) of rear arm joint in jogging mode", ["device"])
    jogForearmAcceleration = Gauge("jog_forearm_acceleration","Acceleration (°/s^2) of forearm joint in jogging mode", ["device"])
    jogEndEffectorAcceleration = Gauge("jog_end_effector_acceleration","Acceleration (°/s^2) of end effector joint in jogging mode", ["device"])
    jogAxisXVelocity = Gauge("jog_axis_x_velocity","Velocity (mm/s) of device\"s X axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisYVelocity = Gauge("jog_axis_y_velocity","Velocity (mm/s) of device\"s Y axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisZVelocity = Gauge("jog_axis_z_velocity","Velocity (mm/s) of device\"s Z axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisRVelocity = Gauge("jog_axis_r_velocity","Velocity (mm/s) of device\"s R axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisXAcceleration = Gauge("jog_axis_x_acceleration","Acceleration (mm/s^2) of device\"s X axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisYAcceleration = Gauge("jog_axis_y_acceleration","Acceleration (mm/s^2) of device\"s Y axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisZAcceleration = Gauge("jog_axis_z_acceleration","Acceleration (mm/s^2) of device\"s Z axis (cartesian coordinate) in jogging mode", ["device"])
    jogAxisRAcceleration = Gauge("jog_axis_r_acceleration","Acceleration (mm/s^2) of device\"s R axis (cartesian coordinate) in jogging mode", ["device"])
    jogVelocityRatio = Gauge("jog_velocity_ratio","Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode", ["device"])
    jogAccelerationRatio = Gauge("jog_acceleration_ratio","Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode", ["device"])
    ptpBaseVelocity = Gauge("ptp_base_velocity","Velocity (°/s) of base joint in point to point mode", ["device"])
    ptpRearArmVelocity = Gauge("ptp_rear_arm_velocity","Velocity (°/s) of rear arm joint in point to point mode", ["device"])
    ptpForearmVelocity = Gauge("ptp_forearm_velocity","Velocity (°/s) of forearm joint in point to point mode", ["device"])
    ptpEndEffectorVelocity = Gauge("ptp_end_effector_velocity","Velocity (°/s) of end effector joint in point to point mode", ["device"])
    ptpBaseAcceleration = Gauge("ptp_base_acceleration","Acceleration (°/s^2) of base joint in point to point mode", ["device"])
    ptpRearArmAcceleration = Gauge("ptp_rear_arm_acceleration","Acceleration (°/s^2) of rear arm joint in point to point mode", ["device"])
    ptpForearmAcceleration = Gauge("ptp_forearm_acceleration","Acceleration (°/s^2) of forearm joint in point to point mode", ["device"])
    ptpEndEffectorAcceleration = Gauge("ptp_end_effector_acceleration","Acceleration (°/s^2) of end effector joint in point to point mode", ["device"])
    ptpXYZVelocity = Gauge("ptp_xyz_velocity","Velocity (mm/s) of device\"s X, Y, Z axis (cartesian coordinate) in point to point mode", ["device"])
    ptpRVelocity = Gauge("ptp_r_velocity","Velocity (mm/s) of device\"s R axis (cartesian coordinate) in point to point mode", ["device"])
    ptpXYZAcceleration = Gauge("ptp_x_y_z_acceleration","Acceleration (mm/s^2) of device\"s X, Y, Z axis (cartesian coordinate) in point to point mode", ["device"])
    ptpRAcceleration = Gauge("ptp_r_acceleration","Acceleration (mm/s^2) of device\"s R axis (cartesian coordinate) in point to point mode", ["device"])
    ptpVelocityRatio = Gauge("ptp_velocity_ratio","Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode", ["device"])
    ptpAccelerationRatio = Gauge("ptp_acceleration_ratio","Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode", ["device"])
    liftingHeight = Gauge("lifting_height","Lifting height in jump mode", ["device"])
    heightLimit = Gauge("height_limit","Max lifting height in jump mode", ["device"])
    cpVelocity = Gauge("cp_velocity","Velocity (mm/s) in cp mode", ["device"])
    cpAcceleration = Gauge("cp_acceleration","Acceleration (mm/s^2) in cp mode", ["device"])
    arcXYZVelocity = Gauge("arc_x_y_z_velocity","Velocity (mm/s) of X, Y, Z axis in arc mode", ["device"])
    arcRVelocity = Gauge("arc_r_velocity","Velocity (mm/s) of R axis in arc mode", ["device"])
    arcXYZAcceleration = Gauge("arc_x_y_z_acceleration","Acceleration (mm/s^2) of X, Y, Z axis in arc mode", ["device"])
    arcRAcceleration = Gauge("arc_r_acceleration","Acceleration (mm/s^2) of R axis in arc mode", ["device"])
    angleStaticErrRear = Gauge("angle_static_err_rear","Rear arm angle sensor static error", ["device"])
    angleStaticErrFront = Gauge("arc_static_err_front","Forearm angle sensor static error", ["device"])
    angleCoefRear = Gauge("angle_coef_rear","Rear arm angle sensor linearization parameter", ["device"])
    angleCoefFront = Gauge("angle_coef_front","Forearm angle sensor linearization parameter", ["device"])
    slidingRailStatus = Enum("sliding_rail_status","Sliding rail\"s status (enabled/disabled)", ["device"], states=["enabled","disabled"])
    slidingRailPose = Gauge("sliding_rail_pose","Sliding rail\"s real-time pose in mm", ["device"])
    slidingRailJogVelocity = Gauge("sliding_rail_jog_velocity","Velocity (mm/s) of sliding rail in jogging mode", ["device"])
    slidingRailJogAcceleration = Gauge("sliding_rail_jog_acceleration","Acceleration (mm/s^2) of sliding rail in jogging mode", ["device"])
    slidingRailPtpVelocity = Gauge("sliding_rail_ptp_velocity","Velocity (mm/s) of sliding rail in point to point mode", ["device"])
    slidingRailPtpAcceleration = Gauge("sliding_rail_ptp_acceleration","Acceleration (mm/s^2) of sliding rail in point to point mode", ["device"])
    wifiModuleStatus = Enum("wifi_module_status","Wifi module status (enabled/disabled)", ["device"], states=["enabled","disabled"])
    wifiConnectionStatus = Enum("wifi_connection_status","Wifi connection status (connected/not connected)", ["device"], states=["enabled","disabled"])

    def _connect(self):
        self.api, state = dTypeX.ConnectDobotX(self.port)

        if state[0] == dTypeX.DobotConnect.DobotConnect_NoError:
            self.__prominit()
            return True
        else:
            return False

    def __prominit(self):
        enabledDeviceInfo = {}
        if self.section.getboolean("devicesn", fallback=Dobot.options["devicesn"]):
            enabledDeviceInfo["serial_number"] = dTypeX.GetDeviceSN(self.api)[0]
        if self.section.getboolean("devicename", fallback=Dobot.options["devicename"]):
            enabledDeviceInfo["device_name"] = dTypeX.GetDeviceName(self.api)[0]
        if self.section.getboolean("deviceversion", fallback=Dobot.options["deviceversion"]):
            enabledDeviceInfo["version"] = ".".join(list(map(str, dTypeX.GetDeviceVersion(self.api))))
        if len(enabledDeviceInfo) > 0:
            Dobot.deviceInfo.labels(self.id).info(enabledDeviceInfo)

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
            Dobot.wifiInfo.labels(self.id).info(enabledWifiInfo)

    def _fetch(self):
        if self.section.getboolean("devicetime", fallback=Dobot.options["devicetime"]):
            Dobot.deviceTime.labels(self.id).set(dTypeX.GetDeviceTime(self.api)[0])

        if self.section.getboolean("queueindex", fallback=Dobot.options["queueindex"]):
            Dobot.queueIndex.labels(self.id).set(dTypeX.GetQueuedCmdCurrentIndex(self.api)[0])

        pose = dTypeX.GetPose(self.api)
        if self.section.getboolean("posex", fallback=Dobot.options["posex"]):
            Dobot.poseX.labels(self.id).set(pose[0])

        if self.section.getboolean("posey", fallback=Dobot.options["posey"]):
            Dobot.poseY.labels(self.id).set(pose[1])

        if self.section.getboolean("posez", fallback=Dobot.options["posez"]):
            Dobot.poseZ.labels(self.id).set(pose[2])

        if self.section.getboolean("poser", fallback=Dobot.options["poser"]):
            Dobot.poseR.labels(self.id).set(pose[3])

        if self.section.getboolean("anglebase", fallback=Dobot.options["anglebase"]):
            Dobot.angleBase.labels(self.id).set(pose[4])

        if self.section.getboolean("anglereararm", fallback=Dobot.options["anglereararm"]):
            Dobot.angleRearArm.labels(self.id).set(pose[5])

        if self.section.getboolean("angleforearm", fallback=Dobot.options["angleforearm"]):
            Dobot.angleForearm.labels(self.id).set(pose[6])

        if self.section.getboolean("angleendeffector", fallback=Dobot.options["angleendeffector"]):
            Dobot.angleEndEffector.labels(self.id).set(pose[7])

        if self.section.getboolean("alarmsstate", fallback=Dobot.options["alarmsstate"]):
            for a in dTypeX.GetAlarmsStateX(self.api):
                Dobot.alarmsState.labels(self.id).state(a)

        home = dTypeX.GetHOMEParams(self.api)
        if self.section.getboolean("homex", fallback=Dobot.options["homex"]):
            Dobot.homeX.labels(self.id).set(home[0])

        if self.section.getboolean("homey", fallback=Dobot.options["homey"]):
            Dobot.homeY.labels(self.id).set(home[1])

        if self.section.getboolean("homez", fallback=Dobot.options["homez"]):
            Dobot.homeZ.labels(self.id).set(home[2])

        if self.section.getboolean("homer", fallback=Dobot.options["homer"]):
            Dobot.homeR.labels(self.id).set(home[3])

        endEffector = dTypeX.GetEndEffectorParams(self.api)
        if self.section.getboolean("endeffectorx", fallback=Dobot.options["endeffectorx"]):
            Dobot.endEffectorX.labels(self.id).set(endEffector[0])

        if self.section.getboolean("endeffectory", fallback=Dobot.options["endeffectory"]):
            Dobot.endEffectorY.labels(self.id).set(endEffector[1])

        if self.section.getboolean("endeffectorz", fallback=Dobot.options["endeffectorz"]):
            Dobot.endEffectorZ.labels(self.id).set(endEffector[2])

        if self.section.getboolean("laserstatus", fallback=Dobot.options["laserstatus"]):
            if bool(dTypeX.GetEndEffectorLaser(self.api)[0]):
                Dobot.laserStatus.labels(self.id).state("enabled")
            else:
                Dobot.laserStatus.labels(self.id).state("disabled")

        if self.section.getboolean("suctioncupstatus", fallback=Dobot.options["suctioncupstatus"]):
            if bool(dTypeX.GetEndEffectorSuctionCup(self.api)[0]):
                Dobot.suctionCupStatus.labels(self.id).state("enabled")
            else:
                Dobot.suctionCupStatus.labels(self.id).state("disabled")

        if self.section.getboolean("gripperstatus", fallback=Dobot.options["gripperstatus"]):
            if bool(dTypeX.GetEndEffectorGripper(self.api)[0]):
                Dobot.gripperStatus.labels(self.id).state("enabled")
            else:
                Dobot.gripperStatus.labels(self.id).state("disabled")

        jogJoints = dTypeX.GetJOGJointParams(self.api)
        if self.section.getboolean("jogbasevelocity", fallback=Dobot.options["jogbasevelocity"]):
            Dobot.jogBaseVelocity.labels(self.id).set(jogJoints[0])

        if self.section.getboolean("jogreararmvelocity", fallback=Dobot.options["jogreararmvelocity"]):
            Dobot.jogRearArmVelocity.labels(self.id).set(jogJoints[1])

        if self.section.getboolean("jogforearmvelocity", fallback=Dobot.options["jogforearmvelocity"]):
            Dobot.jogForearmVelocity.labels(self.id).set(jogJoints[2])

        if self.section.getboolean("jogendeffectorvelocity", fallback=Dobot.options["jogendeffectorvelocity"]):
            Dobot.jogEndEffectorVelocity.labels(self.id).set(jogJoints[3])

        if self.section.getboolean("jogbaseacceleration", fallback=Dobot.options["jogbaseacceleration"]):
            Dobot.jogBaseAcceleration.labels(self.id).set(jogJoints[4])

        if self.section.getboolean("jogreararmacceleration", fallback=Dobot.options["jogreararmacceleration"]):
            Dobot.jogRearArmAcceleration.labels(self.id).set(jogJoints[5])

        if self.section.getboolean("jogforearmacceleration", fallback=Dobot.options["jogforearmacceleration"]):
            Dobot.jogForearmAcceleration.labels(self.id).set(jogJoints[6])

        if self.section.getboolean("jogendeffectoracceleration", fallback=Dobot.options["jogendeffectoracceleration"]):
            Dobot.jogEndEffectorAcceleration.labels(self.id).set(jogJoints[7])

        jogCoords = dTypeX.GetJOGCoordinateParams(self.api)
        if self.section.getboolean("jogaxisxvelocity", fallback=Dobot.options["jogaxisxvelocity"]):
            Dobot.jogAxisXVelocity.labels(self.id).set(jogCoords[0])

        if self.section.getboolean("jogaxisyvelocity", fallback=Dobot.options["jogaxisyvelocity"]):
            Dobot.jogAxisYVelocity.labels(self.id).set(jogCoords[1])

        if self.section.getboolean("jogaxiszvelocity", fallback=Dobot.options["jogaxiszvelocity"]):
            Dobot.jogAxisZVelocity.labels(self.id).set(jogCoords[2])

        if self.section.getboolean("jogaxisrvelocity", fallback=Dobot.options["jogaxisrvelocity"]):
            Dobot.jogAxisRVelocity.labels(self.id).set(jogCoords[3])

        if self.section.getboolean("jogaxisxacceleration", fallback=Dobot.options["jogaxisxacceleration"]):
            Dobot.jogAxisXAcceleration.labels(self.id).set(jogCoords[4])

        if self.section.getboolean("jogaxisyacceleration", fallback=Dobot.options["jogaxisyacceleration"]):
            Dobot.jogAxisYAcceleration.labels(self.id).set(jogCoords[5])

        if self.section.getboolean("jogaxiszacceleration", fallback=Dobot.options["jogaxiszacceleration"]):
            Dobot.jogAxisZAcceleration.labels(self.id).set(jogCoords[6])

        if self.section.getboolean("jogaxisracceleration", fallback=Dobot.options["jogaxisracceleration"]):
            Dobot.jogAxisRAcceleration.labels(self.id).set(jogCoords[7])

        jogCommon = dTypeX.GetJOGCommonParams(self.api)
        if self.section.getboolean("jogvelocityratio", fallback=Dobot.options["jogvelocityratio"]):
            Dobot.jogVelocityRatio.labels(self.id).set(jogCommon[0])

        if self.section.getboolean("jogaccelerationratio", fallback=Dobot.options["jogaccelerationratio"]):
            Dobot.jogAccelerationRatio.labels(self.id).set(jogCommon[1])

        ptpJoints = dTypeX.GetPTPJointParams(self.api)
        if self.section.getboolean("ptpbasevelocity", fallback=Dobot.options["ptpbasevelocity"]):
            Dobot.ptpBaseVelocity.labels(self.id).set(ptpJoints[0])

        if self.section.getboolean("ptpreararmvelocity", fallback=Dobot.options["ptpreararmvelocity"]):
            Dobot.ptpRearArmVelocity.labels(self.id).set(ptpJoints[1])

        if self.section.getboolean("ptpforearmvelocity", fallback=Dobot.options["ptpforearmvelocity"]):
            Dobot.ptpForearmVelocity.labels(self.id).set(ptpJoints[2])

        if self.section.getboolean("ptpendeffectorvelocity", fallback=Dobot.options["ptpendeffectorvelocity"]):
            Dobot.ptpEndEffectorVelocity.labels(self.id).set(ptpJoints[3])

        if self.section.getboolean("ptpbaseacceleration", fallback=Dobot.options["ptpbaseacceleration"]):
            Dobot.ptpBaseAcceleration.labels(self.id).set(ptpJoints[4])

        if self.section.getboolean("ptpreararmacceleration", fallback=Dobot.options["ptpreararmacceleration"]):
            Dobot.ptpRearArmAcceleration.labels(self.id).set(ptpJoints[5])

        if self.section.getboolean("ptpforearmacceleration", fallback=Dobot.options["ptpforearmacceleration"]):
            Dobot.ptpForearmAcceleration.labels(self.id).set(ptpJoints[6])

        if self.section.getboolean("ptpendeffectoracceleration", fallback=Dobot.options["ptpendeffectoracceleration"]):
            Dobot.ptpEndEffectorAcceleration.labels(self.id).set(ptpJoints[7])

        ptpCoords = dTypeX.GetPTPCoordinateParams(self.api)
        if self.section.getboolean("ptpxyzvelocity", fallback=Dobot.options["ptpxyzvelocity"]):
            Dobot.ptpXYZVelocity.labels(self.id).set(ptpCoords[0])

        if self.section.getboolean("ptprvelocity", fallback=Dobot.options["ptprvelocity"]):
            Dobot.ptpRVelocity.labels(self.id).set(ptpCoords[1])

        if self.section.getboolean("ptpxyzacceleration", fallback=Dobot.options["ptpxyzacceleration"]):
            Dobot.ptpXYZAcceleration.labels(self.id).set(ptpCoords[2])

        if self.section.getboolean("ptpracceleration", fallback=Dobot.options["ptpracceleration"]):
            Dobot.ptpRAcceleration.labels(self.id).set(ptpCoords[3])

        ptpCommon = dTypeX.GetPTPCommonParams(self.api)
        if self.section.getboolean("ptpvelocityratio", fallback=Dobot.options["ptpvelocityratio"]):
            Dobot.ptpVelocityRatio.labels(self.id).set(ptpCommon[0])

        if self.section.getboolean("ptpaccelerationratio", fallback=Dobot.options["ptpaccelerationratio"]):
            Dobot.ptpAccelerationRatio.labels(self.id).set(ptpCommon[1])

        ptpJump = dTypeX.GetPTPJumpParams(self.api)
        if self.section.getboolean("liftingheight", fallback=Dobot.options["liftingheight"]):
            Dobot.liftingHeight.labels(self.id).set(ptpJump[0])

        if self.section.getboolean("heighlimit", fallback=Dobot.options["heighlimit"]):
            Dobot.heightLimit.labels(self.id).set(ptpJump[1])

        cp = dTypeX.GetCPParams(self.api)
        if self.section.getboolean("cpvelocity", fallback=Dobot.options["cpvelocity"]):
            Dobot.cpVelocity.labels(self.id).set(cp[0])

        if self.section.getboolean("cpacceleration", fallback=Dobot.options["cpacceleration"]):
            Dobot.cpAcceleration.labels(self.id).set(cp[1])

        arc = dTypeX.GetARCParams(self.api)
        if self.section.getboolean("arcxyzvelocity", fallback=Dobot.options["arcxyzvelocity"]):
            Dobot.arcXYZVelocity.labels(self.id).set(arc[0])

        if self.section.getboolean("arcrvelocity", fallback=Dobot.options["arcrvelocity"]):
            Dobot.arcRVelocity.labels(self.id).set(arc[1])

        if self.section.getboolean("arcxyzacceleration", fallback=Dobot.options["arcxyzacceleration"]):
            Dobot.arcXYZAcceleration.labels(self.id).set(arc[2])

        if self.section.getboolean("arcracceleration", fallback=Dobot.options["arcracceleration"]):
            Dobot.arcRAcceleration.labels(self.id).set(arc[3])

        angleStaticErr = dTypeX.GetAngleSensorStaticError(self.api)
        if self.section.getboolean("anglestaticerrrear", fallback=Dobot.options["anglestaticerrrear"]):
            Dobot.angleStaticErrRear.labels(self.id).set(angleStaticErr[0])

        if self.section.getboolean("anglestaticerrfront", fallback=Dobot.options["anglestaticerrfront"]):
            Dobot.angleStaticErrFront.labels(self.id).set(angleStaticErr[1])

        angleCoef = dTypeX.GetAngleSensorCoef(self.api)
        if self.section.getboolean("anglecoefrear", fallback=Dobot.options["anglecoefrear"]):
            Dobot.angleCoefRear.labels(self.id).set(angleCoef[0])

        if self.section.getboolean("anglecoeffront", fallback=Dobot.options["anglecoeffront"]):
            Dobot.angleCoefFront.labels(self.id).set(angleCoef[1])

        if self.section.getboolean("slidingrailstatus", fallback=Dobot.options["slidingrailstatus"]):
            if bool(dTypeX.GetDeviceWithL(self.api)[0]):
                Dobot.slidingRailStatus.labels(self.id).state("enabled")
            else:
                Dobot.slidingRailStatus.labels(self.id).state("disabled")

        if self.section.getboolean("slidingrailpose", fallback=Dobot.options["slidingrailpose"]):
            Dobot.slidingRailPose.labels(self.id).set(dTypeX.GetPoseL(self.api)[0])

        jogRail = dTypeX.GetJOGLParams(self.api)
        if self.section.getboolean("slidingrailjogvelocity", fallback=Dobot.options["slidingrailjogvelocity"]):
            Dobot.slidingRailJogVelocity.labels(self.id).set(jogRail[0])

        if self.section.getboolean("slidingrailjogacceleration", fallback=Dobot.options["slidingrailjogacceleration"]):
            Dobot.slidingRailJogAcceleration.labels(self.id).set(jogRail[1])

        ptpRail = dTypeX.GetPTPLParams(self.api)
        if self.section.getboolean("slidingrailptpvelocity", fallback=Dobot.options["slidingrailptpvelocity"]):
            Dobot.slidingRailPtpVelocity.labels(self.id).set(ptpRail[0])

        if self.section.getboolean("slidingrailptpacceleration", fallback=Dobot.options["slidingrailptpacceleration"]):
            Dobot.slidingRailPtpAcceleration.labels(self.id).set(ptpRail[1])


        if self.section.getboolean("wifimodulestatus", fallback=Dobot.options["wifimodulestatus"]):
            if bool(dTypeX.GetWIFIConfigMode(self.api)[0]):
                Dobot.wifiModuleStatus.labels(self.id).state("enabled")
            else:
                Dobot.wifiModuleStatus.labels(self.id).state("disabled")

        if self.section.getboolean("wificonnectionstatus", fallback=Dobot.options["wificonnectionstatus"]):
            if bool(dTypeX.GetWIFIConnectStatus(self.api)[0]):
                Dobot.wifiConnectionStatus.labels(self.id).state("enabled")
            else:
                Dobot.wifiConnectionStatus.labels(self.id).state("disabled")

    def _disconnect(self):
        dTypeX.DisconnectAll()
        #dTypeX.DisconnectDobotX(self.api)

class Jevois(Device):
    options = {"objects":"","objectidentified":True,"objectlocation":True,"objectsize":False}

    objectLocationX = Gauge("object_location_x", "Identified object\"s x position", ["device"])
    objectLocationY = Gauge("object_location_y", "Identified object\"s y position", ["device"])
    objectLocationZ = Gauge("object_location_z", "Identified object\"s Z position", ["device"])
    objectSize = Gauge("object_size","Identified object\"s size", ["device"])

    def _connect(self):
        try:
            self.serial = serial.Serial(self.port, 115200, timeout=0)
            self.__prominit()
            return True
        except Exception as e:
            print("Couldn't connect with Jevois Camera device at port " + self.port + " (" + str(e) + ")")
            return False

    def __prominit(self):
        if self.section.getboolean("objectidentified", fallback=Dobot.options["objectidentified"]):
            if self.section.get("objects") is not None:
                self.objects = self.section["objects"].split()
                self.objectIdentified = Enum("object_identified_by_"+self.port, "Object Identified", states=self.objects)
            else:
                print("The \"objects\" list is necessary for monitoring identified objects")
                print("Skipping monitoring objects identified for Jevois:" + self.port)

    def _fetch(self):
        line = self.serial.readline().rstrip().decode()
        tok = line.split()

        # Abort fetching if timeout or malformed line
        if len(tok) < 1: return

        serstyle = tok[0][0]
        dimension = tok[0][1]

        # If the serstyle is not Normal (thus it is unsupported by the module)
        if (serstyle != "N"): return

        if dimension == "1" and len(tok) != 4: return
        if dimension == "2" and len(tok) != 6: return
        if dimension == "3" and len(tok) != 8: return

        if self.section.getboolean("objectidentified", fallback=Jevois.options["objectidentified"]):
            if self.objects is not None and tok[1] in self.objects:
                self.objectIdentified.state(tok[1])

        if self.section.getboolean("objectlocation", fallback=Jevois.options["objectlocation"]):
            Jevois.objectLocationX.labels(self.id).set(float(tok[2]))

            if int(dimension) > 1:
                Jevois.objectLocationY.labels(self.id).set(float(tok[3]))

            if int(dimension) == 3:
                Jevois.objectLOcationZ.labels(self.id).set(float(tok[4]))

        if self.section.getboolean("objectsize", fallback=Jevois.options["objectsize"]):
            if dimension == "1":
                Jevois.objectSize.labels(self.id).set(float(tok[3]))
            elif dimension == "2":
                Jevois.objectSize.labels(self.id).set(float(tok[4])*float(tok[5]))
            elif dimension == "3":
                Jevois.objectSize.labels(self.id).set(float(tok[5])*float(tok[6])*float(tok[7]))

    def _disconnect(self):
        self.serial.close()
