import getopt
import sys
import time
import DobotDllType as dType
from prometheus_client import start_http_server, Info, Gauge, Enum


class DobotMagician():
    # {bit address index : alarm description}
    alarms = {"0x00":"Public Alarm: Reset Alarm", "0x01":"Public Alarm: Undefined Instruction", "0x02":"Public Alarm: File System Error", "0x03":"Public Alarm: Failured Communication between MCU and FPGA", "0x04":"Public Alarm: Angle Sensor Reading Error",
              "0x11":"Planning Alarm: Inverse Resolve Alarm", "0x12":"Planning Alarm: Inverse Resolve Limit", "0x13":"Planning Alarm: Data Repetition", "0x14":"Planning Alarm: Arc Input Parameter Alarm", "0x15":"Planning Alarm: JUMP Parameter Error",
              "0x21":"Kinematic Alarm: Inverse Resolve Alarm", "0x22":"Kinematic Alarm: Inverse Resolve Limit",
              "0x40":"Limit Alarm: Joint 1 Positive Limit Alarm", "0x41":"Limit Alarm: Joint 1 Negative Limit Alarm", "0x42":"Limit Alarm: Joint 2 Positive Limit Alarm", "0x43":"Limit Alarm: Joint 2 Negative Limit Alarm", "0x44":"Limit Alarm: Joint 3 Positive Limit Alarm", "0x45":"Limit Alarm: Joint 3 Negative Limit Alarm", "0x46":"Limit Alarm: Joint 4 Positive Limit Alarm", "0x47":"Limit Alarm: Joint 4 Negative Limit Alarm", "0x48":"Limit Alarm: Parallegram Positive Limit Alarm", "0x49":"Limit Alarm: Parallegram Negative Limit Alarm",
			  "0":"OK"}

    def __init__(self, api):
        self.__api = api
        self.__dinfo = Info('dobot_magician_info', 'General device information')
        self.__dinfo.info({'version': str(dType.GetDeviceInfo(self.__api)),
                           'deviceName': str(dType.GetDeviceName(self.__api)),
                           'serialNumber': str(dType.GetDeviceSN(self.__api))})

        self.__poses_x = Gauge('pose_x', 'Real-time pose of robotic arm: X position')
        self.__poses_y = Gauge('pose_y', 'Real-time pose of robotic arm: Y position')
        self.__poses_z = Gauge('pose_z', 'Real-time pose of robotic arm: Z position')
        self.__poses_r = Gauge('pose_r', 'Real-time pose of robotic arm: R position')
        self.__kinematics_v = Gauge('kinematics_v', 'Kinematics velocity')
        self.__kinematics_a = Gauge('kinematics_a', 'Kinematics acceleration')
        self.__arm_orientation_l = Gauge('arm_orientation_l', 'Arm orientation left')
        self.__arm_orientation_r = Gauge('arm_orientation_r', 'Arm orientation right')
        self.__device_alarm = Enum('alarms', 'Device alarms',states=list(self.alarms.values()))
        # self.__triggerMode = Gauge('triggerMode', 'Handhold teaching trigger mode of saved points')
        # self.__jogJointParams = Gauge('jogJointParams', 'JOG joint parameters (velocity, acceleration)')
        # self.__jogCoordParams = Gauge('jogCoordParams', 'JOG coordinate parameters (velocity, acceleration)')
        # self.__jogCommonParams = Gauge('jogCommonParams', 'JOG common parameters (velocity ratio, acceleration ratio)')
        # self.__ptpJointParams = Gauge('ptpJointParams', 'PTP joint parameters (velocity, acceleration)')
        # self.__ptpCoordParams = Gauge('ptpCoordParams', 'PTP coordinate parameters (velocity, acceleration)')
        # self.__ptpCommonParams = Gauge('ptpCommonParams', 'PTP common parameters (velocity ratio, acceleration ratio)')
        # self.__ptpJumpParams = Gauge('ptpJumpParams', 'PTP jump parameters (jump height, zlimit)')

    def _setAlarms(self):
        alarmBytes = dType.GetAlarmsState(self.__api, 10)[0]

        # Convert Bytes to bits (as string for reading)
        bits = ''
        for byte in alarmBytes:
            bits += f'{byte:0>8b}'
            # print(f'{byte:0>8b}', end=' ')

		# If all bits are 0 then device state is clean/safe
		if bits.strip("0") == '':
		    self.__device_alarm.state("OK")
		else:
	        index = 0
	        for bit in bits:
	            if bit == '1':
	                self.__device_alarm.state(self.alarms.get('0x'+'{:02X}'.format(index)))
					hasError = True
	            index += 1


    def _fetchDobotData(self):
        poses = dType.GetPose(self.__api)
        kinematics = dType.GetKinematics(self.__api)
        armOrientation = dType.GetArmOrientation(self.__api)

        self.__poses_x.set(float(poses[0]))
        self.__poses_y.set(float(poses[1]))
        self.__poses_z.set(float(poses[2]))
        self.__poses_r.set(float(poses[3]))
        self.__kinematics_v.set(float(kinematics[0]))
        self.__kinematics_a.set(float(kinematics[1]))
        self.__arm_orientation_l.set(float(armOrientation[0]))
        self.__arm_orientation_r.set(float(armOrientation[1]))
        self._setAlarms()

        # self.__triggerMode.set(dType.GetHHTTrigMode(self.__api)[0])
        # self.__jogJointParams.set(dType.GetJOGJointParams(self.__api))
        # self.__jogCoordParams.set(dType.GetJOGCoordinateParams(self.__api))
        # self.__jogCommonParams.set(dType.GetJOGCommonParams(self.__api))
        # self.__ptpJointParams.set(dType.GetPTPJointParams(self.__api))
        # self.__ptpCoordParams.set(dType.GetPTPCoordinateParams(self.__api))
        # self.__ptpJumpParams.set(dType.GetPTPJumpParams(self.__api))
        # self.__ptpCommonParams.set(dType.GetPTPCommonParams(self.__api))

    def _getDobotApi(self):
        return self.__api


class JevoisCamera():
    def __init__(self, cap):
        self.__cap = cap

    def _fetchJevoisData(self):
        pass


class MonitoringAgent():
    def __init__(self, name, timeoutPeriod, port):
        self.__name = "Agent0"
        self.__timeoutPeriod = 100
        self.__port = 8000

        if name is not None:
            self.__name = name

        if timeoutPeriod is not None:
            self.__timeoutPeriod = timeoutPeriod

        if port is not None:
            self.__port = port

        self.__fetchTimes = []
        self.__startTime = None
        self.__jevois = None
        self.__dobot = None

    def getAgentRuntime(self):
        if self.__startTime is None:
            print('There is no active routine on this agent.')
            return

        return time.perf_counter() - self.__startTime

    def connectDobot(self):
        # Load Dll and get the CDLL object
        api = dType.load()
        # Connect Dobot
        state = dType.ConnectDobot(api, "", 115200)[0]

        if state == dType.DobotConnect.DobotConnect_NoError:
            self.__dobot = DobotMagician(api)
            print('Dobot Magician connected succesfully!')
            return 0
        else:
            print("Couldn't connect with a Dobot Magician device")
            return 1

    def disconnectDobot(self):
        if self.__dobot is not None:
            dType.DisconnectDobot(self.__dobot._getDobotApi())

    def connectJevois(self):
        pass

    def disconnectJevois(self):
        pass

    def __fetchData(self):
        if self.__dobot is not None:
            self.__dobot._fetchDobotData()

        if self.__jevois is not None:
            self.__jevois._fetchJevoisData()

    def startRoutine(self, duration):

        if self.__dobot is None and self.__jevois is None:
            print("No devices connected to the agent.")
            print("Run connectDobot() or connectJevois() before starting a routine")
            sys.exit(11)

        print('Monitoring..')

        start_http_server(self.__port)
        self.__startTime = time.perf_counter()

        while (1):
            time.sleep(self.__timeoutPeriod)
            self.__fetchData()

            if duration is not None:
                duration = int(duration / 60) - int(self.getAgentRuntime())
                if duration <= 0:
                    break

    def getTimeoutPeriod(self):
        return self.__timeoutPeriod

def printHelp():
    print('Monitoring Agent for Dobot Magician and Jevois Camera\n')
    print('Options:')
    print('-n <agent_name>\t\tGive symbolic name to this monitoring agent (default: Agent0)')
    print('-t <number>\t\tSet timeout period between each routine cycle in milliseconds (min/default: 100)')
    print('-a <number>\t\tDefine time in minutes for how long should the monitoring last (default: until interrupt)')
    print('-d\t\tSearch, connect and monitor a Dobot Magician device')
    print('-j\t\tSearch, connect and monitor a Jevois Camera device')
    print('-p\t\tSet port for prometheus (default: 8000)')
    print('-h\t\tPrint this message')

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'djhp:n:t:')
    except getopt.GetoptError as err:
        print(str(err))
        sys.exit(2)

    agentName = None
    timeoutPeriod = None
    duration = None
    port = None

    for opt, arg in opts:
        if opt == '-h':
            printHelp()
            sys.exit()
        elif opt == '-n':
            agentName = arg
        elif opt == '-t':
            try:
                timeoutPeriod = int(arg)
                if (timeoutPeriod < 100):
                    raise ValueError
            except ValueError:
                print('Timeout must be a number greater or equal to 100')
                sys.exit(3)
        elif opt == '-a':
            try:
                duration = int(arg)
            except ValueError:
                print('Alive duration must be a number')
                sys.exit(4)
        elif opt == '-p':
            try:
                port = int(arg)
                if port > 65535 or port < 0:
                    print(str(port) + " is not a valid port")
                    raise ValueError
            except ValueError:
                print('Port must be a number from 0 to 65535')
                sys.exit(5)


    Agent = MonitoringAgent(agentName, timeoutPeriod, port)

    for opt, arg in opts:
        if opt == '-d':
            Agent.connectDobot()
        elif opt == '-j':
            #Agent.connectJevois()
            pass

    Agent.startRoutine(duration)

if __name__ == '__main__':
    main()
