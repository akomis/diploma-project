import getopt
import sys
import time
import DobotDllType as dType
from prometheus_client import start_http_server, Info, Histogram


class DobotMagician():
    def __init__(self,  api):
        self.__api = api
        self.__dinfo = Info('dobot_magician_info', 'General device information')
        self.__dinfo.info({'version': str(dType.GetDeviceInfo(self.__api)),
                           'deviceName': str(dType.GetDeviceName(self.__api)),
                           'serialNumber':str(dType.GetDeviceSN(self.__api))})

        self.__poses = Histogram('poses', 'Real-time pose of robotic arm')
        self.__kinematics = Histogram('kinematics', 'Kinematics parameter')
        self.__alarmState = Histogram('alarmState', 'Device alarms')
        self.__armOrientation = Histogram('armOrientation', 'Arm orientation')
        self.__triggerMode = Histogram('triggerMode', 'Handhold teaching trigger mode of saved points')
        self.__jogJointParams = Histogram('jogJointParams', 'JOG joint parameters (velocity, acceleration)')
        self.__jogCoordParams = Histogram('jogCoordParams', 'JOG coordinate parameters (velocity, acceleration)')
        self.__jogCommonParams = Histogram('jogCommonParams', 'JOG common parameters (velocity ratio, acceleration ratio)')
        self.__ptpJointParams = Histogram('ptpJointParams', 'PTP joint parameters (velocity, acceleration)')
        self.__ptpCoordParams = Histogram('ptpCoordParams', 'PTP coordinate parameters (velocity, acceleration)')
        self.__ptpCommonParams = Histogram('ptpCommonParams', 'PTP common parameters (velocity ratio, acceleration ratio)')
        self.__ptpJumpParams = Histogram('ptpJumpParams', 'PTP jump parameters (jump height, zlimit)')


    def _fetchDobotData(self):
        self.__poses.observe(dType.GetPose(self.__api))
        self.__kinematics.observe(dType.GetKinematics(self.__api))
        self.__alarmState.observe(dType.GetAlarmsState(self.__api)[1])
        self.__triggerMode.observe(dType.GetHHTTrigMode(self.__api)[0])
        self.__armOrientation.observe(dType.GetArmOrientation(self.__api)[0])
        self.__jogJointParams.observe(dType.GetJOGJointParams(self.__api))
        self.__jogCoordParams.observe(dType.GetJOGCoordinateParams(self.__api))
        self.__jogCommonParams.observe(dType.GetJOGCommonParams(self.__api))
        self.__ptpJointParams.observe(dType.GetPTPJointParams(self.__api))
        self.__ptpCoordParams.observe(dType.GetPTPCoordinateParams(self.__api))
        self.__ptpJumpParams.observe(dType.GetPTPJumpParams(self.__api))
        self.__ptpCommonParams.observe(dType.GetPTPCommonParams(self.__api))

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