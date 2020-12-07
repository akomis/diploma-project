import configparser
import sys
import time
import DobotDllType as dType
from prometheus_client import start_http_server, Info, Gauge, Enum

config = configparser.ConfigParser()


class DobotMagician():
    # {bit address index : alarm description}
    alarms = {"0x00": "Public Alarm: Reset Alarm", "0x01": "Public Alarm: Undefined Instruction", "0x02": "Public Alarm: File System Error", "0x03": "Public Alarm: Failured Communication between MCU and FPGA", "0x04": "Public Alarm: Angle Sensor Reading Error",
              "0x11": "Planning Alarm: Inverse Resolve Alarm", "0x12": "Planning Alarm: Inverse Resolve Limit", "0x13": "Planning Alarm: Data Repetition", "0x14": "Planning Alarm: Arc Input Parameter Alarm", "0x15": "Planning Alarm: JUMP Parameter Error",
              "0x21": "Kinematic Alarm: Inverse Resolve Alarm", "0x22": "Kinematic Alarm: Inverse Resolve Limit",
              "0x40": "Limit Alarm: Joint 1 Positive Limit Alarm", "0x41": "Limit Alarm: Joint 1 Negative Limit Alarm", "0x42": "Limit Alarm: Joint 2 Positive Limit Alarm", "0x43": "Limit Alarm: Joint 2 Negative Limit Alarm", "0x44": "Limit Alarm: Joint 3 Positive Limit Alarm", "0x45": "Limit Alarm: Joint 3 Negative Limit Alarm", "0x46": "Limit Alarm: Joint 4 Positive Limit Alarm", "0x47": "Limit Alarm: Joint 4 Negative Limit Alarm", "0x48": "Limit Alarm: Parallegram Positive Limit Alarm", "0x49": "Limit Alarm: Parallegram Negative Limit Alarm"}


    def __init__(self, api):
        global config
        self.__api = api
        self.__dinfo = Info('dobot_magician', 'General device information')
        self.__dinfo.info({'version': str(dType.GetDeviceInfo(self.__api)[0]),
                           'deviceName': str(dType.GetDeviceName(self.__api)[0]),
                           'serialNumber': str(dType.GetDeviceSN(self.__api)[0])})

        self.__device_alarms = Enum('alarms', 'Device alarms', states=list(self.alarms.values()))

    def _getAlarms(self):
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
                    self.__device_alarms.state(self.alarms[alarm])

    def _fetchDobotData(self):
        self._getAlarms()

    def _getDobotApi(self):
        return self.__api


class JevoisCamera():
    def __init__(self, cap):
        self.__cap = cap

    def _fetchJevoisData(self):
        pass


class MonitoringAgent():
    def __init__(self, name, interval, prometheusPort):
        self.__name = name
        self.__interval = interval
        self.__port = prometheusPort
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

    def startRoutine(self):
        if self.__dobot is None and self.__jevois is None:
            print("No devices connected to the agent.")
            print("Run with -d to connect to a dobot magician robot")
            print("Run with -j to connect to a jevois camera")
            sys.exit(11)

        start_http_server(self.__port)
        self.__startTime = time.perf_counter()

        print('Monitoring..')
        while (1):
            time.sleep(self.__interval / 1000)
            self.__fetchData()


def readAgentSettings():
    global config

    name = config.get('AGENT', 'AgentName', fallback="Agent0")
    if name == '':
        name = "Agent0"

    try:
        interval = config.getint('AGENT', 'routineInterval', fallback=100)
        if (interval < 100):
            raise ValueError
    except ValueError:
        print('RoutineInterval must be a number greater or equal to 100')
        sys.exit(4)

    try:
        port = config.getint('AGENT', 'PrometheusPort', fallback=8080)
        if port > 65535 or port < 0:
            print(str(port) + " is not a valid port")
            raise ValueError
    except ValueError:
        print('PrometheusPort must be a number from 0 to 65535')
        sys.exit(5)

    return name, interval, port

def printHelp():
    print('Monitoring Agent for Dobot Magician and Jevois Camera')
    print('Usage: $ python3 agent.py')


def main():
    global config
    if sys.argv[1] == "-h" or sys.argv[1] == "--help":
        printHelp()
        exit(1)

    try:
        config.read('agent.conf')
    except:
        print("Cant open configuration file. Make sure agent.conf is in the same directory as agent.py")
        exit(2)

    agentName, routineInterval, prometheusPort = readAgentSettings()
    Agent = MonitoringAgent(agentName, routineInterval, prometheusPort)

    if 'DOBOT' in config.sections():
        Agent.connectDobot()

    if 'JEVOIS' in config.sections():
        Agent.connectJevois()

    Agent.startRoutine()


if __name__ == '__main__':
    main()
