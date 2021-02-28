import sys
import time
import serial
import webbrowser
import configparser
from prometheus_client import start_http_server, Info, Gauge, Enum
from device_modules import *

config = configparser.ConfigParser()

class Agent():
    configValidOptions = ["agentname","routineinterval","prometheusport"]
    configIgnoreValueCheck = ["agentname","routineinterval","prometheusport"]

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

    def __connectDevices(self):
        global config

        # Discover through the config which devices should be monitored
        for section in config:
            # Skip the Agent section as it does not represent a device
            if section == 'AGENT':
                continue

            try:
                part = section.split(':')
                if (len(part) != 2):
                    raise Exception("[ERROR] " + section + " is not a valid device entry. All device entries should follow this format [DEVICE_TYPE:PORT]'")
                deviceType = part[0]
                connectionPort = part[1]

                if deviceType not in globals():
                    raise Exception("[ERROR] The agent does not support " + deviceType)
            except Exception as e:
                print(str(e))
                print('For more information use --help')
                continue

            constructor = globals()[deviceType]
            device = constructor(connectionPort)

            if device._connect():
                devices.append(device)
                print("[OK] " + deviceType + " at port " + connectionPort + " connected succesfully!")
            else:
                print("[ERROR] " + deviceType + " at port " + connectionPort + " cannot be connected.")
                print("[WARNING] Device " + deviceType + " at " + connectionPort + " will not be monitored.")

    def __disconnectDevices(self):
        for device in devices:
            device._disconnect()


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
                    device._fetch()
        except KeyboardInterrupt:
            print('Disconnecting devices..')
            self.__disconnectDevices()


def argumentHandler(args):
    if len(args) == 2:
        if args[1] == "-h" or args[1] == "--help":
            # Open latest README.md documentation of project
            webbrowser.open('https://github.com/akomis/diploma-project/blob/master/README.md')
            exit(1)
        else:
            print('Unrecognised option \"' + sys.argv[1] + '\"')
            print('For more information: $ agent.py --help')
            exit(2)

def validateConfig():
    global config
    validValues = ["1","yes","true","on","0","no","false","off"]

    for section in config.sections():
        entityType = section.split(':')[0]

        try:
            entityClass = globals()[entityType]

            if entityClass.configValidOptions is None:
                raise Exception()
        except:
            print("[WARNING] \"" + section + "\" cannot be recognised for validation. Make sure " + entityType + "'s module exists in device_modules and that it implements the static configValidOptions list.")
            continue

        for option in config[section]:
            if option not in entityClass.configValidOptions:
                print("[WARNING] \"" + option + "\" is not a valid option for section \"" + section + "\" and will be ignored.")

            if option not in entityClass.configIgnoreValueCheck and config[section][option] not in validValues:
                print("[WARNING] Value \"" + config[section][option] + "\" for option \"" + option + "\" in section \"" + section +"\" is not valid and the option will be set to default")

def readConfig():
    global config

    print("Opening")
    try:
        check = config.read('agent.conf')[0]
        if check == '':
            raise Exception("Couldn't find agent.conf")

        validateConfig()
    except:
        print("Can't open configuration file. Make sure agent.conf is in the same directory as agent.py")
        exit(3)


def main():
    argumentHandler(sys.argv)
    readConfig()
    Agent = MonitoringAgent()
    Agent.startRoutine()

if __name__ == '__main__':
    main()
