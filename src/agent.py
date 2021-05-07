import sys
import time
import webbrowser
import configparser
from threading import Thread
from prometheus_client import start_http_server
from device_modules import Dobot, Jevois
import cProfile

config = configparser.ConfigParser()

class Agent():
    options = {"agentname":"Agent0","prometheusport":8000}

    def __init__(self):
        global config

        self.devices = []
        self.agentName = config.get("Agent", "agentname", fallback=Agent.options["agentname"])
        if self.agentName == "":
            self.agentName = "Agent0"

        try:
            self.prometheusPort = config.getint("Agent", "prometheusport", fallback=Agent.options["prometheusport"])
            if self.prometheusPort > 65535 or self.prometheusPort < 0:
                print(str(self.prometheusPort) + " is not a valid port")
                raise ValueError
        except ValueError:
            print("PrometheusPort must be a number from 0 to 65535")
            sys.exit(5)

    def __connectDevices(self):
        global config

        # Discover through the config which devices should be monitored
        for section in config:
            # Skip the Agent and DEFAULT sections as they do not represent a device
            if section == "Agent" or section == "DEFAULT":
                continue

            try:
                part = section.split(":")
                if (len(part) != 2):
                    raise Exception("[ERROR] " + section + " is not a valid device entry. All device entries should follow this format [DEVICE_TYPE:PORT]")
                deviceType = part[0]
                connectionPort = part[1]

                if deviceType not in globals():
                    raise Exception("[ERROR] The agent does not support " + deviceType)
            except Exception as e:
                print(str(e))
                print("For more information use --help")
                continue

            constructor = globals()[deviceType]
            device = constructor(config, connectionPort)

            if device._connect():
                self.devices.append(device)
                print("[OK] " + deviceType + " at port " + connectionPort + " connected succesfully!")
            else:
                print("[ERROR] " + deviceType + " at port " + connectionPort + " cannot be connected.")
                print("[WARNING] Device " + deviceType + " at " + connectionPort + " will not be monitored.")

    def __disconnectDevices(self):
        for device in self.devices:
            device._disconnect()


    def __fetchFrom(self, device):
        while (1):
            start = time.time()
            device._fetch()
            elapsed = time.time() - start
            print("Fetched from " + device.id + " in " + str(round(elapsed*1000)) + " ms")
            time.sleep(device.timeout / 1000)

    def startRoutine(self):
        print("Connecting to devices listed in agent.conf..")
        self.__connectDevices()

        if len(self.devices) == 0:
            print("No devices connected to the agent.")
            sys.exit(11)

        print("Starting prometheus server at port " + str(self.prometheusPort) + "..")
        start_http_server(self.prometheusPort)

        try:
            threads = []
            for device in self.devices:
                threads.append(Thread(target = self.__fetchFrom, args=(device,)))

            for thread in threads:
                thread.start()

            print("Monitoring..")
            while (1):
                pass
        except KeyboardInterrupt:
            print("Disconnecting devices..")
            self.__disconnectDevices()
            exit(0)


def argumentHandler(args):
    if len(args) == 2:
        if args[1] == "-h" or args[1] == "--help":
            webbrowser.open("../README.md")
            exit(1)
        else:
            print("Unrecognised option \"" + sys.argv[1] + "\"")
            print("For more information: $ agent.py --help")
            exit(2)

def validateConfig():
    global config
    validBooleanValues = ["1","yes","true","on","0","no","false","off"]
    flag = False

    for section in config.sections():
        entityType = section.split(":")[0]

        try:
            entityClass = globals()[entityType]

            if entityClass.options is None:
                raise Exception()
        except Exception as e:
            print("[ERROR] \"" + section + "\" cannot be recognised for validation. Make sure " + entityType + "'s module exists in device_modules and that it implements the static options dictionary (" + str(e) + ")")
            continue

        for option in config[section]:
            configValue = config[section][option]
            optionsValue = entityClass.options[option]

            if option not in entityClass.options:
                print("[ERROR] \"" + option + "\" is not a valid option for section \"" + section + "\" and will be ignored.")
                flag = True
                continue

            if isinstance(optionsValue, bool) and configValue not in validBooleanValues:
                print("[ERROR] Value \"" +  + "\" for option \"" + option + "\" in section \"" + section +"\" is not valid (must be one of the following: 1,yes,true,on,0,no,false,off)")
                flag = True
                continue

            try:
                type(optionsValue)(configValue)
            except:
                print("[ERROR] Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + section +"\" is not valid (must be of type " + str(type(optionsValue)) + ") and the option will be set to default (" + str(optionsValue) + ")")
                flag = True
                continue

    if flag:
        print("Please resolve the errors in the configuration file in order for the agent to run properly.")
        exit(10)

def readConfig():
    global config

    try:
        check = config.read("agent.conf")[0]
        if check == "":
            raise Exception("Couldn't find agent.conf")

        print("Reading agent.conf..")
    except Exception as e:
        print("Can't open configuration file. Make sure agent.conf is in the same directory as agent.py (" + str(e) + ")")
        exit(3)

def main():
    argumentHandler(sys.argv)
    readConfig()
    validateConfig()
    agent = Agent()
    agent.startRoutine()

if __name__ == "__main__":
    main()
