import sys
import time
import webbrowser
import configparser
from threading import Thread
from prometheus_client import start_http_server
from device_modules import *

class Agent():
    options = {"agentname":"Agent0","prometheusport":8000,"verbose":False}

    def __init__(self):
        self.config = configparser.ConfigParser()
        self.__readConfig()
        self.validSections = self.__validateConfig()

        self.devices = []
        self.agentName = self.config.get("Agent", "agentname", fallback=Agent.options["agentname"])
        if self.agentName == "":
            self.agentName = "Agent0"

        try:
            self.prometheusPort = self.config.getint("Agent", "prometheusport", fallback=Agent.options["prometheusport"])
            if self.prometheusPort > 65535 or self.prometheusPort < 0:
                print(str(self.prometheusPort) + " is not a valid port")
                raise ValueError
        except ValueError:
            print("PrometheusPort must be a number from 0 to 65535")
            sys.exit(5)

        self.verbose = self.config.getboolean("Agent", "verbose", fallback=Agent.options["verbose"])

    def __readConfig(self):
        try:
            check = self.config.read("agent.conf")[0]
            if check == "":
                raise Exception("Couldn't find agent.conf")

            print("Reading agent.conf..")
        except Exception as e:
            print("Can't read configuration file. Make sure agent.conf is in the same directory as agent.py (error: " + str(e) + ")")
            exit(3)

    def __validateConfig(self):
        validBooleanValues = ["1","yes","true","on","0","no","false","off"]
        validSections = {}

        # Check Agent section first and seperately from device sections
        try:
            if "Agent" not in self.config.sections():
                raise Exception("[ERROR] Missing mandatory \"Agent\" section in \"agent.conf\"")

            for option in self.config["Agent"]:
                if option not in Agent.options:
                    raise Exception("[ERROR] Agent does not support the \"" + option + "\" option")

                configValue = self.config["Agent"][option]
                optionsValue = Agent.options[option]

                if isinstance(optionsValue, bool) and configValue not in validBooleanValues:
                    raise Exception("[ERROR] Value \"" + configValue + "\" for option \"" + option + "\" in section \"Agent\" is not valid (must be one of the following: 1,yes,true,on,0,no,false,off)")

                type(optionsValue)(configValue)
        except Exception as e:
            print(str(e))
            print("Exiting..")
            exit(4)

        deviceSections = self.config.sections()
        deviceSections.remove("Agent")
        for sectionName in deviceSections:
            try:
                part = sectionName.split(":")

                if len(part) != 2:
                    raise Exception("[ERROR] " + sectionName + " is not a valid device entry. All device entries should follow this format [DEVICE_TYPE:PORT]")

                deviceType = part[0]
                connectionPort = part[1]

                if deviceType not in globals():
                    raise Exception("[ERROR] The agent does not support the \"" + deviceType + "\" device type. Make sure the appropriate device module exists in device_module.py (case-sensitive)")

                section = self.config[sectionName]
                entityClass = globals()[deviceType]
                device = entityClass(section, connectionPort)

                if entityClass.options == Device.options or len(entityClass.options) == 0:
                    raise Exception("[ERROR] Cannot validate \"" + deviceType + "\". Make sure the options{} dictionary is implemented.")
            except Exception as e:
                print(str(e) + ". " + sectionName + " entry will be ignored. For more information use --help.")
                continue

            errorCount = 0
            options = {}
            options.update(entityClass.options)
            options.update(Device.options)
            for option in section:
                # Check if key is valid (supported by module)
                if option not in options:
                    print("[ERROR] \"" + option + "\" is not a valid option for section \"" + sectionName + "\".")
                    errorCount += 1
                    continue

                configValue = section[option]
                optionsValue = options[option]

                # Check if value type is correct
                if isinstance(optionsValue, bool) and configValue not in validBooleanValues:
                    print("[ERROR] Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + sectionName +"\" is not valid (must be one of the following: 1,yes,true,on,0,no,false,off)")
                    errorCount += 1
                    continue

                try:
                    type(optionsValue)(configValue)
                except:
                    print("[ERROR] Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + sectionName +"\" is not valid (must be of type " + str(type(optionsValue).__name__) + ").")
                    errorCount += 1
                    continue

            if errorCount > 0:
                print(str(errorCount) + " errors in " + sectionName + ". Please resolve the errors in order for " + sectionName + " to be monitored.")
            else:
                validSections[sectionName] = device

        return validSections

    def __connectDevices(self):
        # Discover through the validated config which devices should be monitored
        for sectionName in self.validSections:
            device = self.validSections[sectionName]
            if device._connect():
                self.devices.append(device)
                print("[OK] " + deviceType + " at port " + connectionPort + " connected succesfully!")
            else:
                print("[ERROR] " + deviceType + " at port " + connectionPort + " cannot be connected.")
                print("[WARNING] Device " + deviceType + " at " + connectionPort + " will not be monitored.")

    def __disconnectDevices(self):
        for device in self.devices:
            device._disconnect()

            if self.verbose:
                print("Disconnected device " + device.id)

    def __fetchFrom(self, device):
        while 1:
            if self.verbose:
                start = time.time()
                device._fetch(fetchedBy = self.agentName)
                elapsed = time.time() - start
                print("Fetched from " + device.id + " in " + str(round(elapsed*1000)) + " ms")
            else:
                device._fetch(fetchedBy = self.agentName)

            time.sleep(device.timeout / 1000)

    def startRoutine(self):
        print("Connecting to devices listed in agent.conf..")
        self.__connectDevices()

        if len(self.devices) == 0:
            print("No devices connected to the agent.")
            print("Exiting..")
            sys.exit(11)

        print("Starting prometheus server at port " + str(self.prometheusPort) + "..")
        start_http_server(self.prometheusPort)

        try:
            for device in self.devices:
                Thread(target = self.__fetchFrom, args=(device,)).start()
                if self.verbose:
                    print("[" + self.agentName + "] Started monitoring for device " + device.id + " with " + str(len(device.section)) + " active attributes")

            if !self.verbose:
                print("[" + self.agentName + "] Monitoring.. ")

            while 1:
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

def main():
    argumentHandler(sys.argv)
    Agent().startRoutine()

if __name__ == "__main__":
    main()
