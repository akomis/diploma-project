import sys
import time
import argparse
import webbrowser
import configparser
from threading import Thread
from prometheus_client import start_http_server
from device_modules import *

class Agent():
    def __init__(self, configFileName, name, prometheusPort, killSwitch, verbose):
        self.config = configparser.ConfigParser()
        self.__readConfig(configFileName)
        self.validSections = self.__validateConfig()

        self.name = name
        self.prometheusPort = prometheusPort
        self.verbose = verbose
        self.killSwitch = killSwitch

    def __readConfig(self, filename):
        try:
            check = self.config.read(filename)[0]
            if check == "":
                raise Exception("Couldn't find \"" + filename + "\"")

            print("Reading configuration file..")
        except Exception as e:
            print("Can't read configuration file \"" + filename + "\" (error: " + str(e) + ")")
            exit(3)

    def __validateConfig(self):
        validBooleanValues = ["1","yes","true","on","0","no","false","off"]
        validSections = {}

        # Check configuration validity for the defined (for monitoring) devices
        flag = False
        deviceSections = self.config.sections()
        deviceSections.remove("Agent")
        for sectionName in deviceSections:
            try:
                part = sectionName.split(":")

                if len(part) != 2:
                    raise Exception(sectionName + " is not a valid device entry. All device entries should follow this format [DEVICE_TYPE:PORT]")

                deviceType = part[0]
                connectionPort = part[1]

                if deviceType not in globals():
                    raise Exception("The agent does not support the \"" + deviceType + "\" device type. Make sure the appropriate device module exists in device_module.py (case-sensitive)")

                section = self.config[sectionName]
                entityClass = globals()[deviceType]

                if entityClass.options == Device.options or len(entityClass.options) == 0:
                    raise Exception("Cannot validate \"" + deviceType + "\". Make sure the options{} dictionary is implemented.")
            except Exception as e:
                flag = True
                print("[ERROR] " + str(e), file=sys.stderr)
                print("[WARNING] \"" + sectionName + "\" device will not be monitored.")
                continue

            errorCount = 0
            options = {}
            options.update(entityClass.options)
            options.update(Device.options)
            for option in section:
                try:
                    # Check if key is valid (supported by module)
                    if option not in options:
                        raise Exception(option + "\" is not a valid option for section \"" + sectionName + "\".")

                    configValue = section[option]
                    optionsValue = options[option]

                    # Check if value type is correct
                    if isinstance(optionsValue, bool) and configValue not in validBooleanValues:
                        raise Exception("Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + sectionName +"\" is not valid (must be one of the following: 1,yes,true,on,0,no,false,off)")

                    try:
                        type(optionsValue)(configValue)
                    except:
                        raise Exception("Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + sectionName +"\" is not valid (must be of type " + str(type(optionsValue).__name__) + ").")
                except Exception as e:
                    flag = True
                    errorCount += 1
                    print("[ERROR] " + str(e), file=sys.stderr)

            if errorCount > 0:
                print("[WARNING] " + str(errorCount) + " error(s) in \"" + sectionName + "\" section. The device will not be monitored. Please resolve the errors in order for " + sectionName + " to be monitored.")
            else:
                try:
                    device = entityClass(section, connectionPort)
                except:
                    flag = True
                    print("[ERROR] \"" + deviceType +"\" device module does not properly implement the Device interface", file=sys.stderr)
                    print("[WARNING] \"" + sectionName + "\" device will not be monitored.")
                    continue

                validSections[sectionName] = device

        if flag:
            print("For more information use --help.")
            if self.killSwitch:
                print("Killswitch enabled. Exiting..")
                exit(6)

        return validSections

    def __connectDevices(self):
        # Discover through the validated config which devices should be monitored
        for sectionName in self.validSections:
            device = self.validSections[sectionName]
            if device._connect():
                self.devices.append(device)
                print("[OK] " + deviceType + " at port " + connectionPort + " connected succesfully!")
            else:
                sys.stderr.write("[ERROR] " + deviceType + " at port " + connectionPort + " cannot be connected.")
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
                device._fetch(fetchedBy = self.name)
                elapsed = time.time() - start
                print("Fetched from " + device.id + " in " + str(round(elapsed*1000)) + " ms")
            else:
                device._fetch(fetchedBy = self.name)

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
                    print("[" + self.name + "] Started monitoring for device " + device.id + " with " + str(len(device.section)) + " active attributes")

            if not self.verbose:
                print("[" + self.name + "] Monitoring.. ")

            while 1:
                pass
        except KeyboardInterrupt:
            print("[" + self.name + "] Disconnecting devices..")
            self.__disconnectDevices()
            exit(0)

def isPort(value):
    port = 0
    try:
        port = int(value)
        if port > 65535 or port < 0:
            raise Exception()
    except Exception as e:
        raise argparse.ArgumentTypeError("\"%s\" is not a valid port number (must be an integer between 1 and 65535)" % value)

    return port

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", default="agent.conf", help="specify configuration file path (default: \".\\agent.conf\")")
    parser.add_argument("-n", "--name", default="Agent0", help="specify symbolic agent/station name used for seperation/grouping of stations (default: \"Agent0\")")
    parser.add_argument("-p", "--promport", type=isPort, default=8000, help="specify port number for Prometheus endpoint (default: 8000)")
    parser.add_argument("-k", "--killswitch", action="store_true", help="exit agent if at least 1 error exists in configuration file")
    parser.add_argument("-v", "--verbose", action="store_true", help="print actions with details in standard output")
    parser.add_argument("-m", "--more", action="store_true", help="open README.md with configuration and implementation details")
    args = parser.parse_args()

    if args.more:
        webbrowser.open("../README.md")

    Agent(args.config, args.name, args.promport, args.killswitch, args.verbose).startRoutine()

if __name__ == "__main__":
    main()
