import sys
import time
import argparse
import webbrowser
import configparser
from threading import Thread
from prometheus_client import start_http_server
from device_modules import *

class Agent():
    termcolors = {
    "B":"\033[1m","U":"\033[4m","H":"\033[95m",
    "OK":"\033[92m","INFO":"\033[94m",
    "WARNING":"\033[93m","ERROR":"\033[91m",
    "END":"\033[0m"}

    def __init__(self, devicesFilename:str, name:str, prometheusPort:int, killSwitch:bool, verbose:bool, color:bool):
        self.stopped = False
        self.config = configparser.ConfigParser()
        self.devicesFilename = devicesFilename
        self.name = name
        self.prometheusPort = prometheusPort
        self.killSwitch = killSwitch
        self.verbose = verbose
        if not color:
            for attr in Agent.termcolors:
                Agent.termcolors[attr] = ""

        self.__readConfig()
        self.validSections = self.__validateConfig()
        self.devices = []

    def agentPrint(self, s, type=""):
        termcolors = Agent.termcolors
        prefix = self.name + " (" + time.ctime() + "): "
        body = s + termcolors["END"]

        if type == "i":
            print(prefix + termcolors["INFO"] +  "[INFO] " + body)
        elif type == "o":
            print(prefix + termcolors["OK"] + "[OK] " + body)
        elif type == "w":
            print(prefix + termcolors["WARNING"] + "[WARNING] " + body)
        elif type == "e":
            print(prefix + termcolors["ERROR"] + "[ERROR] " + body, file=sys.stderr)
        elif type == "f":
            print(prefix + termcolors["ERROR"] + "[" + termcolors["U"] + "FATAL" + termcolors["END"] + termcolors["ERROR"] + "] " + body, file=sys.stderr)
        else:
            print(prefix + s)

    def __readConfig(self):
        try:
            check = self.config.read(self.devicesFilename)

            if len(check) == 0:
                raise Exception("Couldn't find file")
        except Exception as e:
            self.agentPrint("Can't read configuration file \"" + self.devicesFilename + "\" (" + str(e) + ")", type="e")
            exit(3)

        self.agentPrint("Succesfully opened configuration file \"" + self.devicesFilename + "\"", type="o")

    def __validateConfig(self):
        self.agentPrint("Validating configuration file..", type="i")

        validBooleanValues = ["1","yes","true","on","0","no","false","off"]
        validSections = {}

        # Check configuration validity for each defined devices
        flag = False
        for sectionName in self.config.sections():
            try:
                part = sectionName.split(":")

                if len(part) != 2:
                    raise Exception("\"" + sectionName + " is not a valid device entry. All device entries should follow this format [DEVICE_TYPE:PORT]")

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
                self.agentPrint(str(e), type="e")
                self.agentPrint("\"" + sectionName + "\" device will not be monitored.", type="w")
                continue

            # Check if device fields in configuration are valid (supported by module)
            options = {}
            options.update(entityClass.options)
            options.update(Device.options)
            errorCount = 0
            for option in section:
                try:
                    if option not in options:
                        raise Exception("\"" + option + "\" is not a valid option for section \"" + sectionName + "\".")

                    configValue = section[option]
                    optionsValue = options[option]

                    # Check if value type is correct
                    if isinstance(optionsValue, bool) and configValue not in validBooleanValues:
                        raise Exception("Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + sectionName +"\" is not valid (can only be 1|yes|true|on|0|no|false|off)")

                    try:
                        type(optionsValue)(configValue)
                    except:
                        raise Exception("Value \"" + configValue + "\" for option \"" + option + "\" in section \"" + sectionName +"\" is not valid (must be of type " + str(type(optionsValue).__name__) + ").")
                except Exception as e:
                    flag = True
                    errorCount += 1
                    self.agentPrint(str(e), type="e")

            if errorCount > 0:
                self.agentPrint(str(errorCount) + " error(s) in \"" + sectionName + "\" section. The device will not be monitored. Please resolve the errors in order for this device to be monitored.", type="w")
            else:
                try:
                    device = entityClass(section, connectionPort, self.name)
                except Exception as e:
                    flag = True
                    self.agentPrint("\"" + deviceType + "\" device module does not properly implement the Device interface", type="e")
                    self.agentPrint("\"" + sectionName + "\" device will not be monitored.", type="w")
                    continue

                if device.activeAttr == 0:
                    flag = True
                    self.agentPrint("Device \"" + sectionName + "\" has 0 enabled attributes to be monitored", type="e")
                    self.agentPrint("\"" + sectionName + "\" device will not be monitored.", type="w")
                    continue

                validSections[sectionName] = device

        if flag:
            self.agentPrint("For more information use --more.", type="i")
            if self.killSwitch:
                self.agentPrint("Killswitch is enabled. Exiting..", type="f")
                exit(6)
        else:
            if self.verbose:
                self.agentPrint("No errors in \"" + self.devicesFilename + "\"", type="o")

        return validSections

    def __connectDevices(self):
        # Discover through the validated config which devices should be monitored
        for sectionName in self.validSections:
            start = time.time()
            device = self.validSections[sectionName]

            if self.verbose:
                self.agentPrint("Connecting to " + device.id + "..", type="i")

            try:
                device.connect()
                elapsed = time.time() - start

                self.devices.append(device)
                if self.verbose:
                    self.agentPrint("Device " + device.type + " at " + device.port + " connected succesfully! (" + str(round(elapsed*1000)) + "ms)", type="o")
                else:
                    self.agentPrint("Device " + device.type + " at " + device.port + " connected succesfully!", type="o")
            except Exception as e:
                self.agentPrint(device.type + " at " + device.port + " cannot be connected. (" + str(e) + ")", type="e")
                self.agentPrint("Device " + device.type + " at " + device.port + " will not be monitored.", type="w")
                if self.killSwitch:
                    self.agentPrint("Killswitch is enabled. Exiting..", type="f")
                    exit(7)

    def __disconnectDevices(self):
        for device in self.devices:
            device.disconnect()

            if self.verbose:
                self.agentPrint("Disconnected device " + device.id, type="o")

    def __fetchFrom(self, device):
        while not self.stopped:
            try:
                start = time.time()
                device.fetch()
                elapsed = time.time() - start
                if self.verbose:
                    self.agentPrint("Fetched from " + device.id + " in " + str(round(elapsed*1000)) + " ms", type="o")

                time.sleep(device.timeout / 1000)
            except Exception as e:
                self.agentPrint("Couldn't fetch from " + device.id + " (" + str(e) + ")", type="e")
                time.sleep(device.timeout / 1000)

    def startRoutine(self):
        self.agentPrint("Connecting to devices listed in \"" + self.devicesFilename + "\"..", type="i")
        self.__connectDevices()

        if len(self.devices) == 0:
            self.agentPrint("No devices connected to the agent. Exiting..", type="f")
            sys.exit(8)

        self.agentPrint("Starting prometheus server at port " + str(self.prometheusPort) + "..", type="i")
        start_http_server(self.prometheusPort)

        try:
            threads = {}
            for device in self.devices:
                threads[device] = Thread(target = self.__fetchFrom, args=(device,))

            for device in threads:
                threads[device].start()
                if self.verbose:
                    self.agentPrint("Started monitoring for device " + device.id + " with " + str(device.activeAttr) + " active attributes (fetching timeout: " + str(device.timeout) + "ms)", type="o")

            if not self.verbose:
                self.agentPrint("Monitoring..", type="i")

            while 1:
                pass
        except KeyboardInterrupt:
            self.stopped = True
            self.agentPrint("Waiting for active fetching to complete..", type="i")
            for device in threads:
                threads[device].join()

            self.agentPrint("Disconnecting devices..", type="i")
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
    parser = argparse.ArgumentParser(description="Agent Settings")
    parser.add_argument("-d", "--devices", default="devices.conf", help="specify discovery/configuration file absolute path (default: \".\\devices.conf\")")
    parser.add_argument("-n", "--name", default="Agent0", help="specify symbolic agent/station name used for seperation/grouping of stations (default: \"Agent0\")")
    parser.add_argument("-p", "--promport", type=isPort, default=8000, help="specify port number for the Prometheus endpoint (default: 8000)")
    parser.add_argument("-k", "--killswitch", action="store_true", help="exit agent if error occurs in validation/connection phase")
    parser.add_argument("-v", "--verbose", action="store_true", help="print actions with details in standard output")
    parser.add_argument("-c", "--color", action="store_true", help="print color rich messages to terminal (terminal needs to support ANSI escape colors)")
    parser.add_argument("-m", "--more", action="store_true", help="open README.md with configuration and implementation details and exit")
    args = parser.parse_args()

    if args.more:
        webbrowser.open("..\README.md")
        exit(0)

    Agent(args.devices, args.name, args.promport, args.killswitch, args.verbose, args.color).startRoutine()

if __name__ == "__main__":
    main()
