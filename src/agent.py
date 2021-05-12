import sys
import time
import webbrowser
import configparser
from threading import Thread
from prometheus_client import start_http_server
from device_modules import *

class Agent():
    options = {"agentname":"Agent0","prometheusport":8000,"verbose":False}

    def __init__(self, ks):
        self.killSwitch = ks
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

        # Check Agent section's validity first and seperately from device sections
        try:
            if "Agent" not in self.config.sections():
                raise Exception("Missing mandatory \"Agent\" section in \"agent.conf\"")

            for option in self.config["Agent"]:
                if option not in Agent.options:
                    raise Exception("Agent does not support the \"" + option + "\" option")

                configValue = self.config["Agent"][option]
                optionsValue = Agent.options[option]

                if isinstance(optionsValue, bool) and configValue not in validBooleanValues:
                    raise Exception("Value \"" + configValue + "\" for option \"" + option + "\" in section \"Agent\" is not valid (must be one of the following: 1,yes,true,on,0,no,false,off)")

                try:
                    type(optionsValue)(configValue)
                except:
                    raise Exception("Value \"" + configValue + "\" for option \"" + option + "\" in section \"Agent\" is not valid (must be of type " + str(type(optionsValue).__name__) + ").")

                type(optionsValue)(configValue)
        except Exception as e:
            print("[ERROR] " + str(e), file=sys.stderr)
            print("Exiting..")
            exit(4)

        # Check configuration validity for the monitored devices
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

            if not self.verbose:
                print("[" + self.agentName + "] Monitoring.. ")

            while 1:
                pass
        except KeyboardInterrupt:
            print("Disconnecting devices..")
            self.__disconnectDevices()
            exit(0)


def agentInit(args):
    try:
        if len(args) > 2:
            raise Exception("Too many arguments")
        elif len(args) == 2:
            if args[1] == "-h" or args[1] == "--help":
                webbrowser.open("../README.md")
                exit(1)
            elif args[1] == "-k" or args[1] == "--killswitch":
                return Agent(True)
            else:
                raise Exception("Unrecognised option \"" + sys.argv[1] + "\"")
    except Exception as e:
        print(str(e))
        print("For more information: $ agent.py --help")
        exit(2)

    return Agent(False)

def main():
    agent = agentInit(sys.argv)
    agent.startRoutine()

if __name__ == "__main__":
    main()
