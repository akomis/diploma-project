import sys, os
import threading
import time
import serial
import configparser
import DobotDllTypeX as dType

def measureSwitchOverhead():
    # Run for 1 minute
    stop = time.time() + 60
    sum = 0
    count = 0
    api = dType.load()
    while (time.time() < stop):
        state = dType.ConnectDobot(api, "192.168.43.4", 115200)[0]
        if state != dType.DobotConnect.DobotConnect_NoError:
            print("Can't connect to the dobot. Aborting test.")
            break
        start = time.time()
        dType.DisconnectDobot(api)
        state = dType.ConnectDobot(api, "192.168.43.5", 115200)[0]
        if state != dType.DobotConnect.DobotConnect_NoError:
            print("Can't connect to the dobot. Aborting test.")
            break
        end = time.time()
        sum += end - start
        count += 1
        print(f"Switching overhead is {end - start}\n")

    print("Recorded %3d switches\n"% (count))
    print("Total overhead: %5.2f seconds\n"% (sum))
    print("Average Switch Overhead: %5.2f seconds\n"% (sum / count))
    print("Percentage of time spend on overhead %5.2f%%\n"% (sum * 100 / 60))

def testJevoisConnectivity(port):
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        line = ser.readline().rstrip()
        tok = line.split()
        # Raise exception if timeout or malformed line or not supported serstyle
        #if len(tok) < 1: raise Exception('The line is contains no standardized message.')
        print("Jevois Camera at port " + port + " connected succesfully!")
    except Exception as e:
        print("Couldn't connect with Jevois Camera device at port " + port + " (" + str(e) + ")")
        return

    while (1):
        line = ser.readline().rstrip()
        tok = line.split()
        # Abort fetching if timeout or malformed line
        if len(tok) < 1: print("No data found"); continue
        if tok[0][0] != 'N': print("Unsupported serstyle"); continue
        if tok[0][1] == '1' and len(tok) != 4: continue
        elif tok[0][1] == '2' and len(tok) != 6: continue
        elif tok[0][1] == '3' and len(tok) != 8: continue
        print(line)

def testAlarms(port):
    dobot0, state1 = dType.ConnectDobotX(port)

    if state1[0] == dType.DobotConnect.DobotConnect_NoError:
        print(port + "\'s name: " + str(dType.GetDeviceName(dobot0)[0]))

    stop = time.time() + 60
    while (time.time() < stop):
        print(time.time + "Active alarms:")
        for a in dType.GetAlarmsStateX(dobot0):
            print(a)
        time.sleep(0.2)

    dType.DisconnectAll()

def testParallelDobotConnection(portList):
    dobotList = []
    for port in portList:
        dobot, state = dType.ConnectDobotX(port)

        if state[0] == dType.DobotConnect.DobotConnect_NoError:
            dobotList.append(dobot)

    print("\nConnected Dobots:")
    for dobot in dobotList:
        print("Device Name: " + str(dType.GetDeviceName(dobot)[0]))
        print("Device Serial No: " + str(dType.GetDeviceSN(dobot)[0]))
        print("Device Version: " + '.'.join(list(map(str, dType.GetDeviceVersion(dobot)))))
        print()

    dType.DisconnectAll()


#measureSwitchOverhead()
#testJevoisConnectivity("COM12")
#testAlarms("192.168.43.4")
#testParallelDobotConnection(["192.168.43.4","192.168.43.5"])


config = configparser.ConfigParser()
config.read('agent.conf')

validOptions = {"AGENT":["agentname","routineinterval","prometheusport"],
                "DOBOT":["devicesn","devicename","deviceversion","devicetime","queueindex",
                "posex","posey","posez","poser","anglebase","anglereararm","angleforearm",
                "angleendeffector","alarmsstate","homex","homey","homez","homer","autolevelingresult",
                "endeffectorx","endeffectory","endeffectorz","laserstatus","suctioncupstatus","gripperstatus","jogbasevelocity",
                "jogreararmvelocity","jogforearmvelocity","jogendeffectorvelocity","jogbaseacceleration","jogreararmacceleration",
                "jogforearmacceleration","jogendeffectoracceleration","jogaxisxvelocity","jogaxisyvelocity","jogaxiszvelocity","jogaxisrvelocity","jogaxisxacceleration",
                "jogaxisyacceleration","jogaxiszacceleration","jogaxisracceleration","jogvelocityratio","jogaccelerationratio","ptpbasevelocity","ptpreararmvelocity",
                "ptpforearmvelocity","ptpendeffectorvelocity","ptpbaseacceleration","ptpreararmacceleration","ptpforearmacceleration","ptpendeffectoracceleration","ptpaxisxyzvelocity",
                "ptpaxisrvelocity","ptpaxisxyzacceleration","ptpaxisracceleration","ptpvelocityratio","ptpaccelerationratio","liftingheight","heightlimit",
                "cpvelocity","cpacceleration","arcxyzvelocity","arcrvelocity","arcxyzacceleration","arcracceleration","anglestaticerrrear",
                "anglestaticerrfront","anglecoefrear","anglecoeffront","slidingrailstatus","slidingrailpose","slidingrailjogvelocity","slidingrailjogacceleration",
                "slidingrailptpvelocity","slidingrailptpacceleration","wifimodulestatus","wificonnectionstatus","wifissid","wifipassword","wifiipaddress",
                "wifinetmask","wifigateway","wifidns"],
                "JEVOIS":["objects","objectidentified","objectlocation","objectsize"]}

ignoreValueCheck = ["agentname","routineinterval","prometheusport","objects"]

validValues = ["1","yes","true","on","0","no","false","off"]

for section in config.sections():
    sectionType = section.split(':')[0]

    if sectionType not in validOptions.keys():
        print("[WARNING] \"" + section + "\" cannot be recognised for validation.")
        continue

    for option in config[section]:
        if option not in validOptions[sectionType]:
            print("[WARNING] \"" + option + "\" is not a valid option for section \"" + section + "\" and will be ignored.")

        if option not in ignoreValueCheck and config[section][option] not in validValues:
            print("[WARNING] Value \"" + config[section][option] + "\" for option \"" + option + "\" in section \"" + section +"\" is not valid and the option will be set to default")
