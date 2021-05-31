'''
A small manageable testing utility for the monitoring agent (agent.py) is the `test.py` script
which includes a number of functions respective to different functional (f) and performance (p) tests.
Each functional test represents a function of the monitoring agent (agent.py).
Each test returns true in successful completion and false otherwise.
Performance tests produce results/statistics to standard output that can be further analyzed.
The naming convention for better organization and use of the test functions
is as follows: typeOfTest_moduleName_description
e.g.    For a performance test regarding the Jevois module => p_Jevois_DescriptionOfTest()
        For a functional test regarding the Dobot module => f_Dobot_DescriptionOfTest()
'''
import sys, os
import threading
import time
import serial
import configparser
import runtime.DobotDllTypeX as dTypeX

### Performance Tests ###
'''
Description: Helper function to measureFetchingOverheadDobot()
Execute a dobot function and return the execution time in ms
Parameters:
    func: (dTypeX) api call to be measured
    arg: dobot api object
'''
def _execute(func, arg):
    start = time.time()
    func(arg)
    stop = time.time()
    return (stop - start) * 1000

'''
Description: Measure fetching times (in ms) for each attribute through multiple iterations
Parameters:
    port: dobot device port (e.g. "COM4" / "192.168.43.4")
    n: number of iterations
'''
def p_Dobot_FetchingOverhead(port, n):
    dobot, state = dTypeX.ConnectDobotX(port)
    if state[0] != dTypeX.DobotConnect.DobotConnect_NoError:
        print("Couldn't connect Dobot at port " + str(port))
        return False

    if n < 1:
        print("Iterations number must be a positive integer.")
        return False

    iterations = n
    attributes = {"GetDeviceSN":[],"GetDeviceName":[],"GetDeviceVersion":[],"GetWIFISSID":[],"GetWIFIPassword":[],"GetWIFIIPAddress":[],
    "GetWIFINetmask":[],"GetWIFIGateway":[],"GetWIFIDNS":[],"GetDeviceTime":[],"GetQueuedCmdCurrentIndex":[],
    "GetPose":[],"GetAlarmsStateX":[],"GetHOMEParams":[],"GetEndEffectorParams":[],
    "GetEndEffectorLaser":[],"GetEndEffectorSuctionCup":[],"GetEndEffectorGripper":[],"GetJOGJointParams":[],
    "GetJOGCoordinateParams":[],"GetJOGCommonParams":[],"GetPTPJointParams":[],"GetPTPCoordinateParams":[],
    "GetPTPCommonParams":[],"GetPTPJumpParams":[],"GetCPParams":[],"GetARCParams":[],"GetAngleSensorStaticError":[],
    "GetAngleSensorCoef":[],"GetDeviceWithL":[],"GetPoseL":[],"GetJOGLParams":[],"GetPTPLParams":[],
    "GetWIFIConfigMode":[],"GetWIFIConnectStatus":[]}

    for i in range(iterations):
        attributes["GetDeviceSN"].append(_execute(dTypeX.GetDeviceSN, dobot))
        attributes["GetDeviceName"].append(_execute(dTypeX.GetDeviceName, dobot))
        attributes["GetDeviceVersion"].append(_execute(dTypeX.GetDeviceVersion, dobot))
        attributes["GetWIFISSID"].append(_execute(dTypeX.GetWIFISSID, dobot))
        attributes["GetWIFIPassword"].append(_execute(dTypeX.GetWIFIPassword, dobot))
        attributes["GetWIFIIPAddress"].append(_execute(dTypeX.GetWIFIIPAddress, dobot))
        attributes["GetWIFINetmask"].append(_execute(dTypeX.GetWIFINetmask, dobot))
        attributes["GetWIFIGateway"].append(_execute(dTypeX.GetWIFIGateway, dobot))
        attributes["GetWIFIDNS"].append(_execute(dTypeX.GetWIFIDNS, dobot))
        attributes["GetDeviceTime"].append(_execute(dTypeX.GetDeviceTime, dobot))
        attributes["GetQueuedCmdCurrentIndex"].append(_execute(dTypeX.GetQueuedCmdCurrentIndex, dobot))
        attributes["GetPose"].append(_execute(dTypeX.GetPose, dobot))
        attributes["GetAlarmsStateX"].append(_execute(dTypeX.GetAlarmsStateX, dobot))
        attributes["GetHOMEParams"].append(_execute(dTypeX.GetHOMEParams, dobot))
        attributes["GetEndEffectorParams"].append(_execute(dTypeX.GetEndEffectorParams, dobot))
        attributes["GetEndEffectorLaser"].append(_execute(dTypeX.GetEndEffectorLaser, dobot))
        attributes["GetEndEffectorSuctionCup"].append(_execute(dTypeX.GetEndEffectorSuctionCup, dobot))
        attributes["GetEndEffectorGripper"].append(_execute(dTypeX.GetEndEffectorGripper, dobot))
        attributes["GetJOGJointParams"].append(_execute(dTypeX.GetJOGJointParams, dobot))
        attributes["GetJOGCoordinateParams"].append(_execute(dTypeX.GetJOGCoordinateParams, dobot))
        attributes["GetJOGCommonParams"].append(_execute(dTypeX.GetJOGCommonParams, dobot))
        attributes["GetPTPJointParams"].append(_execute(dTypeX.GetPTPJointParams, dobot))
        attributes["GetPTPCoordinateParams"].append(_execute(dTypeX.GetPTPCoordinateParams, dobot))
        attributes["GetPTPCommonParams"].append(_execute(dTypeX.GetPTPCommonParams, dobot))
        attributes["GetPTPJumpParams"].append(_execute(dTypeX.GetPTPJumpParams, dobot))
        attributes["GetCPParams"].append(_execute(dTypeX.GetCPParams, dobot))
        attributes["GetARCParams"].append(_execute(dTypeX.GetARCParams, dobot))
        attributes["GetAngleSensorStaticError"].append(_execute(dTypeX.GetAngleSensorStaticError, dobot))
        attributes["GetAngleSensorCoef"].append(_execute(dTypeX.GetAngleSensorCoef, dobot))
        attributes["GetDeviceWithL"].append(_execute(dTypeX.GetDeviceWithL, dobot))
        attributes["GetPoseL"].append(_execute(dTypeX.GetPoseL, dobot))
        attributes["GetJOGLParams"].append(_execute(dTypeX.GetJOGLParams, dobot))
        attributes["GetPTPLParams"].append(_execute(dTypeX.GetPTPLParams, dobot))
        attributes["GetWIFIConfigMode"].append(_execute(dTypeX.GetWIFIConfigMode, dobot))
        attributes["GetWIFIConnectStatus"].append(_execute(dTypeX.GetWIFIConnectStatus, dobot))

    print("\nResults for " + str(iterations) + " iterations (attribute_name, avg, min, max (in milliseconds))")
    for attr in attributes:
        max = 0
        min = sys.maxsize
        sum = 0
        for i in range(iterations):
            ms = attributes[attr][i]
            sum += ms
            if ms > max:
                max = ms
            if ms < min:
                min = ms

        print(str(attr) + "," + str(round(sum/iterations)) + "," + str(round(min)) + "," + str(round(max)))

    return True

'''
Description: Measure the fetching rate in which one can receive standardized messages through the serial port
Parameters:
    port: jevois serial port
    n: number of iterations
'''
def p_Jevois_FetchingRate(port, n):
    try:
        ser = serial.Serial(port, 115200, timeout=0)
        line = ser.readline().rstrip()
        tok = line.split()
        print("Jevois Camera at port " + port + " connected succesfully!")
    except Exception as e:
        print("Couldn't connect with Jevois Camera device at port " + port + " (" + str(e) + ")")
        return False

    if n < 1:
        print("Iterations number must be a positive integer.")
        return False

    iterations = n

    max = 0
    min = sys.maxsize
    sum = 0
    i = 0
    while i < iterations:
        start = time.time()
        line = ser.readline().rstrip().decode()
        stop = time.time()
        if len(line) > 0:
            print(line)
            ms = (stop - start) * 1000
            sum += ms
            if ms > max:
                max = ms
            if ms < min:
                min = ms
            i += 1

    print("\nResults for " + str(iterations) + " iterations (attribute_name, avg, min, max (in milliseconds))")
    print(str(round(sum/iterations)) + "," + str(round(min)) + "," + str(round(max)))
    return True

'''
Description: Measure the switching overhead of connecting to multiple dobot devices
through a 2 dobot connection paradigm using the default dTypeX.ConnectDobot() call
Parameters:
    port1: dobot device port (e.g. "COM4" / "192.168.43.4")
    port2: dobot device port (e.g. "COM4" / "192.168.43.4")
'''
def p_Dobot_SwitchOverhead(port1, port2):
    # Run for 1 minute
    stop = time.time() + 60
    sum = 0
    count = 0
    api = dTypeX.load()
    while (time.time() < stop):
        state = dTypeX.ConnectDobot(api, port1, 115200)[0]
        if state != dTypeX.DobotConnect.DobotConnect_NoError:
            print("Can't connect to the dobot. Aborting test.")
            return False
        start = time.time()
        dTypeX.DisconnectDobot(api)
        state = dTypeX.ConnectDobot(api, port2, 115200)[0]
        if state != dTypeX.DobotConnect.DobotConnect_NoError:
            print("Can't connect to the dobot. Aborting test.")
            return False
        end = time.time()
        sum += end - start
        count += 1
        print(f"Switching overhead is {end - start} \n")

    print("Recorded %3d switches\n"% (count))
    print("Total overhead: %5.2f seconds\n"% (sum))
    print("Average Switch Overhead: %5.2f seconds\n"% (sum / count))
    print("Percentage of time spend on overhead %5.2f%%\n"% (sum * 100 / 60))
    return True

### Functional Tests ###
'''
Description: Test connecting to a JeVois camera device
Parameters:
    port: jevois device port (e.g. "COM3")
'''
def f_Jevois_Connectivity(port):
    try:
        ser = serial.Serial(port, 115200, timeout=0.25)
        print("Jevois Camera at port " + port + " connected succesfully!")
    except Exception as e:
        print("Couldn't connect with Jevois Camera device at port " + port + " (" + str(e) + ")")
        return False

    stop = time.time() + 30
    while (time.time() < stop):
        line = ser.readline().rstrip().decode()
        tok = line.split()
        # Abort fetching if timeout or malformed line
        if len(tok) < 1: print("No data found"); continue
        if tok[0][0] != 'N': print("Unsupported serstyle"); continue
        if tok[0][1] == '1' and len(tok) != 4: continue
        elif tok[0][1] == '2' and len(tok) != 6: continue
        elif tok[0][1] == '3' and len(tok) != 8: continue
        print(line)

    return True

'''
Description: Test the non-standard dTypeX.GetAlarmsStateX() function to fetch alarms
created as an alternative to the standard dTypeX.GetAlarmsState()
Parameters:
    port: dobot device port (e.g. "COM4" / "192.168.43.4")
'''
def f_Dobot_Alarms(port):
    dobot, state = dTypeX.ConnectDobotX(port)

    if state[0] != dTypeX.DobotConnect.DobotConnect_NoError:
        return False

    print(port + "\'s name: " + str(dTypeX.GetDeviceName(dobot)[0]))

    stop = time.time() + 60
    while (time.time() < stop):
        print(time.time + "Active alarms:")
        for a in dTypeX.GetAlarmsStateX(dobot):
            print(a)
        time.sleep(0.2)

    dTypeX.DisconnectAll()
    return True

'''
Description: Test the non-standard dTypeX.ConnectDobotX() function to connect to multiple
dobot devices in parallel (and diminish switching overhead), created as
an alternative to the standard dTypeX.ConnectDobot()
Parameters:
    portList: a list of strings indicating the multiple ports of dobot devices
    (e.g. ["192.168.43.4","192.168.43.5"])
'''
def f_Dobot_ParallelConnection(portList):
    dobotList = []
    for port in portList:
        dobot, state = dTypeX.ConnectDobotX(port)

        if state[0] == dTypeX.DobotConnect.DobotConnect_NoError:
            dobotList.append(dobot)
        else:
            return False

    print("\nConnected Dobots:")
    for dobot in dobotList:
        print("Device Name: " + str(dTypeX.GetDeviceName(dobot)[0]))
        print("Device Serial No: " + str(dTypeX.GetDeviceSN(dobot)[0]))
        print("Device Version: " + ".".join(list(map(str, dTypeX.GetDeviceVersion(dobot)))))
        print()

    dTypeX.DisconnectAll()
    return True

### Enable/Disable Tests ###
#p_Dobot_FetchingOverhead("192.168.43.4", 30)
#p_Jevois_FetchingRate("COM4", 30)
#p_Dobot_SwitchOverhead("192.168.43.4","192.168.43.5")
#f_Jevois_Connectivity("COM4")
#f_Dobot_Alarms("192.168.43.4")
#f_Dobot_ParallelConnection(["192.168.43.4","192.168.43.5"])
