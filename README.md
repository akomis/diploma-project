<div align="center">
<h1>Diploma Project</h1>
<p>Monitoring System for Industrial IoT Applications</p>
<img alt="Dobot Magician with Belt" src="/pics/dobot-magician-belt.png">
</div>

## Overview
A monitoring system that collects data from devices such as the [Dobot Magician](https://www.dobot.cc/dobot-magician/product-overview.html), [Sliding Rail](https://www.dobot.cc/products/sliding-rail-kit-overview.html) and [JeVois Camera](http://www.jevois.org/) that are used in an industrial setting and monitors them. Using [Prometheus](https://prometheus.io/) for monitoring device metrics, which is natively compatible with [Grafana](https://grafana.com/) to visualize them and provide insight.  
The goal of this system is to be a cost-effective, efficient, modular and extensible solution at monitoring such devices. This project aims at doing that while being efficient, without interfering with the normal operations of the devices, and be highly scalable in terms of the number of devices that can be monitored as well as the number of different types of devices it supports. Also provide high monitoring flexibility through a plethora of configuration options for the monitoring agent and what device attributes to be extracted and monitored, configured individually for each device.
<br><br>

## Dependencies
- [Python 3.9+](https://www.python.org/downloads/windows/)
- [Dobot Robot Driver](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=70#sub-download)
- [Prometheus](https://prometheus.io/download/)
- [Grafana](https://grafana.com/get)
<br><br>

## Installation
`git clone https://github.com/akomis/diploma-project.git`
<br><br>

## Configuration
Change agent's settings and choose which devices and which data/attributes of those will be monitored by changing the `agent.conf` file.  
For changing the agent's settings you can change the values under the `[Agent]` section.  
In order for the agent to find a Dobot Magician and connect to it, a section of the device, `[Dobot:PORT]` must exist in the configuration file e.g. `[Dobot:COM7]` for serial or `[Dobot:192.168.0.3]` for connecting through WiFi. You can connect multiple devices through various ports (serial port/IP address).  
Similarly in order for the agent to find a JeVois camera and connect to it, a section of the device `[Jevois:PORT]` must exist in the configuration file (e.g. `[Jevois:COM3]`) with the only difference that the port can only be serial as the camera does not support wireless connection with the host. For monitoring the object's identity one must provide a space-separated list with object names in the "objects" entry (e.g. objects = cube pen paper).  
For enabling data to be monitored you can use `on`, `1`, `yes` or `true` and in order to not monitor certain data use `off`, `0`, `no`, `false` depending on your preference. By removing an entry completely the value for the entry will be resolved to the default. All keys are case-insensitive but all section names must be in the same format as the class name representing the device module.  
For custom device modules/classes in the device_modules.py e.g. `DeviceType` a `[DeviceType:<port>]` entry must exist in the configuration for the monitoring agent to automatically discover it and use the appropriate module for connecting, fetching and disconnecting (see more in "Extensibility" section).  
Each device entry supports the `Timeout` attribute which sets the timeout period in milliseconds inbetween fetches and defaults to 100.  
All configuration is parsed and validated based on the above information, before the start of the routine, and warns the user for any invalid entries, fields and values.
For more details on the configuration settings for the Agent, Dobot Magician and JeVois camera devices check their respective tables below with all options and their details.  

### Agent
|   Config Name   |                        Description                        | Type | Default |
|:---------------:|:---------------------------------------------------------:|:----:|:-------:|
|    AgentName    |                  Symbolic name for agent                  |  str |  Agent0 |
|  PrometheusPort |                Port for Prometheus endpoint               |  int |   8000  |

### Dobot
|         Config Name        |                                          Description                                          | Prometheus Type |     Default     |            API Call            |
|:--------------------------:|:---------------------------------------------------------------------------------------------:|:---------------:|:---------------:|:------------------------------:|
|          DeviceSN          |                                     Device's serial number                                    |    info (str)   |        on       |        GetDeviceSN(api)        |
|         DeviceName         |                                      Device's name/alias                                      |    info (str)   |        on       |       GetDeviceName(api)       |
|        DeviceVersion       |                            Device's verion (major.minor.0.revision)                           |    info (str)   |        on       |      GetDeviceVersion(api)     |
|         DeviceTime         |                                      Device's clock/time                                      |    info (str)   |       off       |       GetDeviceTime(api)       |
|         QueueIndex         |                                 Current index in command queue                                |   gauge (int)   |       off       |  GetQueuedCmdCurrentIndex(api) |
|            PoseX           |                       Real-time cartesian coordinate of device's X axis                       |  gauge (float)  |        on       |          GetPose(api)          |
|            PoseY           |                       Real-time cartesian coordinate of device's Y axis                       |  gauge (float)  |        on       |          GetPose(api)          |
|            PoseZ           |                       Real-time cartesian coordinate of device's Z axis                       |  gauge (float)  |        on       |          GetPose(api)          |
|            PoseR           |                       Real-time cartesian coordinate of device's R axis                       |  gauge (float)  |        on       |          GetPose(api)          |
|          AngleBase         |                                        Base joint angle                                       |  gauge (float)  |        on       |          GetPose(api)          |
|        AngleRearArm        |                                      Rear arm joint angle                                     |  gauge (float)  |        on       |          GetPose(api)          |
|        AngleForearm        |                                      Forearm joint angle                                      |  gauge (float)  |        on       |          GetPose(api)          |
|      AngleEndEffector      |                                    End effector joint angle                                   |  gauge (float)  |        on       |          GetPose(api)          |
|         AlarmsState        |                                     Device's active alarms                                    |  enum (alarms)  |        on       |       GetAlarmsStateX(api)     |
|            HomeX           |                                    Home position for X axis                                   |  gauge (float)  |       off       |       GetHOMEParams(api)       |
|            HomeY           |                                    Home position for Y axis                                   |  gauge (float)  |       off       |       GetHOMEParams(api)       |
|            HomeZ           |                                    Home position for Z axis                                   |  gauge (float)  |       off       |       GetHOMEParams(api)       |
|            HomeR           |                                    Home position for R axis                                   |  gauge (float)  |       off       |       GetHOMEParams(api)       |
|     AutoLevelingResult     |                              Automatic leveling precision result                              |  gauge (float)  |       off       |   GetAutoLevelingResult(api)   |
|        EndEffectorX        |                                 X-axis offset of end effector                                 |  gauge (float)  |       off       |    GetEndEffectorParams(api)   |
|        EndEffectorY        |                                 Y-axis offset of end effector                                 |  gauge (float)  |       off       |    GetEndEffectorParams(api)   |
|        EndEffectorZ        |                                 Z-axis offset of end effector                                 |  gauge (float)  |       off       |    GetEndEffectorParams(api)   |
|         LaserStatus        |                               Status (enabled/disabled) of laser                              |   enum (bool)   |       off       |    GetEndEffectorLaser(api)    |
|      SuctionCupStatus      |                            Status (enabled/disabled) of suction cup                           |   enum (bool)   |       off       |  GetEndEffectorSuctionCup(api) |
|        GripperStatus       |                              Status (enabled/disabled) of gripper                             |   enum (bool)   |       off       |   GetEndEffectorGripper(api)   |
|       JogBaseVelocity      |                          Velocity (°/s) of base joint in jogging mode                         |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|     JogRearArmVelocity     |                        Velocity (°/s) of rear arm joint in jogging mode                       |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|     JogForearmVelocity     |                        Velocity (°/s) of forearm joint in jogging mode                        |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|   JogEndEffectorVelocity   |                      Velocity (°/s) of end effector joint in jogging mode                     |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|     JogBaseAcceleration    |                       Acceleration (°/s^2) of base joint in jogging mode                      |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|   JogRearArmAcceleration   |                     Acceleration (°/s^2) of rear arm joint in jogging mode                    |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|   JogForearmAcceleration   |                     Acceleration (°/s^2) of forearm joint in jogging mode                     |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
| JogEndEffectorAcceleration |                   Acceleration (°/s^2) of end effector joint in jogging mode                  |  gauge (float)  |       off       |     GetJOGJointParams(api)     |
|      JogAxisXVelocity      |           Velocity (mm/s) of device's X axis (cartesian coordinate) in jogging mode           |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|      JogAxisYVelocity      |           Velocity (mm/s) of device's Y axis (cartesian coordinate) in jogging mode           |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|      JogAxisZVelocity      |           Velocity (mm/s) of device's Z axis (cartesian coordinate) in jogging mode           |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|      JogAxisRVelocity      |           Velocity (mm/s) of device's R axis (cartesian coordinate) in jogging mode           |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|    JogAxisXAcceleration    |        Acceleration (mm/s^2) of device's X axis (cartesian coordinate) in jogging mode        |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|    JogAxisYAcceleration    |        Acceleration (mm/s^2) of device's Y axis (cartesian coordinate) in jogging mode        |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|    JogAxisZAcceleration    |        Acceleration (mm/s^2) of device's Z axis (cartesian coordinate) in jogging mode        |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|    JogAxisRAcceleration    |        Acceleration (mm/s^2) of device's R axis (cartesian coordinate) in jogging mode        |  gauge (float)  |       off       |   GetJOGCoordinateParams(api)  |
|      JogVelocityRatio      |       Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode      |  gauge (float)  |       off       |     GetJOGCommonParams(api)    |
|    JogAccelerationRatio    |     Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode    |  gauge (float)  |       off       |     GetJOGCommonParams(api)    |
|       PtpBaseVelocity      |                      Velocity (°/s) of base joint in point to point mode                      |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|     PtpRearArmVelocity     |                    Velocity (°/s) of rear arm joint in point to point mode                    |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|     PtpForearmVelocity     |                     Velocity (°/s) of forearm joint in point to point mode                    |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|   PtpEndEffectorVelocity   |                  Velocity (°/s) of end effector joint in point to point mode                  |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|     PtpBaseAcceleration    |                   Acceleration (°/s^2) of base joint in point to point mode                   |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|   PtpRearArmAcceleration   |                 Acceleration (°/s^2) of rear arm joint in point to point mode                 |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|   PtpForearmAcceleration   |                  Acceleration (°/s^2) of forearm joint in point to point mode                 |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
| PtpEndEffectorAcceleration |               Acceleration (°/s^2) of end effector joint in point to point mode               |  gauge (float)  |       off       |     GetPTPJointParams(api)     |
|     PtpAxisXYZVelocity     |     Velocity (mm/s) of device's X, Y, Z axis (cartesian coordinate) in point to point mode    |  gauge (float)  |       off       |   GetPTPCoordinateParams(api)  |
|      PtpAxisRVelocity      |        Velocity (mm/s) of device's R axis (cartesian coordinate) in point to point mode       |  gauge (float)  |       off       |   GetPTPCoordinateParams(api)  |
|   PtpAxisXYZAcceleration   |  Acceleration (mm/s^2) of device's X, Y, Z axis (cartesian coordinate) in point to point mode |  gauge (float)  |       off       |   GetPTPCoordinateParams(api)  |
|    PtpAxisRAcceleration    |     Acceleration (mm/s^2) of device's R axis (cartesian coordinate) in point to point mode    |  gauge (float)  |       off       |   GetPTPCoordinateParams(api)  |
|      PtpVelocityRatio      |   Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode   |  gauge (float)  |       off       |     GetPTPCommonParams(api)    |
|    PtpAccelerationRatio    | Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode |  gauge (float)  |       off       |     GetPTPCommonParams(api)    |
|        LiftingHeight       |                                  Lifting height in jump mode                                  |  gauge (float)  |       off       |      GetPTPJumpParams(api)     |
|         HeighLimit         |                                Max lifting height in jump mode                                |  gauge (float)  |       off       |      GetPTPJumpParams(api)     |
|         CpVelocity         |                                   Velocity (mm/s) in cp mode                                  |  gauge (float)  |       off       |        GetCPParams(api)        |
|       CpAcceleration       |                                Acceleration (mm/s^2) in cp mode                               |  gauge (float)  |       off       |        GetCPParams(api)        |
|       ArcXYZVelocity       |                          Velocity (mm/s) of X, Y, Z axis in arc mode                          |  gauge (float)  |       off       |        GetARCParams(api)       |
|        ArcRVelocity        |                             Velocity (mm/s) of R axis in arc mode                             |  gauge (float)  |       off       |        GetARCParams(api)       |
|     ArcXYZAcceleration     |                       Acceleration (mm/s^2) of X, Y, Z axis in arc mode                       |  gauge (float)  |       off       |        GetARCParams(api)       |
|      ArcRAcceleration      |                          Acceleration (mm/s^2) of R axis in arc mode                          |  gauge (float)  |       off       |        GetARCParams(api)       |
|     AngleStaticErrRear     |                               Rear arm angle sensor static error                              |  gauge (float)  |       off       | GetAngleSensorStaticError(api) |
|     AngleStaticErrFront    |                               Forearm angle sensor static error                               |  gauge (float)  |       off       | GetAngleSensorStaticError(api) |
|        AngleCoefRear       |                         Rear arm angle sensor linearization parameter                         |  gauge (float)  |       off       |     GetAngleSensorCoef(api)    |
|       AngleCoefFront       |                          Forearm angle sensor linearization parameter                         |  gauge (float)  |       off       |     GetAngleSensorCoef(api)    |
|      SlidingRailStatus     |                            Sliding rail's status (enabled/disabled)                           |   enum (bool)   |       off       |       GetDeviceWithL(api)      |
|       SlidingRailPose      |                              Sliding rail's real-time pose in mm                              |  gauge (float)  |       off       |          GetPoseL(api)         |
|   SlidingRailJogVelocity   |                        Velocity (mm/s) of sliding rail in jogging mode                        |  gauge (float)  |       off       |       GetJOGLParams(api)       |
| SlidingRailJogAcceleration |                     Acceleration (mm/s^2) of sliding rail in jogging mode                     |  gauge (float)  |       off       |       GetJOGLParams(api)       |
|   SlidingRailPtpVelocity   |                     Velocity (mm/s) of sliding rail in point to point mode                    |  gauge (float)  |       off       |       GetPTPLParams(api)       |
| SlidingRailPtpAcceleration |                  Acceleration (mm/s^2) of sliding rail in point to point mode                 |  gauge (float)  |       off       |       GetPTPLParams(api)       |
|      WifiModuleStatus      |                             Wifi module status (enabled/disabled)                             |   enum (bool)   |       off       |     GetWIFIConfigMode(api)     |
|    WifiConnectionStatus    |                        Wifi connection status (connected/not connected)                       |   enum (bool)   |       off       |    GetWIFIConnectStatus(api)   |
|          WifiSSID          |                                      Configured Wifi SSID                                     |    info (str)   |       off       |        GetWIFISSID(api)        |
|        WifiPassword        |                                    Configured Wifi Password                                   |    info (str)   |       off       |      GetWIFIPassword(api)      |
|        WifiIPAddress       |                                      Device's IP address                                      |    info (str)   |       off       |      GetWIFIIPAddress(api)     |
|         WifiNetmask        |                                          Subnet mask                                          |    info (str)   |       off       |       GetWIFINetmask(api)      |
|         WifiGateway        |                                        Default Gateway                                        |    info (str)   |       off       |       GetWIFIGateway(api)      |
|           WifiDNS          |                                              DNS                                              |    info (str)   |       off       |         GetWIFIDNS(api)        |

### Jevois
|    Config Name   |          Description         |  Prometheus Type | Default |
|:----------------:|:----------------------------:|:----------------:|:-------:|
| ObjectIdentified |   Identified object's name   |    enum (str)    |    on   |
|  ObjectLocation  | Identified object's location | gauge(s) (float) |    on   |
|    ObjectSize    |   Identified object's size   |   gauge (float)  |   off   |

For a more practical insight check the default `agent.conf` included.
<br><br>

## Usage
Make sure that `agent.conf` is properly setup and in the same directory as the executable.  
`$ python3 agent.py`
<br><br>

## Scalability
The system can scale (monitoring station level) vertically as the agent can connect to and monitor a variable amount of devices, a number constrained by the monitoring station's available ports and resources. In addition for larger and more complex setups one can scale the system vertically by adding multiple monitoring stations. For the configuration of these stations one can tweak their respective prometheus and agent configurations. For their coordination one can change the grafana (visual layer) settings on the main monitoring station.
<br><br>

## Extensibility
The agent currently supports Dobot Magician and JeVois Camera devices. For extending the agent's capabilities to support a different type of device one can create a device class (device module) and place it in the `device_modules.py`. This class needs to be a child of the Device class (found in the same file) and implement all its attributes and methods. The name of the class is determining the name that the agent will use to discover a device through `agent.conf`, connect to it, fetch (and inform prometheus) its attributes and finally disconnect from the device.  
The attributes that need to be implemented is the configValidOptions[] list and the configIgnoreValueCheck[] list and the methods are the _connect(), _fetch() and _disconnect() methods. More specifically
### `configValidOptions[]`
Includes all the valid fields/options a device can have in the configuration file (monitored attribute fields).
### `configIgnoreValueCheck[]`
Includes the fields that are not considered monitoring options (which enables/disables the attributes to be monitored) and shall skip the enabling/disabling value check. This list must be a subset of the configValidOptions list.
### `_connect()`
Responsible for connecting to the device, initialize the prometheus metrics and other necessary device information that is vital for the use of the other methods. If the connection attempt is unsuccessful this method should return False, otherwise it should return True.
### `_fetch()`
Used to extract all enabled monitoring attributes for said device and update the Prometheus metrics accordingly.
### `_disconnect()`
Responsible for disconnecting the device, close any open ports/streams and remove any runtime temporary files regarding the device.  

All other necessary modules needed for implementing the above functions (e.g. `DobotDllTypeX.py` for the `Dobot` device module) and any runtime files should be included in the `runtime` directory.  
For a practical example one can review the source code in `device_modules.py` and more specifically the `Dobot` and `Jevois` classes (device modules).  
<br><br>

## Testing
A small manageable testing utility for the monitoring agent (agent.py) is the `test.py` script which includes a number of functions respective to different functional and performance tests and run examples in comments.
<br><br>

## DobotDllType.py Fork (DobotDllTypeX.py)
Throughout the analysis of the Dobot API, some minor issues arose with fetching certain useful attributes, either due to typos in the API. Fixing those bugs to not sacrifice any wanted data led to a greater understanding of how the Dobot API works and resulted to more changes that make the Dobot API more flexible and more convenient to use. No functions are changed as to not break any existing implementations utilizing the official API as all changes to functiones are done through wrappers. For using the improved functions provided by the fork one should create a `runtime` directory in the directory of the agent with all the files provided in the Dobot Demo. For importing and utilizing the fork: `import DobotDllTypeX as dType`
### Fixes
* Fixed `GetPoseL(api)` function, which returns the position of the sliding rail (if there is one connected to the robot), by importing the math library which is required for the needs of the function, however not included by default.
### Improvements
* `loadX()` function to replace `load()` that implements loading individual dll/so (DobotDll.dll instances) for each connected device in order to enable parallel connection with multiple dobots. In addition to that a "connections" list is maintained by the API to include all connected devices (their dll/so). This function is not meant to be called explicitly.
* `ConnectDobotX(port)` function to replace `api = load(); state = ConnectDobot(api, port, baudrate)` for connecting to a Dobot Magician device. The main improvement this change provides is that through its implementation, by utilizing the `loadX()` improvement, it allows parallel connections to Dobot Magicians and removes the need to issue it separately. When using the default API this model is not feasible and multiple Dobot Magicians can be connected concurrently with a switching overhead of approximately 0.3 seconds per switch. Apart from the performance benefits this function provides, it is also a more readable and convenient option for connecting a Dobot Magician device as all the standardized procedures are included either in the function or through default arguments. Example of use:  
```python
dobot0, state0 = dType.ConnectDobotX("192.168.43.4")
dobot1, state1 = dType.ConnectDobotX("192.168.43.5")

if state0[0] == dType.DobotConnect.DobotConnect_NoError:
    print("192.168.43.4 name: " + str(dType.GetDeviceName(dobot0)[0]))

if state1[0] == dType.DobotConnect.DobotConnect_NoError:
    print("192.168.43.5 name: " + str(dType.GetDeviceName(dobot1)[0]))
```
* `GetAlarmsStateX(api)` function to replace `GetAlarmsState(api, maxLen)` that uses a hardcoded dictionary of bit addresses and alarm descriptions that is used for decoding the byte array returned by the default function and instead return the active alarms per name and description. The decoding of the alarms byte array is achieved by traversing the array by alarm index based on a hardcoded dictionary called alarms with the key being the bit index and the corresponding value the alarm description as described in the Dobot ALARM document. This results in retrieving only the active alarms with a time complexity of O(N) where N is the number of documented alarms and leaves unrelated LOC from the monitoring agent as this is a Dobot matter and is cleaner to be resolved in the Python encapsulation. Example of use:  
```python
print("Active alarms:")
for a in dType.GetAlarmsStateX(dobot0):
    print(a)
```
### Additions
* `GetActiveDobots()` function that returns the amount of currently connected Dobot Magicians  
* `DisconnectAll()` function to disconnect from all connected Dobot Magician devices and clean up any runtime files  
Both additions were due to the `ConnectDobotX(port)` function and their purpose is to accommodate it and enrich the flexibility it provides.
<br><br>

## Resources
- [Dobot API & Dobot Communication Protocol](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)
- [Dobot ALARM](http://www.dobot.it/wp-content/uploads/2018/03/dobot-magician-alarm-en.pdf)
- [JeVois: Standardized serial messages formatting](http://jevois.org/doc/UserSerialStyle.html)
- [Prometheus Documentation](https://prometheus.io/docs/introduction/overview/)
- [Grafana Documentation](https://grafana.com/docs/)
<br><br>
