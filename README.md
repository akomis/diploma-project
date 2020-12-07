# Diploma Project
Implementation of Data Management and Monitoring System for Industrial IoT Applications

<div align="center">
<img alt="Dobot Magician with Belt" src="/pics/dobot-magician-belt.png">
</div>

## Overview
A data management system that collects data from IoT devices such as the [Dobot Magician](https://www.dobot.cc/dobot-magician/product-overview.html), [Conveyor belt](https://www.dobot.cc/products/conveyor-belt-kit-overview.html) (sliding rail) and [Jevois Camera](http://www.jevois.org/) that are used in an industrial setting and monitors them. Using [Prometheus](https://prometheus.io/) for monitoring device metrics, which is natively compatible with [Grafana](https://grafana.com/) to visualize them and provide insight.
<br><br><br>

## Dependencies
- [Python 3.x](https://www.python.org/downloads/windows/)
- [DobotDllType.py](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)
- [Dobot Robot Driver](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=70#sub-download)
- [Prometheus](https://prometheus.io/download/)
<br><br>

## Installation
`git clone https://github.com/akomis/diploma-project.git`   
Place `agent.py` on the same directory as `demo-magician-python-64-master`
<br><br>

## Configuration
Change agent's settings and choose which device data/attributes will be monitored by changing the `agent.conf` file. For enabling data to be monitored you can use `on`, `1`, `yes` or `true` and in order to not monitor certain data use `off`, `0`, `no`, `false` or remove the entry completely, depending on your preference.
### AGENT
|   Config Name   |                        Description                        | Type | Default |
|:---------------:|:---------------------------------------------------------:|:----:|:-------:|
|    AgentName    |                  Symbolic name for agent                  |  str |  Agent0 |
| RoutineInterval | Timeout period between each routine cycle in milliseconds |  int |   100   |
|  PrometheusPort |                Port for Prometheus endpoint               |  int |   8080  |

### DOBOT
|         Config Name        |                                          Description                                          | Prometheus Type | Default Setting |            API Call            |
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
|         AlarmsState        |                                     Device's active alarms                                    |  enum (alarms)  |        on       |     GetAlarmsState(api, 10)    |
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

For better insight check the default `agent.conf` included.
<br><br>

## Usage
Make sure that `agent.conf` is properly setup and in the same directory as the executable  
`$ python3 agent.py`
<br><br>

## Resources
- [Dobot API & Dobot Communication Protocol](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)
- [Dobot ALARM](http://www.dobot.it/wp-content/uploads/2018/03/dobot-magician-alarm-en.pdf)
- [Prometheus Documentation](https://prometheus.io/docs/introduction/overview/)
- [Grafana Documentation](https://grafana.com/docs/)
<br><br>
