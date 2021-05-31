<div align="center">
<h1>Diploma Project</h1>
<p>Monitoring System for Industrial IoT Applications</p>
<img alt="Dobot Magician with Belt" src="/pics/dobot-magician-belt-jevois.png">
</div>

## Overview
A monitoring system that collects data from devices such as the [Dobot Magician](https://www.dobot.cc/dobot-magician/product-overview.html), [Sliding Rail](https://www.dobot.cc/products/sliding-rail-kit-overview.html) and [JeVois Camera](http://www.jevois.org/) that are used in an industrial setting using [Prometheus](https://prometheus.io/) for monitoring device metrics. By using a visualization option compatible with Prometheus such as [Grafana](https://grafana.com/) one can visualize such metrics and provide better insight.  
The goal of this system is to be an effective, modular and extensible solution at monitoring such devices. This project aims at doing that while being efficient, without interfering with the normal operations of the devices, and be highly scalable in terms of the number of devices that can be monitored as well as the number of different types of devices it supports. Also provide high monitoring flexibility through a plethora of configuration options for the monitoring agent and what device attributes to be extracted and monitored, configured individually for each device.
<div align="center">
<img alt="overview" src="/pics/overview.jpg">
</div>
<br><br>

## Dependencies
- [Python 3.9+](https://www.python.org/downloads/windows/)
- [Prometheus](https://prometheus.io/download/#prometheus)
- [prometheus-client](https://pypi.org/project/prometheus-client/)

For using the `Dobot` device module
- [Dobot Robot Driver](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=70#sub-download)

For using the `Jevois` device module
- [pyserial](https://pythonhosted.org/pyserial/)
<br><br>

## Installation
- Install Python 3.9+
- Install Prometheus
- Install Magician Studio (which includes the Dobot Robot Driver)
- `pip3 install prometheus-client pyserial`
- `git clone https://github.com/akomis/diploma-project.git`
<br><br>

## Usage
When using the default configuration file location, make sure that `devices.conf` is properly setup and in the same directory as the executable.  
```
$ agent.py [-h] [-d DEVICES] [-n NAME] [-p PROMPORT] [-k] [-v] [-m]

Optional arguments:
  -h, --help                        show this help message and exit
  -d DEVICES, --devices DEVICES     specify configuration file path (default: ".\devices.conf")
  -n NAME, --name NAME              specify symbolic agent/station name used for separation/grouping of stations (default: "Agent0")
  -p PROMPORT, --promport PROMPORT  specify port number for the Prometheus endpoint (default: 8000)
  -k, --killswitch                  exit agent if error occurs in validation/connection phase
  -v, --verbose                     print actions with details in standard output
  -c, --color                       print color rich messages to terminal (terminal needs to support ANSI escape colors)
  -m, --more                        open README.md with configuration and implementation details and exit
```
<div align="center">
<img alt="overview" src="/pics/agentruntime.png">
</div>
<br>

## Device Discovery/Configuration
Choose which devices and which data/attributes of those will be monitored by changing the `devices.conf` file.  
For monitoring a device the corresponding class in device_modules.py must exist. For the agent to discover the device and use the appropriate module for connecting, fetching and disconnecting (see more in "Extensibility" section), a device entry must exist in the configuration file e.g. `class DeviceType` a `[DeviceType:<port>]`. One can connect multiple devices through various ports (serial port/IP address).    
For enabling data to be monitored you can use `on`, `1`, `yes` or `true` and in order to not monitor certain data use `off`, `0`, `no`, `false` depending on your preference. By removing an entry completely the value for the entry will be resolved to the default. All keys are case-insensitive but all section names must be exactly the same as the class name representing the device module.  
Each device entry supports by default the `Timeout` attribute which sets the timeout period in milliseconds in between fetches and defaults to 0.  
All configuration is parsed and validated based on the above information, before the start of the routine, and warns the user for any invalid entries, fields and values.  
For more details on the configuration settings for the Dobot Magician and JeVois camera devices check their respective tables below with all options and their details.  

### Dobot
In order for the agent to find a Dobot Magician and connect to it, a section of the device, `[Dobot:PORT]` must exist in the configuration file e.g. `[Dobot:COM7]` for serial or `[Dobot:192.168.0.3]` for connecting through WiFi (WLAN).
|         Config Name        |                                          Description                                          | Prometheus Type | Default |          API Call         |
|:--------------------------:|:---------------------------------------------------------------------------------------------:|:---------------:|:-------:|:-------------------------:|
|          DeviceSN          |                                     Device's serial number                                    |    info (str)   |    on   |        GetDeviceSN        |
|         DeviceName         |                                      Device's name/alias                                      |    info (str)   |    on   |       GetDeviceName       |
|        DeviceVersion       |                            Device's verion (major.minor.0.revision)                           |    info (str)   |    on   |      GetDeviceVersion     |
|         DeviceTime         |                                      Device's clock/time                                      |    info (str)   |   off   |       GetDeviceTime       |
|         QueueIndex         |                                 Current index in command queue                                |   gauge (int)   |   off   |  GetQueuedCmdCurrentIndex |
|            PoseX           |                       Real-time cartesian coordinate of device's X axis                       |  gauge (float)  |    on   |          GetPose          |
|            PoseY           |                       Real-time cartesian coordinate of device's Y axis                       |  gauge (float)  |    on   |          GetPose          |
|            PoseZ           |                       Real-time cartesian coordinate of device's Z axis                       |  gauge (float)  |    on   |          GetPose          |
|            PoseR           |                       Real-time cartesian coordinate of device's R axis                       |  gauge (float)  |    on   |          GetPose          |
|          AngleBase         |                                        Base joint angle                                       |  gauge (float)  |    on   |          GetPose          |
|        AngleRearArm        |                                      Rear arm joint angle                                     |  gauge (float)  |    on   |          GetPose          |
|        AngleForearm        |                                      Forearm joint angle                                      |  gauge (float)  |    on   |          GetPose          |
|      AngleEndEffector      |                                    End effector joint angle                                   |  gauge (float)  |    on   |          GetPose          |
|         AlarmsState        |                                     Device's active alarms                                    |  enum (alarms)  |    on   |      GetAlarmsStateX      |
|            HomeX           |                                    Home position for X axis                                   |  gauge (float)  |   off   |       GetHOMEParams       |
|            HomeY           |                                    Home position for Y axis                                   |  gauge (float)  |   off   |       GetHOMEParams       |
|            HomeZ           |                                    Home position for Z axis                                   |  gauge (float)  |   off   |       GetHOMEParams       |
|            HomeR           |                                    Home position for R axis                                   |  gauge (float)  |   off   |       GetHOMEParams       |
|        EndEffectorX        |                                 X-axis offset of end effector                                 |  gauge (float)  |   off   |    GetEndEffectorParams   |
|        EndEffectorY        |                                 Y-axis offset of end effector                                 |  gauge (float)  |   off   |    GetEndEffectorParams   |
|        EndEffectorZ        |                                 Z-axis offset of end effector                                 |  gauge (float)  |   off   |    GetEndEffectorParams   |
|         LaserStatus        |                               Status (enabled/disabled) of laser                              |   enum (bool)   |   off   |    GetEndEffectorLaser    |
|      SuctionCupStatus      |                            Status (enabled/disabled) of suction cup                           |   enum (bool)   |   off   |  GetEndEffectorSuctionCup |
|        GripperStatus       |                              Status (enabled/disabled) of gripper                             |   enum (bool)   |   off   |   GetEndEffectorGripper   |
|       JogBaseVelocity      |                          Velocity (°/s) of base joint in jogging mode                         |  gauge (float)  |   off   |     GetJOGJointParams     |
|     JogRearArmVelocity     |                        Velocity (°/s) of rear arm joint in jogging mode                       |  gauge (float)  |   off   |     GetJOGJointParams     |
|     JogForearmVelocity     |                        Velocity (°/s) of forearm joint in jogging mode                        |  gauge (float)  |   off   |     GetJOGJointParams     |
|   JogEndEffectorVelocity   |                      Velocity (°/s) of end effector joint in jogging mode                     |  gauge (float)  |   off   |     GetJOGJointParams     |
|     JogBaseAcceleration    |                       Acceleration (°/s^2) of base joint in jogging mode                      |  gauge (float)  |   off   |     GetJOGJointParams     |
|   JogRearArmAcceleration   |                     Acceleration (°/s^2) of rear arm joint in jogging mode                    |  gauge (float)  |   off   |     GetJOGJointParams     |
|   JogForearmAcceleration   |                     Acceleration (°/s^2) of forearm joint in jogging mode                     |  gauge (float)  |   off   |     GetJOGJointParams     |
| JogEndEffectorAcceleration |                   Acceleration (°/s^2) of end effector joint in jogging mode                  |  gauge (float)  |   off   |     GetJOGJointParams     |
|      JogAxisXVelocity      |           Velocity (mm/s) of device's X axis (cartesian coordinate) in jogging mode           |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|      JogAxisYVelocity      |           Velocity (mm/s) of device's Y axis (cartesian coordinate) in jogging mode           |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|      JogAxisZVelocity      |           Velocity (mm/s) of device's Z axis (cartesian coordinate) in jogging mode           |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|      JogAxisRVelocity      |           Velocity (mm/s) of device's R axis (cartesian coordinate) in jogging mode           |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|    JogAxisXAcceleration    |        Acceleration (mm/s^2) of device's X axis (cartesian coordinate) in jogging mode        |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|    JogAxisYAcceleration    |        Acceleration (mm/s^2) of device's Y axis (cartesian coordinate) in jogging mode        |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|    JogAxisZAcceleration    |        Acceleration (mm/s^2) of device's Z axis (cartesian coordinate) in jogging mode        |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|    JogAxisRAcceleration    |        Acceleration (mm/s^2) of device's R axis (cartesian coordinate) in jogging mode        |  gauge (float)  |   off   |   GetJOGCoordinateParams  |
|      JogVelocityRatio      |       Velocity ratio of all axis (joint and cartesian coordinate system) in jogging mode      |  gauge (float)  |   off   |     GetJOGCommonParams    |
|    JogAccelerationRatio    |     Acceleration ratio of all axis (joint and cartesian coordinate system) in jogging mode    |  gauge (float)  |   off   |     GetJOGCommonParams    |
|       PtpBaseVelocity      |                      Velocity (°/s) of base joint in point to point mode                      |  gauge (float)  |   off   |     GetPTPJointParams     |
|     PtpRearArmVelocity     |                    Velocity (°/s) of rear arm joint in point to point mode                    |  gauge (float)  |   off   |     GetPTPJointParams     |
|     PtpForearmVelocity     |                     Velocity (°/s) of forearm joint in point to point mode                    |  gauge (float)  |   off   |     GetPTPJointParams     |
|   PtpEndEffectorVelocity   |                  Velocity (°/s) of end effector joint in point to point mode                  |  gauge (float)  |   off   |     GetPTPJointParams     |
|     PtpBaseAcceleration    |                   Acceleration (°/s^2) of base joint in point to point mode                   |  gauge (float)  |   off   |     GetPTPJointParams     |
|   PtpRearArmAcceleration   |                 Acceleration (°/s^2) of rear arm joint in point to point mode                 |  gauge (float)  |   off   |     GetPTPJointParams     |
|   PtpForearmAcceleration   |                  Acceleration (°/s^2) of forearm joint in point to point mode                 |  gauge (float)  |   off   |     GetPTPJointParams     |
| PtpEndEffectorAcceleration |               Acceleration (°/s^2) of end effector joint in point to point mode               |  gauge (float)  |   off   |     GetPTPJointParams     |
|     PtpAxisXYZVelocity     |     Velocity (mm/s) of device's X, Y, Z axis (cartesian coordinate) in point to point mode    |  gauge (float)  |   off   |   GetPTPCoordinateParams  |
|      PtpAxisRVelocity      |        Velocity (mm/s) of device's R axis (cartesian coordinate) in point to point mode       |  gauge (float)  |   off   |   GetPTPCoordinateParams  |
|   PtpAxisXYZAcceleration   |  Acceleration (mm/s^2) of device's X, Y, Z axis (cartesian coordinate) in point to point mode |  gauge (float)  |   off   |   GetPTPCoordinateParams  |
|    PtpAxisRAcceleration    |     Acceleration (mm/s^2) of device's R axis (cartesian coordinate) in point to point mode    |  gauge (float)  |   off   |   GetPTPCoordinateParams  |
|      PtpVelocityRatio      |   Velocity ratio of all axis (joint and cartesian coordinate system) in point to point mode   |  gauge (float)  |   off   |     GetPTPCommonParams    |
|    PtpAccelerationRatio    | Acceleration ratio of all axis (joint and cartesian coordinate system) in point to point mode |  gauge (float)  |   off   |     GetPTPCommonParams    |
|        LiftingHeight       |                                  Lifting height in jump mode                                  |  gauge (float)  |   off   |      GetPTPJumpParams     |
|         HeighLimit         |                                Max lifting height in jump mode                                |  gauge (float)  |   off   |      GetPTPJumpParams     |
|         CpVelocity         |                                   Velocity (mm/s) in cp mode                                  |  gauge (float)  |   off   |        GetCPParams        |
|       CpAcceleration       |                                Acceleration (mm/s^2) in cp mode                               |  gauge (float)  |   off   |        GetCPParams        |
|       ArcXYZVelocity       |                          Velocity (mm/s) of X, Y, Z axis in arc mode                          |  gauge (float)  |   off   |        GetARCParams       |
|        ArcRVelocity        |                             Velocity (mm/s) of R axis in arc mode                             |  gauge (float)  |   off   |        GetARCParams       |
|     ArcXYZAcceleration     |                       Acceleration (mm/s^2) of X, Y, Z axis in arc mode                       |  gauge (float)  |   off   |        GetARCParams       |
|      ArcRAcceleration      |                          Acceleration (mm/s^2) of R axis in arc mode                          |  gauge (float)  |   off   |        GetARCParams       |
|     AngleStaticErrRear     |                               Rear arm angle sensor static error                              |  gauge (float)  |   off   | GetAngleSensorStaticError |
|     AngleStaticErrFront    |                               Forearm angle sensor static error                               |  gauge (float)  |   off   | GetAngleSensorStaticError |
|        AngleCoefRear       |                         Rear arm angle sensor linearization parameter                         |  gauge (float)  |   off   |     GetAngleSensorCoef    |
|       AngleCoefFront       |                          Forearm angle sensor linearization parameter                         |  gauge (float)  |   off   |     GetAngleSensorCoef    |
|      SlidingRailStatus     |                            Sliding rail's status (enabled/disabled)                           |   enum (bool)   |   off   |       GetDeviceWithL      |
|       SlidingRailPose      |                              Sliding rail's real-time pose in mm                              |  gauge (float)  |   off   |          GetPoseL         |
|   SlidingRailJogVelocity   |                        Velocity (mm/s) of sliding rail in jogging mode                        |  gauge (float)  |   off   |       GetJOGLParams       |
| SlidingRailJogAcceleration |                     Acceleration (mm/s^2) of sliding rail in jogging mode                     |  gauge (float)  |   off   |       GetJOGLParams       |
|   SlidingRailPtpVelocity   |                     Velocity (mm/s) of sliding rail in point to point mode                    |  gauge (float)  |   off   |       GetPTPLParams       |
| SlidingRailPtpAcceleration |                  Acceleration (mm/s^2) of sliding rail in point to point mode                 |  gauge (float)  |   off   |       GetPTPLParams       |
|      WifiModuleStatus      |                             Wifi module status (enabled/disabled)                             |   enum (bool)   |   off   |     GetWIFIConfigMode     |
|    WifiConnectionStatus    |                        Wifi connection status (connected/not connected)                       |   enum (bool)   |   off   |    GetWIFIConnectStatus   |
|          WifiSSID          |                                      Configured Wifi SSID                                     |    info (str)   |   off   |        GetWIFISSID        |
|        WifiPassword        |                                    Configured Wifi Password                                   |    info (str)   |   off   |      GetWIFIPassword      |
|        WifiIPAddress       |                                      Device's IP address                                      |    info (str)   |   off   |      GetWIFIIPAddress     |
|         WifiNetmask        |                                          Subnet mask                                          |    info (str)   |   off   |       GetWIFINetmask      |
|         WifiGateway        |                                        Default Gateway                                        |    info (str)   |   off   |       GetWIFIGateway      |
|           WifiDNS          |                                              DNS                                              |    info (str)   |   off   |         GetWIFIDNS        |

Note: Only enable the WiFi attributes if the Dobot is currently not executing any movement commands.   

### Jevois
In order for the agent to find a JeVois camera and connect to it, a section of the device `[Jevois:PORT]` must exist in the configuration file (e.g. `[Jevois:COM3]`) with the only difference that the port can only be serial as the camera does not support wireless connection with the host. For monitoring the object's identity one must provide a space-separated list with object names (representing the pictures (file names) the camera uses for training) in the "objects" entry (e.g. objects = cube.png pen.png paper.png).
|    Config Name   |          Description         |  Prometheus Type | Default |   Message Type   |
|:----------------:|:----------------------------:|:----------------:|:-------:|:----------------:|
| ObjectIdentified |   Identified object's name   |    enum (str)    |    on   |  1D/2D/3D Normal |
|  ObjectLocation  | Identified object's location | gauge(s) (float) |    on   |  1D/2D/3D Normal |
|    ObjectSize    |   Identified object's size   |   gauge (float)  |   off   |  1D/2D/3D Normal |

For a more practical insight check the default `devices.conf` included.
<br><br>

## Querying
The supported device modules currently implement the device_id, device_type and station labels for querying based on specific device id (e.g. Dobot:192.168.43.4), device type (e.g. Dobot, Jevois) and agent/station name, respectively. You can find the monitoring options and their respective Prometheus metric names for querying below.

### Dobot
|         Config Name        |     Prometheus Metric Name    |
|:--------------------------:|:-----------------------------:|
|          DeviceSN          |  dobot_magician_info{serial}  |
|         DeviceName         |   dobot_magician_info{name}   |
|        DeviceVersion       |  dobot_magician_info{version} |
|         DeviceTime         |          device_time          |
|         QueueIndex         |          queue_index          |
|            PoseX           |             pose_x            |
|            PoseY           |             pose_y            |
|            PoseZ           |             pose_z            |
|            PoseR           |             pose_r            |
|          AngleBase         |           angle_base          |
|        AngleRearArm        |         angle_rear_arm        |
|        AngleForearm        |         angle_forearm         |
|      AngleEndEffector      |       angle_end_effector      |
|         AlarmsState        |             alarms            |
|            HomeX           |             home_x            |
|            HomeY           |             home_y            |
|            HomeZ           |             home_z            |
|            HomeR           |             home_r            |
|        EndEffectorX        |         end_effector_x        |
|        EndEffectorY        |         end_effector_y        |
|        EndEffectorZ        |         end_effector_z        |
|         LaserStatus        |          laser_status         |
|      SuctionCupStatus      |       suction_cup_status      |
|        GripperStatus       |         gripper_status        |
|       JogBaseVelocity      |       jog_base_velocity       |
|     JogRearArmVelocity     |     jog_rear_arm_velocity     |
|     JogForearmVelocity     |      jog_forearm_velocity     |
|   JogEndEffectorVelocity   |   jog_end_effector_velocity   |
|     JogBaseAcceleration    |     jog_base_acceleration     |
|   JogRearArmAcceleration   |   jog_rear_arm_acceleration   |
|   JogForearmAcceleration   |    jog_forearm_acceleration   |
| JogEndEffectorAcceleration | jog_end_effector_acceleration |
|      JogAxisXVelocity      |      jog_axis_x_velocity      |
|      JogAxisYVelocity      |      jog_axis_y_velocity      |
|      JogAxisZVelocity      |      jog_axis_z_velocity      |
|      JogAxisRVelocity      |      jog_axis_r_velocity      |
|    JogAxisXAcceleration    |    jog_axis_x_acceleration    |
|    JogAxisYAcceleration    |    jog_axis_y_acceleration    |
|    JogAxisZAcceleration    |    jog_axis_z_acceleration    |
|    JogAxisRAcceleration    |    jog_axis_r_acceleration    |
|      JogVelocityRatio      |       jog_velocity_ratio      |
|    JogAccelerationRatio    |     jog_acceleration_ratio    |
|       PtpBaseVelocity      |       ptp_base_velocity       |
|     PtpRearArmVelocity     |     ptp_rear_arm_velocity     |
|     PtpForearmVelocity     |      ptp_forearm_velocity     |
|   PtpEndEffectorVelocity   |   ptp_end_effector_velocity   |
|     PtpBaseAcceleration    |     ptp_base_acceleration     |
|   PtpRearArmAcceleration   |   ptp_rear_arm_acceleration   |
|   PtpForearmAcceleration   |    ptp_forearm_acceleration   |
| PtpEndEffectorAcceleration | ptp_end_effector_acceleration |
|     PtpAxisXYZVelocity     |     ptp_axis_xyz_velocity     |
|      PtpAxisRVelocity      |      ptp_axis_r_velocity      |
|   PtpAxisXYZAcceleration   |  ptp_axis_x_y_z_acceleration  |
|    PtpAxisRAcceleration    |    ptp_axis_r_acceleration    |
|      PtpVelocityRatio      |       ptp_velocity_ratio      |
|    PtpAccelerationRatio    |     ptp_acceleration_ratio    |
|        LiftingHeight       |         lifting_height        |
|         HeighLimit         |          height_limit         |
|         CpVelocity         |          cp_velocity          |
|       CpAcceleration       |        cp_acceleration        |
|       ArcXYZVelocity       |       arc_x_y_z_velocity      |
|        ArcRVelocity        |         arc_r_velocity        |
|     ArcXYZAcceleration     |     arc_x_y_z_acceleration    |
|      ArcRAcceleration      |       arc_r_acceleration      |
|     AngleStaticErrRear     |     angle_static_err_rear     |
|     AngleStaticErrFront    |      arc_static_err_front     |
|        AngleCoefRear       |        angle_coef_rear        |
|       AngleCoefFront       |        angle_coef_front       |
|      SlidingRailStatus     |      sliding_rail_status      |
|       SlidingRailPose      |       sliding_rail_pose       |
|   SlidingRailJogVelocity   |   sliding_rail_jog_velocity   |
| SlidingRailJogAcceleration | sliding_rail_jog_acceleration |
|   SlidingRailPtpVelocity   |   sliding_rail_ptp_velocity   |
| SlidingRailPtpAcceleration | sliding_rail_ptp_acceleration |
|      WifiModuleStatus      |       wifi_module_status      |
|    WifiConnectionStatus    |     wifi_connection_status    |
|          WifiSSID          |        wifi_info{ssid}        |
|        WifiPassword        |      wifi_info{password}      |
|        WifiIPAddress       |     wifi_info{ip_address}     |
|         WifiNetmask        |       wifi_info{netmask}      |
|         WifiGateway        |       wifi_info{gateway}      |
|           WifiDNS          |         wifi_info{dns}        |

### Jevois
|    Config Name   |           Prometheus Metric Name           |
|:----------------:|:------------------------------------------:|
| ObjectIdentified |               object_id_{port}             |
|  ObjectLocation  |          object_location_{x\|y\|z}         |
|    ObjectSize    |                 object_size                |

<br>

## Fetch Times
### Dobot
For the fetch times regarding the regarding the Dobot Magician device and the Dobot device module for all attributes the average fetch time while connected through usb was ~15ms and when connected through WiFi ~24ms, which leads to the conclusion that the general wireless overhead is around 9ms. However, for some of the WiFi related (see Tabel 3.4 and Image 3.1) and the GetDeviceTime api calls, the fetch times were lower (better) when connected wirelessly. These attributes are also the only attributes that their fetch times always exceed 500ms in both cases. Since these WiFi attributes are of type Info, which means they are only fetched once at the start of the monitoring phase this is not a bottleneck to the fetching routine. It has been observed that if the Dobot is under any kind of movement starting the monitoring with the WiFi attributes enabled will affect the normal operations of the robot (supposedly due to the high fetch times interfering with the normal command execution) so only enable these attributes if the monitoring will start prior to the normal operations. Since DeviceTime is the only non-info attribute that exceeds the average low fetch times (by a great amount), it is advised that enabling the DeviceTime option is avoided. A detailed look of the results for both wired and wireless connections can be seen in the table below.

<table>
<thead>
  <tr>
    <th rowspan="2">API Call</th>
    <th colspan="3">Wired</th>
    <th colspan="3">Wireless</th>
  </tr>
  <tr>
    <td>Average</td>
    <td>Min</td>
    <td>Max</td>
    <td>Average</td>
    <td>Min</td>
    <td>Max</td>
  </tr>
</thead>
<tbody>
  <tr>
    <td>GetDeviceTime</td>
    <td>719</td>
    <td>505</td>
    <td>1014</td>
    <td>608</td>
    <td>205</td>
    <td>655</td>
  </tr>
  <tr>
    <td>GetQueuedCmdCurrentIndex</td>
    <td>15</td>
    <td>10</td>
    <td>20</td>
    <td>27</td>
    <td>18</td>
    <td>147</td>
  </tr>
  <tr>
    <td>GetPose</td>
    <td>16</td>
    <td>12</td>
    <td>23</td>
    <td>24</td>
    <td>20</td>
    <td>33</td>
  </tr>
  <tr>
    <td>GetAlarmsStateX</td>
    <td>17</td>
    <td>10</td>
    <td>20</td>
    <td>25</td>
    <td>19</td>
    <td>32</td>
  </tr>
  <tr>
    <td>GetHOMEParams</td>
    <td>20</td>
    <td>13</td>
    <td>24</td>
    <td>29</td>
    <td>22</td>
    <td>34</td>
  </tr>
  <tr>
    <td>GetAutoLevelingResult</td>
    <td>15</td>
    <td>10</td>
    <td>21</td>
    <td>24</td>
    <td>18</td>
    <td>31</td>
  </tr>
  <tr>
    <td>GetEndEffectorParams</td>
    <td>15</td>
    <td>10</td>
    <td>22</td>
    <td>23</td>
    <td>18</td>
    <td>34</td>
  </tr>
  <tr>
    <td>GetEndEffectorLaser</td>
    <td>13</td>
    <td>7</td>
    <td>20</td>
    <td>23</td>
    <td>17</td>
    <td>30</td>
  </tr>
  <tr>
    <td>GetEndEffectorSuctionCup</td>
    <td>13</td>
    <td>7</td>
    <td>20</td>
    <td>23</td>
    <td>18</td>
    <td>34</td>
  </tr>
  <tr>
    <td>GetEndEffectorGripper</td>
    <td>14</td>
    <td>8</td>
    <td>22</td>
    <td>22</td>
    <td>18</td>
    <td>29</td>
  </tr>
  <tr>
    <td>GetJOGJointParams</td>
    <td>17</td>
    <td>12</td>
    <td>31</td>
    <td>26</td>
    <td>21</td>
    <td>33</td>
  </tr>
  <tr>
    <td>GetJOGCoordinateParams</td>
    <td>17</td>
    <td>9</td>
    <td>22</td>
    <td>28</td>
    <td>20</td>
    <td>33</td>
  </tr>
  <tr>
    <td>GetJOGCommonParams</td>
    <td>15</td>
    <td>9</td>
    <td>20</td>
    <td>25</td>
    <td>16</td>
    <td>30</td>
  </tr>
  <tr>
    <td>GetPTPJointParams</td>
    <td>16</td>
    <td>12</td>
    <td>23</td>
    <td>26</td>
    <td>20</td>
    <td>33</td>
  </tr>
  <tr>
    <td>GetPTPCoordinateParams</td>
    <td>17</td>
    <td>10</td>
    <td>24</td>
    <td>25</td>
    <td>20</td>
    <td>31</td>
  </tr>
  <tr>
    <td>GetPTPCommonParams</td>
    <td>14</td>
    <td>9</td>
    <td>24</td>
    <td>24</td>
    <td>17</td>
    <td>29</td>
  </tr>
  <tr>
    <td>GetPTPJumpParams</td>
    <td>15</td>
    <td>9</td>
    <td>25</td>
    <td>24</td>
    <td>17</td>
    <td>31</td>
  </tr>
  <tr>
    <td>GetCPParams</td>
    <td>14</td>
    <td>9</td>
    <td>21</td>
    <td>24</td>
    <td>19</td>
    <td>30</td>
  </tr>
  <tr>
    <td>GetARCParams</td>
    <td>16</td>
    <td>9</td>
    <td>21</td>
    <td>24</td>
    <td>18</td>
    <td>32</td>
  </tr>
  <tr>
    <td>GetAngleSensorStaticError</td>
    <td>15</td>
    <td>9</td>
    <td>22</td>
    <td>25</td>
    <td>20</td>
    <td>30</td>
  </tr>
  <tr>
    <td>GetAngleSensorCoef</td>
    <td>16</td>
    <td>8</td>
    <td>22</td>
    <td>25</td>
    <td>18</td>
    <td>39</td>
  </tr>
  <tr>
    <td>GetDeviceWithL</td>
    <td>14</td>
    <td>8</td>
    <td>19</td>
    <td>23</td>
    <td>17</td>
    <td>29</td>
  </tr>
  <tr>
    <td>GetPoseL</td>
    <td>13</td>
    <td>8</td>
    <td>20</td>
    <td>22</td>
    <td>18</td>
    <td>31</td>
  </tr>
  <tr>
    <td>GetJOGLParams</td>
    <td>13</td>
    <td>10</td>
    <td>20</td>
    <td>21</td>
    <td>17</td>
    <td>28</td>
  </tr>
  <tr>
    <td>GetPTPLParams</td>
    <td>13</td>
    <td>9</td>
    <td>20</td>
    <td>23</td>
    <td>18</td>
    <td>30</td>
  </tr>
  <tr>
    <td>GetWIFIConfigMode</td>
    <td>14</td>
    <td>8</td>
    <td>20</td>
    <td>24</td>
    <td>18</td>
    <td>30</td>
  </tr>
  <tr>
    <td>GetWIFIConnectStatus</td>
    <td>13</td>
    <td>10</td>
    <td>21</td>
    <td>22</td>
    <td>18</td>
    <td>28</td>
  </tr>
  <tr>
    <td>GetDeviceSN</td>
    <td>13</td>
    <td>9</td>
    <td>19</td>
    <td>24</td>
    <td>19</td>
    <td>40</td>
  </tr>
  <tr>
    <td>GetDeviceName</td>
    <td>16</td>
    <td>11</td>
    <td>22</td>
    <td>26</td>
    <td>18</td>
    <td>34</td>
  </tr>
  <tr>
    <td>GetDeviceVersion</td>
    <td>16</td>
    <td>9</td>
    <td>20</td>
    <td>26</td>
    <td>20</td>
    <td>32</td>
  </tr>
  <tr>
    <td>GetWIFISSID</td>
    <td>521</td>
    <td>516</td>
    <td>528</td>
    <td>536</td>
    <td>531</td>
    <td>541</td>
  </tr>
  <tr>
    <td>GetWIFIPassword</td>
    <td>1676</td>
    <td>1011</td>
    <td>2541</td>
    <td>1153</td>
    <td>1046</td>
    <td>1647</td>
  </tr>
  <tr>
    <td>GetWIFIIPAddress</td>
    <td>1499</td>
    <td>1012</td>
    <td>2016</td>
    <td>1368</td>
    <td>1122</td>
    <td>1739</td>
  </tr>
  <tr>
    <td>GetWIFINetmask</td>
    <td>1550</td>
    <td>1012</td>
    <td>2031</td>
    <td>1200</td>
    <td>625</td>
    <td>1727</td>
  </tr>
  <tr>
    <td>GetWIFIGateway</td>
    <td>1635</td>
    <td>1013</td>
    <td>2537</td>
    <td>1141</td>
    <td>1066</td>
    <td>1630</td>
  </tr>
  <tr>
    <td>GetWIFIDNS</td>
    <td>1483</td>
    <td>1013</td>
    <td>2029</td>
    <td>1145</td>
    <td>1075</td>
    <td>1736</td>
  </tr>
</tbody>
</table>

### Jevois
Since the Jevois is directly connected to the host machine through a serial port, considering that timeout to the serial reading is set to 0 the consecutive fetch times, including reading, stripping and decoding the standardized message are on average around 1ms with a max fetch time of 4ms. The testing was done with Normal style 2D standardized messages. Fetch times on empty messages were not counted.
<br><br>

## Scalability
The system can scale (monitoring station level) vertically as the agent can connect to and monitor a variable amount of devices, a number constrained by the monitoring station's available ports and resources. In addition for larger and more complex setups one can scale the system vertically by adding multiple monitoring stations. For load balancing a section of the monitoring one can use multiple monitoring stations with the same name and query metrics based on that. For the configuration of these stations one can tweak their respective Prometheus and agent configurations. For their coordination one can use a [Prometheus Hierarchical Federation](https://prometheus.io/docs/prometheus/latest/federation/#hierarchical-federation).
<br><br>

## Extensibility
The agent currently supports Dobot Magician and JeVois Camera devices. For extending the agent's capabilities to support a different type of device one can create a device class (device module) and place it in the `device_modules.py`. This class needs to implement the `Device` abstract base class (virtual interface). The name of the class is determining the name that the agent will use to discover a device through `devices.conf`, connect to it, fetch its attributes (and update prometheus endpoint) and disconnect from the device.  
The member that needs to be implemented is the options{} dictionary and the methods are the connect(), fetch() and disconnect() methods. More specifically
### `options{}`
A dictionary that includes all the valid fields/options a device can have in the configuration file (monitored attribute fields) as keys and their default value (also used to validate the type of the value in the configuration file) as values.
### `connect()`
Responsible for connecting to the device, initialize the Prometheus metrics and other necessary device information that is vital for the use of the other methods. If the connection attempt is unsuccessful it should raise an exception.
### `fetch()`
Used to extract all enabled monitoring attributes for said device and update the Prometheus metrics accordingly. If the fetch attempt is unsuccessful it should raise an exception.
### `disconnect()`
Responsible for disconnecting the device, close any open ports/streams and remove any runtime temporary files regarding the device.  

All other necessary modules needed for implementing the above functions (e.g. `DobotDllTypeX.py` for the `Dobot` device module) and any runtime files should be included in the `runtime` directory.  
For a practical example one can review the source code in `device_modules.py` and more specifically the `Dobot` and `Jevois` classes (device modules).
<br><br>

## Load Balancing
Monitoring stations can be used to group a set of devices (probably in the same section in the workflow) where the agent/station name option becomes relevant. Monitoring/grouping many devices through a single monitoring station (host computer) can bottleneck it and affect monitoring rates. In this case, another use of the agent’s name attribute is naming two or more stations with the same name thus creating a single virtual station. Since each metric supports the station label one can query based on the station name and get metrics from multiple stations while looking like it is one. For setups where the use of multiple monitoring stations is doable, one can use more than one monitoring station for monitoring a section and thus if distributed properly load balance the monitoring stations while still looking like a unified station.
<br><br>

## Testing
A small manageable testing utility is the `test.py` script which includes a number of functions respective to different functional (f) and performance (p) tests for different device modules. Each functional test represents a function of a device module. Each test returns true in successful completion and false otherwise. Performance tests produce results/statistics to standard output that can be further analyzed.  
The naming convention for better organization and use of the test functions
is as follows: `typeOfTest_moduleName_description`  
e.g.    For a performance test regarding the Jevois module => `p_Jevois_DescriptionOfTest()`  
        For a functional test regarding the Dobot module => `f_Dobot_DescriptionOfTest()`
<br><br>

## DobotDllType.py Fork (DobotDllTypeX.py)
Throughout the usage of the Dobot API, some minor issues arose with fetching certain useful attributes, either due to typos or bugs in the API. Fixing those bugs to not sacrifice any wanted data led to a greater understanding of how the Dobot API works and resulted to more changes that make the Dobot API more flexible and more convenient to use. No functions are changed as to not break any existing implementations utilizing the official API as all changes to functions are done through wrappers. For using the improved functions provided by the fork one should create a `runtime` directory in the directory of the agent with all the files provided in the [Dobot Demo](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download). For importing and utilizing the fork place `DobotDllTypeX` inside project's directory and add `import DobotDllTypeX as dTypeX` to the script. (for this reading `dType` refers to the original `DobotDllType.py` and `dTypeX` to the fork)
### Fixes
* Fixed `GetPoseL(api)` function, which returns the position of the sliding rail (if there is one connected to the robot), by importing the math library which is required for the needs of the function, however not imported by default.
### Improvements
* `loadX()` function to replace `load()` that implements loading individual dll (DobotDll.dll instances) for each connected device in order to enable parallel connection with multiple dobots. In addition to that a "connections" list is maintained by the API to include all connected devices (their dll). This function is not meant to be called explicitly.
* `ConnectDobotX(port)` function to replace `api = dType.load(); state = dType.ConnectDobot(api, port, baudrate)` for connecting to a Dobot Magician device. The main improvement this change provides is that through its implementation, by utilizing the `loadX()`, it allows parallel connections to Dobot Magicians and also removes the need to issue it separately. When using the default API this model is not feasible and multiple Dobot Magicians can be connected concurrently with a switching overhead of approximately 0.3 seconds per switch. Apart from the performance benefits this function provides, it is also a more readable and convenient option for connecting a Dobot Magician device as all the standardized procedures are included either in the function or through default arguments. Example of use:  
```python
dobot0, state0 = dTypeX.ConnectDobotX("192.168.43.4")
dobot1, state1 = dTypeX.ConnectDobotX("192.168.43.5")

if state0[0] == dTypeX.DobotConnect.DobotConnect_NoError:
    print("192.168.43.4 name: " + str(dType.GetDeviceName(dobot0)[0]))

if state1[0] == dTypeX.DobotConnect.DobotConnect_NoError:
    print("192.168.43.5 name: " + str(dType.GetDeviceName(dobot1)[0]))
```
* `GetAlarmsStateX(api)` function to replace `GetAlarmsState(api, maxLen)` that uses a hardcoded dictionary of bit addresses and alarm descriptions that is used for decoding the byte array returned by the default function and instead return the active alarms per name and description. The decoding of the alarms byte array is achieved by traversing the array by alarm index based on a hardcoded dictionary called alarms with the key being the bit index and the corresponding value the alarm description as described in the Dobot ALARM document. This results in retrieving only the active alarms with a time complexity of O(N) where N is the number of documented alarms and leaves unrelated LOC from the monitoring agent as this is a Dobot matter and is cleaner to be resolved in the Python encapsulation. Example of use:  
```python
print("Active alarms:")
for a in dTypeX.GetAlarmsStateX(dobot0):
    print(a)
```
### Additions
* `GetActiveDobots()` function that returns the amount of currently connected Dobot Magicians  
* `DisconnectAll()` function to disconnect from all connected Dobot Magician devices and clean up any runtime files  
Both additions were due to the creation of the `ConnectDobotX(port)` function and their purpose is to accommodate it and enrich the flexibility it provides.
<br><br>

## Resources
- [Prometheus Documentation](https://prometheus.io/docs/introduction/overview/)
- [Dobot API & Dobot Communication Protocol](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)
- [Dobot ALARM](http://www.dobot.it/wp-content/uploads/2018/03/dobot-magician-alarm-en.pdf)
- [JeVois: Standardized serial messages formatting](http://jevois.org/doc/UserSerialStyle.html)
