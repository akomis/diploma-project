# Diploma Project
Implementation of Data Management and Monitoring System for Industrial IoT Applications

![Dobot Magician with Belt](/pics/dobot-magician-belt.png)
## Overview
A data management system that collects data from IoT devices such as [Dobot Magician](https://www.dobot.cc/dobot-magician/product-overview.html), [Conveyor belt](https://www.dobot.cc/products/conveyor-belt-kit-overview.html) and [Jevois Camera](http://www.jevois.org/) and monitors them.


## Dependencies
- [Python 3.x](https://www.python.org/downloads/windows/)
- [Dobot Demo](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)
- [Dobot Robot Driver](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=70#sub-download)
- [Prometheus](https://prometheus.io/download/)
- [Grafana](https://grafana.com/grafana/download?platform=windows)

## Installation
`git clone https://github.com/akomis/diploma-project.git`   
Place `agent.py` on the same directory as `demo-magician-python-64-master`


## Usage
`python3 agent.py [OPTIONS]`  

### Options
`-n <agent_name>		Give symbolic name to this monitoring agent (default: Agent0)`  
`-t <number>			Set timeout period between each routine cycle in milliseconds (min/default: 100)`  
`-a <number>			Define time in minutes for how long should the monitoring last (default: until interrupt)`  
`-d						Search, connect and monitor a Dobot Magician device`  
`-j						Search, connect and monitor a Jevois Camera device`  
`-p						Set port for prometheus (default: 8000)`  
`-h						Print this message`  

## Resources
- [Dobot API & Dobot Communication Protocol](https://www.dobot.cc/downloadcenter/dobot-magician.html?sub_cat=72#sub-download)
- [Prometheus Documentation](https://prometheus.io/docs/introduction/overview/)
- [Grafana Documentation](https://grafana.com/docs/)
