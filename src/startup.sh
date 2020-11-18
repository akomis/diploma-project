#!/bin/bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:DOBOT_LIB_PATH
./prometheus --config.file=prometheus.yml && main.py || printf "Couldn't properly setup the enviroment for program to launch"
