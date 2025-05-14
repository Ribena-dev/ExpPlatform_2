#!/bin/bash

xterm -iconic -hold -e "python postercombiner.py" &
sleep 3; xterm -iconic -hold -e "python gui.py" &
sleep 1; xterm -iconic -hold -e "python joystick/combined_joystick_mv_avg.py" &  #joy cmd vel output .py file change this to test diffrent motions
sleep 1; xterm -iconic -hold -e "./tcp-server 8080 8081"
