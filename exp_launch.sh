#!/bin/bash

xterm -iconic -hold -e "python postercombiner.py" &
sleep 3; xterm -iconic -hold -e "python gui.py" &
<<<<<<< HEAD
<<<<<<< HEAD
#sleep 1; xterm -iconic -hold -e "python combined_joystick.py" &
=======
# sleep 1; xterm -iconic -hold -e "python combined_joystick.py" &
>>>>>>> parent of 798d688... modified_combined_joystick.py has a bug where is randomly doesn't allow side movements
sleep 1; xterm -iconic -hold -e "python modified_combined_joystick.py" &
sleep 1; xterm -iconic -hold -e "python sonar_test/sonar_get_min_dist.py"&
=======
sleep 1; xterm -iconic -hold -e "python joystick/combined_joystick_mv_avg.py" &  #joy cmd vel output .py file change this to test diffrent motions
>>>>>>> fcfed2a67f74421251f3d961e1c962de7a726bcd
sleep 1; xterm -iconic -hold -e "./tcp-server 8080 8081"
