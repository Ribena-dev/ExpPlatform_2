# !/bin/bash

xterm -iconic -hold -e "python postercombiner.py" &
sleep 3; xterm -iconic -hold -e "python gui.py" &
sleep 1; xterm -iconic -hold -e "python simplified_combined_joystick.py" &
sleep 1; xtrem -iconic -hold -e "python lidar_processor.py"&

#sleep 1; xterm -iconic -hold -e "python modified_combined_joystick.py" &
#sleep 1; xterm -iconic -hold -e "python sonar_test/sonar_get_min_dist.py"&
sleep 1; xterm -iconic -hold -e "./tcp-server 8080 8081"
