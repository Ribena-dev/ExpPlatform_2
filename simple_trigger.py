#simple script to test trigering jucier
import time
import subprocess
while True:

    subprocess.run('./on')
    time.sleep(100 )
    subprocess.run('./off')