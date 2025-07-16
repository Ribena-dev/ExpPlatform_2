#simple script to test trigering jucier
import time
import subprocess
while True:

    subprocess.call('./on')
    time.sleep(10)
    subprocess.call('./off')
    time.sleep(10)