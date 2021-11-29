import os
from datetime import datetime

sessionID = int(6)
print(sessionID)

today = datetime.today()
d1 = today.strftime("%y%m%d")
print(d1)
d1 = d1 + "_{:03d}".format(sessionID)

old_name = "/data/"+edffile
new_name = "/data/"+d1+".edf"
os.rename(old_name,new_name)