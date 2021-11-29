import os
from datetime import datetime

sessionID = 6

today = datetime.today()
d1 = today.strftime("%y%m%d")
d1 = d1 + "_{:03d}".format(sessionID)

edffile = 'test.edf'

old_name = "./data/"+edffile
new_name = "./data/"+d1+".edf"
os.rename(old_name,new_name)