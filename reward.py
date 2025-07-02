# testing script designed to send high or low signals via serial port

import EasyMCP2221
from time import sleep 
def send_reward(send):
    mcp = EasyMCP2221.Device()
    print(mcp)
    mcp.set_pin_function(gp0 = "GPIO_OUT")
    if send == True:
        mcp.GPIO_write(gp2 = True)
        sleep(1.0)
    if send == False:
        mcp.GPIO_write(gp2= False)

while True:
    send_reward(True)
    print("hello world")
    send_reward(False)
    sleep(1.0)
