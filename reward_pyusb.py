#using pyusb lib to interface with a MCP2221 board

import usb.core
import time
# devices = usb.core.find(find_all=True)
# for device in devices:
# 	print ("vendor:",device.idVendor,"Product:", device.idProduct)

device = usb.core.find(idVendor=0x04D8,idProduct=0x00DD)
if device:
	print("found MCP2221")
else:
	print("no MCP2221 found")


# device.set_configuration()
def gpio_write(dev, pin, value):

    cmd = [0x50] + [0x00] * 63
    cmd[7 + pin * 4] = 0x01      # Output mode
    cmd[7 + pin * 4 + 1] = value # Value
    dev.write(0x03,cmd)
    # dev.ctrl_transfer(0x21, 0x09, 0x0350, 0, cmd)
    #print(f"GPIO{pin} = {'HIGH' if value else 'LOW'}")

def reward(pin_number):
	print("checking if kernel tracking")
	if device.is_kernel_driver_active(1) is True:
		device.detach_kernel_driver(1)
		print("kernel driver detached")
	if device.is_kernel_driver_active(0) is True:
		device.detach_kernel_driver(0)
		print("no kernel attached")
	if device.is_kernel_driver_active(2) is True:
		device.detach_kernel_driver(2)
		print("no kernel attached")
	# device.set_configuration()
	# cmd = [0x50]+[0x00] *63
	# pin_offset = 7 + (pin_number * 4)
	# cmd[pin_offset] = 0x01      # Set as output
	# cmd[pin_offset + 1] = 0x00  # Set value high
	
	# # Send command to device
	# device.write(0x3, cmd)
	# time.sleep(10)
	#cmd[pin_offset + 1] = 0x00
	gpio_write(device,0,1)
	return 

reward(0)
reward(2)
