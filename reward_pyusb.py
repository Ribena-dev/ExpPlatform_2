#using pyusb lib to interface with a MCP2221 board

import usb.core

# devices = usb.core.find(find_all=True)
# for device in devices:
# 	print ("vendor:",device.idVendor,"Product:", device.idProduct)

device = usb.core.find(idVendor=0x04D8,idProduct=0x00DD)
if device:
	print("found MCP2221")
else:
	print("no MCP2221 found")
print("checking if kernel tracking")
if device.is_kernel_driver_active(1) is True:

	device.detach_kernel_driver(1)
	print("kernel driver detached")
if device.is_kernel_driver_active(0) is True:
	device.detach_kernel_driver(0)
	print("kernel driver detached")
if device.is_kernel_driver_active(2) is True:
	device.detach_kernel_driver(2)
	print("kernel driver detached")

print(device.is_kernel_driver_active(2),device.is_kernel_driver_active(1),device.is_kernel_driver_active(0))

device.set_configuration()
def reward(pin_number):
	device.set_configuration()
	cmd = [0x50]+[0x00] *63
	pin_offset = 7 + (pin_number * 4)
	cmd[pin_offset] = 0x01      # Set as output
	cmd[pin_offset + 1] = 0x01  # Set value high
	
	# Send command to device
	device.write(0x3, cmd)

	return 

reward(0)
reward(2)
