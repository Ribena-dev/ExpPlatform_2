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

def scram():

	sram_cmd = [0x60] +[0x00]*63
	sram_cmd[7] = 0x01
	for i in range(8,12):

		sram_cmd[i] = 0b00000000

	device.write(0x02, sram_cmd)
	print(sram_cmd)
#device.set_configuration()
def gpio_write(dev, pin, value):
	cmd = [0x50] + [0x00] * 63
	cmd[2] = 0x01
	cmd[3]= 0x00
	cmd[4] = 0x01
	cmd[5] = 0x00
	print(cmd)
	# dev.write(0x02,cmd)
	dev.ctrl_transfer(0x21,0x09,0x03550,0,cmd)
	return 

def check_kernel():
	print("checking if kernel tracking")
	for interface in range(3):
		if device.is_kernel_driver_active(interface) is True:
			device.detach_kernel_driver(interface)
		print("no kernel attached")
	return 
def gpio_read():
	responds =device.read(0x02,64,1000)
	print(responds)
check_kernel()
#scram()
gpio_write(device,0,0)
gpio_read()


#reward(2)
#time.sleep(5)
