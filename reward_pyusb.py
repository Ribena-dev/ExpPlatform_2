#using pyusb lib to interface with a MCP2221 board

import usb.core

devices = usb.core.find(find_all=True)
for device in devices:
	print ("vendor:",device.idVendor,"Product:", device.idProduct)

dev = usb.core.find(idVendor=0x04DB,idProduct=0x00DD)
if dev:
	print("found MCP2221")
else:
	print("no MCP2221 found")
