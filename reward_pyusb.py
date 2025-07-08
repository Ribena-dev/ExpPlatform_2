#using pyusb lib to interface with a MCP2221 board

import usb.core

# devices = usb.core.find(find_all=True)
# for device in devices:
# 	print ("vendor:",device.idVendor,"Product:", device.idProduct)

dev = usb.core.find(idVendor=0x04D8,idProduct=0x00DD)
if dev:
	print("found MCP2221")
else:
	print("no MCP2221 found")

try:
        dev.set_configuration()  # Try without detaching first
        print("Device available, no driver detachment needed")
except usb.core.USBError as e:
        if "busy" in str(e).lower():
            print("Device busy, will detach driver...")
            # Only detach if really necessary
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
            dev.set_configuration()
def reward(pin_number):
	dev.set_configuration()
	cmd = [0x50]+[0x00] *63
	pin_offset = 7 + (pin_number * 4)
	cmd[pin_offset] = 0x01      # Set as output
	cmd[pin_offset + 1] = 0x01  # Set value high
	
	# Send command to device
	dev.write(0x01, cmd)

	return 

reward(0)
reward(2)