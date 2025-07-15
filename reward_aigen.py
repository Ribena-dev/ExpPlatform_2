import usb.core
import usb.util
import time

# Find MCP2221 device
device = usb.core.find(idVendor=0x04D8, idProduct=0x00DD)
if device:
    #print("found MCP2221")
#else:
    #print("no MCP2221 found")
    exit()

def check_kernel():
    """Detach kernel drivers properly"""
    #print("Checking kernel drivers...")
    
    # Try to detach from all interfaces
    for interface in range(3):
        try:
            if device.is_kernel_driver_active(interface):
                device.detach_kernel_driver(interface)
                #print(f"Kernel driver detached from interface {interface}")
                #print(f"No kernel driver on interface {interface}")
        except Exception as e:
            print("ah ho")
            #print(f"Interface {interface}: {e}")

def claim_interface():
    """Claim the HID interface"""
    try:
        # Try to claim interface 2 (HID interface)
        usb.util.claim_interface(device, 2)
        #print(" Interface 2 claimed successfully")
        return True
    except Exception as e:
        #print(f" Failed to claim interface: {e}")
        return False

def reset_device():
    """Reset the device"""
    try:
        device.reset()
        time.sleep(1)
        #print(" Device reset")
        return True
    except Exception as e:
        #print(f" Reset failed: {e}")
        return False

def sram_config():
    """Configure SRAM for GPIO using direct endpoints"""
    #print("Configuring SRAM for GPIO...")
    
    cmd = [0x60] + [0x00] * 63  # 64 bytes total
    cmd[7] = 0x01  # Alter GPIO configuration
    cmd[8] = 0x00  # GP0: GPIO output, LOW, GPIO mode
    
    #print(f"SRAM Command: {cmd[:12]}")
    
    try:
        # Write to endpoint 0x02 (OUT)
        result = device.write(0x02, cmd, timeout=1000)
        #print(f" SRAM write: {result} bytes")
        
        # Read from endpoint 0x82 (IN) 
        response = device.read(0x82, 64, timeout=1000)
        #print(f"SRAM Response: {list(response[:8])}")
        
        # Check if response indicates success
        if response[0] == 0x60:  # Command echo
            #print(" SRAM configuration successful")
            return True
        else:
            #print(f" SRAM configuration failed, response: {response[0]}")
            return False
            
    except Exception as e:
        #print(f" SRAM failed: {e}")
        return False

def gpio_write(pin, value):
    """Write to GPIO using direct endpoints"""
    #print(f"Setting GPIO{pin} to {'HIGH' if value else 'LOW'}")
    
    cmd = [0x50] + [0x00] * 63  # 64 bytes total
    cmd[2] = 0x01  # Alter GP0 output
    cmd[3] = 0x01 if value else 0x00  # GP0 value
    cmd[4] = 0x01  # Alter GP0 direction
    cmd[5] = 0x00  # GP0 as output
    
    #print(f"GPIO Command: {cmd[:10]}")
    
    try:
        # Write to endpoint 0x02 (OUT)
        result = device.write(0x02, cmd, timeout=1000)
        #print(f" GPIO write: {result} bytes")
        
        # Read from endpoint 0x82 (IN)
        response = device.read(0x82, 64, timeout=1000)
        #print(f"GPIO Response: {list(response[:8])}")
        
        # Check response
        if response[0] == 0x50:  # Command echo
            if response[1] == 0x00:  # Success
                #print(f" GPIO{pin} set successfully")
                return True
            else:
                #print(f" GPIO command failed with error: {response[1]}")
                return False
        else:
            #print(f" Unexpected response: {response[0]}")
            return False
            
    except Exception as e:
        #print(f" GPIO failed: {e}")
        return False

def gpio_read_status():
    """Read GPIO status"""
    #print("Reading GPIO status...")
    
    cmd = [0x51] + [0x00] * 63  # Get GPIO values command
    
    try:
        device.write(0x02, cmd, timeout=1000)
        response = device.read(0x82, 64, timeout=1000)
        #print(f"GPIO Status: {list(response[:15])}")
        
        if response[0] == 0x51:
            #print(" GPIO status read successful")
            return response
        else:
            #print(f" GPIO status read failed: {response[0]}")
            return None
            
    except Exception as e:
        #print(f"GPIO status read failed: {e}")
        return None

def setup_device_properly():
    """Complete device setup sequence"""
    #print("=== Setting up MCP2221 ===")
    
    # Step 1: Reset device
    if not reset_device():
        return False
    
    # Step 2: Check and detach kernel drivers
    check_kernel()
    
    # Step 3: Try to claim interface
    #if not claim_interface():
        #print(" Warning: Could not claim interface, continuing anyway...")
    
    return True

def test_led_complete():
    """Complete LED test"""
    #print("=== Complete LED Test ===")
    
    # Setup device
    if not setup_device_properly():
        #print("Device setup failed")
        return
    
    # Configure SRAM
    #print("\n--- Step 1: Configure SRAM ---")
    if not sram_config():
        #print("SRAM configuration failed")
        return
    
    time.sleep(0.2)
    
    # Read initial status
    #print("\n--- Step 2: Read initial status ---")
    gpio_read_status()
    
    # Test sequence
    #print("\n--- Step 3: LED OFF ---")
    gpio_write(0, 0)
    time.sleep(2)
    
    #print("\n--- Step 4: LED ON ---")
    gpio_write(0, 1) 
    time.sleep(2)
    
    #print("\n--- Step 5: LED OFF ---")
    gpio_write(0, 0)
    
    #print("\n--- Final status ---")
    gpio_read_status()

def simple_led_off():
    """Simple LED off function"""
    setup_device_properly()
    sram_config()
    time.sleep(0.1)
    gpio_write(0, 0)

# Run test
if __name__ == "__main__":
    test_led_complete()
