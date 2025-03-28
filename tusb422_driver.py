"""
TUSB422 - MicroPython Driver for TI's TUSB422 USB Type-C and PD Controller

Designed to work with the BQ25798 Battery Charger IC and Raspberry Pi Pico.
Handles USB-C PD negotiation in both sink (charging) and source (OTG) modes.
"""

import time
from micropython import const
from machine import I2C, Pin

# TUSB422 I2C Address
TUSB422_I2C_ADDR = const(0x20)  # Default I2C address

# TUSB422 Register Map
REG_VENDOR_ID_LSB = const(0x00)
REG_VENDOR_ID_MSB = const(0x01)
REG_PRODUCT_ID_LSB = const(0x02)
REG_PRODUCT_ID_MSB = const(0x03)
REG_DEVICE_ID_LSB = const(0x04)
REG_DEVICE_ID_MSB = const(0x05)
REG_USBTYPEC_REV_LSB = const(0x06)
REG_USBTYPEC_REV_MSB = const(0x07)
REG_USBPD_REV_LSB = const(0x08)
REG_USBPD_REV_MSB = const(0x09)
REG_PD_INTERFACE_REV = const(0x0A)
REG_ALERT = const(0x10)
REG_ALERT_MASK = const(0x12)
REG_POWER_STATUS = const(0x14)
REG_POWER_STATUS_MASK = const(0x15)
REG_PORT_STATUS = const(0x16)
REG_MODE_CFG = const(0x18)
REG_ROLE_CTRL = const(0x1A)
REG_FAULT_CTRL = const(0x1B)
REG_POWER_CTRL = const(0x1C)
REG_CC_STATUS = const(0x1D)
REG_COMMAND = const(0x23)
REG_MESSAGE_HEADER_INFO = const(0x2E)
REG_RX_SOURCE_CAPABILITIES = const(0x30)
REG_TX_SOURCE_CAPABILITIES = const(0x60)

# Commands
CMD_DISABLE_PD = const(0x0C)
CMD_ENABLE_PD = const(0x0D)
CMD_SOURCE_CAPABILITIES = const(0x10)
CMD_SINK_CAPABILITIES = const(0x11)
CMD_REQUEST = const(0x12)
CMD_SOFT_RESET = const(0x1F)

# Role Control Register Bits
ROLE_CTRL_DRP = const(0x30)      # Dual Role Power
ROLE_CTRL_SOURCE = const(0x20)   # Source Power Role
ROLE_CTRL_SINK = const(0x10)     # Sink Power Role
ROLE_CTRL_ACCESSORY = const(0x08)  # Accessory
ROLE_CTRL_DFP = const(0x04)      # Downstream Facing Port (host)
ROLE_CTRL_UFP = const(0x02)      # Upstream Facing Port (device)
ROLE_CTRL_DRP_DATA = const(0x06)  # Dual Role Data

# Power Status Register Bits
PWR_STATUS_VBUS_PRESENT = const(0x01)
PWR_STATUS_VBUS_DET = const(0x02)
PWR_STATUS_VCONN_PRESENT = const(0x04)
PWR_STATUS_VBUS_SRC = const(0x08)
PWR_STATUS_VBUS_SNK = const(0x10)
PWR_STATUS_VCONN_SRC = const(0x20)

# Power Control Register Bits
PWR_CTRL_ENABLE_VCONN = const(0x01)
PWR_CTRL_DISABLE_SRC_VBUS = const(0x02)
PWR_CTRL_ENABLE_SRC_VBUS = const(0x04)
PWR_CTRL_DISABLE_SNK_VBUS = const(0x08)
PWR_CTRL_ENABLE_SNK_VBUS = const(0x10)

# Alert Register Bits
ALERT_CC_STATUS = const(0x01)
ALERT_POWER_STATUS = const(0x02)
ALERT_PD_STATUS = const(0x04)
ALERT_RX_STATUS = const(0x08)

# CC Status Register Bits
CC_STATUS_LOOKING4CONNECTION = const(0x01)
CC_STATUS_CONNECT_RESULT = const(0x02)
CC_STATUS_TERM_RESULT_CC1 = const(0x30)  # Mask for CC1 termination
CC_STATUS_TERM_RESULT_CC2 = const(0xC0)  # Mask for CC2 termination

# Voltage definitions for PD profiles (in 50mV units)
VOLT_5V = const(100)     # 5V    (100 * 50mV)
VOLT_9V = const(180)     # 9V    (180 * 50mV)
VOLT_12V = const(240)    # 12V   (240 * 50mV)
VOLT_15V = const(300)    # 15V   (300 * 50mV)
VOLT_20V = const(400)    # 20V   (400 * 50mV)

# Current definitions for PD profiles (in 10mA units)
CURR_500MA = const(50)    # 500mA (50 * 10mA)
CURR_1A = const(100)      # 1A    (100 * 10mA)
CURR_1P5A = const(150)    # 1.5A  (150 * 10mA)
CURR_2A = const(200)      # 2A    (200 * 10mA)
CURR_3A = const(300)      # 3A    (300 * 10mA)
CURR_5A = const(500)      # 5A    (500 * 10mA)


class TUSB422:
    """Driver for TI TUSB422 USB Type-C and Power Delivery controller."""

    def __init__(self, i2c=None, i2c_addr=TUSB422_I2C_ADDR, sda_pin=0, scl_pin=1, int_pin=2):
        """Initialize the TUSB422 driver.
        
        Args:
            i2c: MicroPython I2C object
            i2c_addr: I2C device address
            sda_pin: SDA pin number (if i2c is None)
            scl_pin: SCL pin number (if i2c is None)
            int_pin: Interrupt pin number
        """
        # Initialize I2C if not provided
        if i2c is None:
            self.i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=100000)
        else:
            self.i2c = i2c
            
        self.i2c_addr = i2c_addr
        
        # Configure interrupt pin
        self.int_pin = Pin(int_pin, Pin.IN, Pin.PULL_UP)
        
        # Current state
        self.is_source_mode = False
        self.is_sink_mode = False
        self.is_connected = False
        self.negotiated_voltage = 0
        self.negotiated_current = 0
        
        # Check communication with the device
        try:
            vendor_id = self._read_reg16(REG_VENDOR_ID_LSB)
            product_id = self._read_reg16(REG_PRODUCT_ID_LSB)
            device_id = self._read_reg16(REG_DEVICE_ID_LSB)
            
            print(f"TUSB422 detected: Vendor ID=0x{vendor_id:04X}, Product ID=0x{product_id:04X}, Device ID=0x{device_id:04X}")
            
            # Initialize the device
            self._initialize()
            
        except Exception as e:
            print(f"Error communicating with TUSB422: {e}")
            raise OSError(f"TUSB422 communication failed: {e}")

    def _read_reg(self, reg):
        """Read a single register byte."""
        try:
            result = bytearray(1)
            self.i2c.readfrom_mem_into(self.i2c_addr, reg, result)
            return result[0]
        except Exception as e:
            print(f"Failed to read register 0x{reg:02X}: {e}")
            raise OSError(f"I2C read failed: {e}")

    def _write_reg(self, reg, value):
        """Write a single register byte."""
        try:
            self.i2c.writeto_mem(self.i2c_addr, reg, bytes([value]))
        except Exception as e:
            print(f"Failed to write register 0x{reg:02X}: {e}")
            raise OSError(f"I2C write failed: {e}")

    def _read_reg16(self, reg):
        """Read a 16-bit value (2 registers)."""
        try:
            result = bytearray(2)
            self.i2c.readfrom_mem_into(self.i2c_addr, reg, result)
            return result[0] | (result[1] << 8)
        except Exception as e:
            print(f"Failed to read register 0x{reg:02X}: {e}")
            raise OSError(f"I2C read failed: {e}")

    def _read_block(self, reg, length):
        """Read a block of registers."""
        try:
            result = bytearray(length)
            self.i2c.readfrom_mem_into(self.i2c_addr, reg, result)
            return result
        except Exception as e:
            print(f"Failed to read block from register 0x{reg:02X}: {e}")
            raise OSError(f"I2C block read failed: {e}")

    def _write_block(self, reg, data):
        """Write a block of registers."""
        try:
            self.i2c.writeto_mem(self.i2c_addr, reg, data)
        except Exception as e:
            print(f"Failed to write block to register 0x{reg:02X}: {e}")
            raise OSError(f"I2C block write failed: {e}")

    def _initialize(self):
        """Initialize the TUSB422 controller."""
        # Reset the device
        self.reset()
        time.sleep_ms(50)
        
        # Set default alert mask
        self._write_reg(REG_ALERT_MASK, 0x0F)  # Enable all alerts
        
        # Clear any existing alerts
        self._read_reg(REG_ALERT)
        
        # Start in sink mode by default (for charging)
        self.set_sink_mode()
        
        print("TUSB422 initialized successfully")
    
    def reset(self):
        """Perform a soft reset of the TUSB422."""
        try:
            self._write_reg(REG_COMMAND, CMD_SOFT_RESET)
            time.sleep_ms(50)  # Wait for reset to complete
            return True
        except Exception as e:
            print(f"Failed to reset TUSB422: {e}")
            return False
            
    def set_sink_mode(self):
        """Configure the TUSB422 for sink mode (receiving power)."""
        try:
            # Set as sink and UFP (device)
            self._write_reg(REG_ROLE_CTRL, ROLE_CTRL_SINK | ROLE_CTRL_UFP)
            
            # Enable sink VBUS
            power_ctrl = self._read_reg(REG_POWER_CTRL)
            power_ctrl &= ~PWR_CTRL_DISABLE_SNK_VBUS  # Clear disable flag
            power_ctrl |= PWR_CTRL_ENABLE_SNK_VBUS    # Set enable flag
            self._write_reg(REG_POWER_CTRL, power_ctrl)
            
            # Enable PD
            self._write_reg(REG_COMMAND, CMD_ENABLE_PD)
            
            self.is_source_mode = False
            self.is_sink_mode = True
            
            print("TUSB422 configured for sink mode")
            return True
        except Exception as e:
            print(f"Failed to set sink mode: {e}")
            return False
    
    def set_source_mode(self):
        """Configure the TUSB422 for source mode (providing power)."""
        try:
            # Set as source and DFP (host)
            self._write_reg(REG_ROLE_CTRL, ROLE_CTRL_SOURCE | ROLE_CTRL_DFP)
            
            # Enable source VBUS
            power_ctrl = self._read_reg(REG_POWER_CTRL)
            power_ctrl &= ~PWR_CTRL_DISABLE_SRC_VBUS  # Clear disable flag
            power_ctrl |= PWR_CTRL_ENABLE_SRC_VBUS    # Set enable flag
            self._write_reg(REG_POWER_CTRL, power_ctrl)
            
            # Enable PD
            self._write_reg(REG_COMMAND, CMD_ENABLE_PD)
            
            self.is_source_mode = True
            self.is_sink_mode = False
            
            print("TUSB422 configured for source mode")
            return True
        except Exception as e:
            print(f"Failed to set source mode: {e}")
            return False
    
    def set_dual_role_mode(self):
        """Configure the TUSB422 for dual role mode (can be source or sink)."""
        try:
            # Set as dual role power and data
            self._write_reg(REG_ROLE_CTRL, ROLE_CTRL_DRP | ROLE_CTRL_DRP_DATA)
            
            # Enable PD
            self._write_reg(REG_COMMAND, CMD_ENABLE_PD)
            
            self.is_source_mode = False
            self.is_sink_mode = False
            
            print("TUSB422 configured for dual role mode")
            return True
        except Exception as e:
            print(f"Failed to set dual role mode: {e}")
            return False
    
    def advertise_sink_capabilities(self):
        """Configure and advertise sink capabilities (what power we can accept)."""
        try:
            # Define sink capabilities message (PDO)
            # Format: 4 bytes per PDO, supporting several voltages
            
            # PDO1: 5V, 3A (Fixed Supply)
            pdo1 = (1 << 30) | (1 << 29) | (VOLT_5V << 10) | CURR_3A
            
            # PDO2: 9V, 3A (Fixed Supply)
            pdo2 = (1 << 30) | (1 << 29) | (VOLT_9V << 10) | CURR_3A
            
            # PDO3: 15V, 3A (Fixed Supply)
            pdo3 = (1 << 30) | (1 << 29) | (VOLT_15V << 10) | CURR_3A
            
            # PDO4: 20V, 2A (Fixed Supply)
            pdo4 = (1 << 30) | (1 << 29) | (VOLT_20V << 10) | CURR_2A
            
            # Create PDO byte array (4 PDOs, 4 bytes each = 16 bytes)
            pdos = bytearray(16)
            
            # Convert PDOs to bytes
            for i in range(4):
                idx = i * 4
                if i == 0:
                    pdo = pdo1
                elif i == 1:
                    pdo = pdo2
                elif i == 2:
                    pdo = pdo3
                else:
                    pdo = pdo4
                
                pdos[idx] = pdo & 0xFF
                pdos[idx+1] = (pdo >> 8) & 0xFF
                pdos[idx+2] = (pdo >> 16) & 0xFF
                pdos[idx+3] = (pdo >> 24) & 0xFF
            
            # Write PDOs to sink capabilities register
            self._write_block(REG_RX_SOURCE_CAPABILITIES, pdos)
            
            # Send sink capabilities
            self._write_reg(REG_COMMAND, CMD_SINK_CAPABILITIES)
            
            print("Sink capabilities advertised")
            return True
        except Exception as e:
            print(f"Failed to advertise sink capabilities: {e}")
            return False
    
    def advertise_source_capabilities(self):
        """Configure and advertise source capabilities (what power we can provide)."""
        try:
            # Define source capabilities message (PDO)
            # Format: 4 bytes per PDO, supporting several voltages
            
            # PDO1: 5V, 3A (Fixed Supply)
            pdo1 = (1 << 30) | (VOLT_5V << 10) | CURR_3A
            
            # PDO2: 9V, 2.5A (Fixed Supply)
            pdo2 = (1 << 30) | (VOLT_9V << 10) | (CURR_2A + CURR_500MA)
            
            # PDO3: 15V, 1.5A (Fixed Supply)
            pdo3 = (1 << 30) | (VOLT_15V << 10) | CURR_1P5A
            
            # Create PDO byte array (3 PDOs, 4 bytes each = 12 bytes)
            pdos = bytearray(12)
            
            # Convert PDOs to bytes
            for i in range(3):
                idx = i * 4
                if i == 0:
                    pdo = pdo1
                elif i == 1:
                    pdo = pdo2
                else:
                    pdo = pdo3
                
                pdos[idx] = pdo & 0xFF
                pdos[idx+1] = (pdo >> 8) & 0xFF
                pdos[idx+2] = (pdo >> 16) & 0xFF
                pdos[idx+3] = (pdo >> 24) & 0xFF
            
            # Write PDOs to source capabilities register
            self._write_block(REG_TX_SOURCE_CAPABILITIES, pdos)
            
            # Send source capabilities
            self._write_reg(REG_COMMAND, CMD_SOURCE_CAPABILITIES)
            
            print("Source capabilities advertised")
            return True
        except Exception as e:
            print(f"Failed to advertise source capabilities: {e}")
            return False
    
    def request_power(self, voltage, current):
        """Request specific power from the source.
        
        Args:
            voltage: Voltage in volts (supports 5, 9, 15, 20)
            current: Current in amps
        
        Returns:
            bool: Success or failure
        """
        try:
            # Convert to PD units
            volt_unit = int(voltage * 20)  # 50mV units
            curr_unit = int(current * 100)  # 10mA units
            
            # Build request message
            # Format: [Object Position] [GiveBack] [CapabilityMismatch] [0] [Operating Current] [Maximum Current]
            
            # Determine object position based on voltage
            obj_pos = 0
            if abs(voltage - 5.0) < 0.1:
                obj_pos = 1
            elif abs(voltage - 9.0) < 0.1:
                obj_pos = 2
            elif abs(voltage - 15.0) < 0.1:
                obj_pos = 3
            elif abs(voltage - 20.0) < 0.1:
                obj_pos = 4
            else:
                print(f"Unsupported voltage: {voltage}V")
                return False
            
            # Create request message
            req_msg = bytearray(4)
            
            # First byte: [Object Position (bits 0-2)] [GiveBack (bit 3)] [CapabilityMismatch (bit 4)]
            req_msg[0] = obj_pos & 0x07
            
            # Operating current (10mA units)
            req_msg[1] = curr_unit & 0xFF
            req_msg[2] = (curr_unit >> 8) & 0x03
            
            # Maximum current (same as operating for simplicity)
            req_msg[2] |= (curr_unit & 0x03) << 2
            req_msg[3] = (curr_unit >> 2) & 0xFF
            
            # Write request message to register
            # Note: The actual register may vary based on TUSB422 implementation
            # This is a simplified example
            self._write_block(0x40, req_msg)  # Request message register
            
            # Send request command
            self._write_reg(REG_COMMAND, CMD_REQUEST)
            
            # Save the requested values
            self.negotiated_voltage = voltage
            self.negotiated_current = current
            
            print(f"Requested power: {voltage}V, {current}A")
            return True
        except Exception as e:
            print(f"Failed to request power: {e}")
            return False
    
    def is_vbus_present(self):
        """Check if VBUS is present."""
        power_status = self._read_reg(REG_POWER_STATUS)
        return bool(power_status & PWR_STATUS_VBUS_PRESENT)
    
    def get_connection_status(self):
        """Get the current connection status.
        
        Returns:
            tuple: (connected, cc_pin)
                connected (bool): True if connected, False otherwise
                cc_pin (int): 1 for CC1, 2 for CC2, 0 if not connected
        """
        cc_status = self._read_reg(REG_CC_STATUS)
        
        # Bit 1 indicates successful connection
        connected = bool(cc_status & CC_STATUS_CONNECT_RESULT)
        
        # Check which CC pin is active
        cc_pin = 0
        if connected:
            cc1_term = (cc_status & CC_STATUS_TERM_RESULT_CC1) >> 4
            cc2_term = (cc_status & CC_STATUS_TERM_RESULT_CC2) >> 6
            
            if cc1_term > 0:
                cc_pin = 1
            elif cc2_term > 0:
                cc_pin = 2
        
        self.is_connected = connected
        return (connected, cc_pin)
    
    def get_current_role(self):
        """Get the current power role.
        
        Returns:
            str: 'source', 'sink', or 'disconnected'
        """
        power_status = self._read_reg(REG_POWER_STATUS)
        
        if power_status & PWR_STATUS_VBUS_SRC:
            return 'source'
        elif power_status & PWR_STATUS_VBUS_SNK:
            return 'sink'
        else:
            return 'disconnected'
    
    def handle_interrupt(self):
        """Handle TUSB422 interrupt.
        
        Returns:
            bool: True if interrupt was handled, False if no interrupt
        """
        # Check if interrupt pin is low (active)
        if self.int_pin.value() == 0:
            # Read alert register to determine the cause
            alert = self._read_reg(REG_ALERT)
            
            if alert & ALERT_CC_STATUS:
                # CC status changed - connection/disconnection
                connected, cc_pin = self.get_connection_status()
                if connected:
                    print(f"USB-C connected on CC{cc_pin}")
                    
                    # If we're in sink mode, request power
                    role = self.get_current_role()
                    if role == 'sink':
                        # After connection, advertise capabilities and request power
                        self.advertise_sink_capabilities()
                        # Request 15V at 3A (adjust based on your needs)
                        self.request_power(15.0, 3.0)
                    elif role == 'source':
                        # After connection, advertise source capabilities
                        self.advertise_source_capabilities()
                else:
                    print("USB-C disconnected")
            
            if alert & ALERT_POWER_STATUS:
                # Power status changed
                power_status = self._read_reg(REG_POWER_STATUS)
                
                if power_status & PWR_STATUS_VBUS_PRESENT:
                    print("VBUS present")
                else:
                    print("VBUS not present")
                    
                role = self.get_current_role()
                print(f"Current role: {role}")
            
            if alert & ALERT_PD_STATUS:
                # PD message status changed
                print("PD status changed")
                
                # This is where you would handle PD negotiation results
                # Check if our voltage/current request was accepted
                # This would require reading additional PD status registers
            
            if alert & ALERT_RX_STATUS:
                # Received a PD message
                print("Received PD message")
                
                # This is where you would handle incoming PD messages
                # Like reading source capabilities from a power source
            
            return True
        
        return False
    
    def poll(self):
        """Poll the TUSB422 for status changes.
        
        This can be called regularly in your main loop if not using interrupts.
        
        Returns:
            bool: True if any status changed, False otherwise
        """
        return self.handle_interrupt()
