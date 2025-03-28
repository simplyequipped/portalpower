"""
BQ25798 - MicroPython Driver for TI's BQ25798 Battery Charger IC (Pi Pico)

For 4S LiFePO4 battery with solar, DC, and USB-C PD charging sources.
Automatically detects and configures for different input sources based on voltage.

Battery Configuration:
- 4S LiFePO4 Battery 
- Max Charge Voltage: 14.6V (3.65V per cell)
- Nominal Voltage: 13.2V (3.3V per cell)
- Min Voltage: 10.0V (2.5V per cell)

Charging Source Configuration:
- Solar: >15V input, MPPT enabled, 5.0A charge current, 3.3A input limit
- DC: <15V input, 2.0A charge current, 2.5A input limit
- USB-C PD: 15V trigger voltage, 3.0A charge current, 3.0A input limit
"""

import time
from micropython import const
from machine import I2C, Pin

# Battery chemistry types
LIFEPO4 = const(0)
LI_ION = const(1)
NIMH = const(2)

# Charge states
NOT_CHARGING = const(0)
TRICKLE_CHARGE = const(1)
PRECHARGE = const(2)
FAST_CHARGE = const(3)
TAPER_CHARGE = const(4)
RESERVED = const(5)
TOP_OFF = const(6)
CHARGE_DONE = const(7)

# ADC modes
ADC_ONE_SHOT = const(0)
ADC_CONTINUOUS = const(1)

# Fault types
FAULT_IBAT_REGULATION = const(0)
FAULT_VBUS_OVP = const(1)
FAULT_VBAT_OVP = const(2)
FAULT_IBUS_OCP = const(3)
FAULT_IBAT_OCP = const(4)
FAULT_CONVERTER_OCP = const(5)
FAULT_VAC2_OVP = const(6)
FAULT_VAC1_OVP = const(7)
FAULT_VSYS_SHORT = const(8)
FAULT_VSYS_OVP = const(9)
FAULT_OTG_OVP = const(10)
FAULT_OTG_UVP = const(11)
FAULT_TDIE_SHUTDOWN = const(12)

# MPPT percentages of open circuit voltage
VOC_MPPT_PCT_56_25 = const(0)  # 56.25% of VOC
VOC_MPPT_PCT_62_50 = const(1)  # 62.50% of VOC
VOC_MPPT_PCT_68_75 = const(2)  # 68.75% of VOC
VOC_MPPT_PCT_75_00 = const(3)  # 75.00% of VOC
VOC_MPPT_PCT_81_25 = const(4)  # 81.25% of VOC
VOC_MPPT_PCT_87_50 = const(5)  # 87.50% of VOC (default)
VOC_MPPT_PCT_93_75 = const(6)  # 93.75% of VOC
VOC_MPPT_PCT_100_0 = const(7)  # 100.0% of VOC

# ADC sample speeds
ADC_RES_15_BIT = const(0)  # 15-bit effective resolution
ADC_RES_14_BIT = const(1)  # 14-bit effective resolution
ADC_RES_13_BIT = const(2)  # 13-bit effective resolution
ADC_RES_12_BIT = const(3)  # 12-bit effective resolution (not recommended)


# BQ25798 Register definitions
class Regs:
    # System config registers
    MIN_SYS_VOLTAGE = const(0x00)
    CHARGE_VOLTAGE = const(0x01)
    CHARGE_CURRENT = const(0x03)
    INPUT_VOLTAGE_LIMIT = const(0x05)
    INPUT_CURRENT_LIMIT = const(0x06)
    PRECHARGE_CONTROL = const(0x08)
    TERMINATION_CONTROL = const(0x09)
    RECHARGE_CONTROL = const(0x0A)
    OTG_VOLTAGE = const(0x0B)
    OTG_CURRENT = const(0x0D)
    TIMER_CONTROL = const(0x0E)
    
    # Charger control registers
    CHARGER_CTRL_0 = const(0x0F)
    CHARGER_CTRL_1 = const(0x10)
    CHARGER_CTRL_2 = const(0x11)
    CHARGER_CTRL_3 = const(0x12)
    CHARGER_CTRL_4 = const(0x13)
    CHARGER_CTRL_5 = const(0x14)
    MPPT_CONTROL = const(0x15)
    TEMPERATURE_CONTROL = const(0x16)
    NTC_CONTROL_0 = const(0x17)
    NTC_CONTROL_1 = const(0x18)
    ICO_CURRENT_LIMIT = const(0x19)
    
    # Status registers
    CHARGER_STATUS_0 = const(0x1B)
    CHARGER_STATUS_1 = const(0x1C)
    CHARGER_STATUS_2 = const(0x1D)
    CHARGER_STATUS_3 = const(0x1E)
    CHARGER_STATUS_4 = const(0x1F)
    FAULT_STATUS_0 = const(0x20)
    FAULT_STATUS_1 = const(0x21)
    
    # Flag registers
    CHARGER_FLAG_0 = const(0x22)
    CHARGER_FLAG_1 = const(0x23)
    CHARGER_FLAG_2 = const(0x24)
    CHARGER_FLAG_3 = const(0x25)
    FAULT_FLAG_0 = const(0x26)
    FAULT_FLAG_1 = const(0x27)
    
    # Mask registers
    CHARGER_MASK_0 = const(0x28)
    CHARGER_MASK_1 = const(0x29)
    CHARGER_MASK_2 = const(0x2A)
    CHARGER_MASK_3 = const(0x2B)
    FAULT_MASK_0 = const(0x2C)
    FAULT_MASK_1 = const(0x2D)
    
    # ADC registers
    ADC_CONTROL = const(0x2E)
    ADC_FUNCTION_DIS_0 = const(0x2F)
    ADC_FUNCTION_DIS_1 = const(0x30)
    ADC_IBUS = const(0x31)
    ADC_IBAT = const(0x33)
    ADC_VBUS = const(0x35)
    ADC_VAC1 = const(0x37)
    ADC_VAC2 = const(0x39)
    ADC_VBAT = const(0x3B)
    ADC_VSYS = const(0x3D)
    ADC_TS = const(0x3F)
    ADC_TDIE = const(0x41)
    ADC_DP = const(0x43)
    ADC_DM = const(0x45)
    
    # Miscellaneous registers
    DPDM_DRIVER = const(0x47)
    PART_INFO = const(0x48)


class BQ25798:
    """Driver for the TI BQ25798 Battery Charger IC for MicroPython on Pi Pico."""
    
    # Default I2C address for the BQ25798
    DEFAULT_I2C_ADDRESS = const(0x6B)
    
    # Voltage threshold to determine solar vs DC input
    SOLAR_VOLTAGE_THRESHOLD = const(15.0)  # Volts
    
    # USB-C PD voltage trigger value
    USB_PD_VOLTAGE_TRIGGER = const(15.0)  # Volts
    
    # LiFePO4 4S battery parameters
    LIFEPO4_4S_MAX_VOLTAGE = const(14.6)  # 4 * 3.65V
    LIFEPO4_4S_NOMINAL_VOLTAGE = const(13.2)  # 4 * 3.3V
    LIFEPO4_4S_MIN_VOLTAGE = const(10.0)  # 4 * 2.5V
    
    # Configuration for DC charging (lower power)
    DC_CHARGE_CURRENT = const(2.0)  # Amps
    DC_INPUT_CURRENT_LIMIT = const(2.5)  # Amps
    
    # Configuration for Solar charging (higher power)
    SOLAR_CHARGE_CURRENT = const(5.0)  # Amps
    SOLAR_INPUT_CURRENT_LIMIT = const(3.3)  # Amps
    
    # USB-C PD configuration
    USB_PD_CHARGE_CURRENT = const(3.0)  # Amps
    USB_PD_INPUT_CURRENT_LIMIT = const(3.0)  # Amps
    
    # Battery SoC curve reference points for LiFePO4
    LIFEPO4_SOC_CURVE = [
        (2.5, 0),    # (voltage_per_cell, percentage)
        (2.8, 10),
        (3.0, 20),
        (3.2, 30),
        (3.25, 40),
        (3.3, 50),
        (3.35, 60),
        (3.4, 70),
        (3.45, 80),
        (3.5, 90),
        (3.6, 100)
    ]

    def __init__(self, i2c=None, i2c_address=DEFAULT_I2C_ADDRESS, sda_pin=0, scl_pin=1, update_interval=5000):
        """Initialize the BQ25798 driver.
        
        Args:
            i2c: MicroPython I2C object
            i2c_address: I2C device address
            sda_pin: SDA pin number (if i2c is None, will create a new I2C instance)
            scl_pin: SCL pin number (if i2c is None, will create a new I2C instance)
            update_interval: How often to update status in milliseconds
        """
        # Initialize I2C if not provided
        if i2c is None:
            self.i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=100000)
        else:
            self.i2c = i2c
            
        self.i2c_address = i2c_address
        self.update_interval = update_interval
        
        # Battery configuration for 4S LiFePO4
        self.battery_cells = 4
        self.battery_chemistry = LIFEPO4
        self.battery_capacity_ah = 100.0  # Default to 100Ah, change as needed
        
        # Status variables
        self._charge_state = NOT_CHARGING
        self._adc_mode = ADC_ONE_SHOT
        self._last_update_ms = 0
        self._input_source = "NONE"  # "NONE", "DC", "SOLAR", "USB_PD"
        
        # ADC readings
        self.vbus = 0.0      # Input voltage (V)
        self.ibus = 0.0      # Input current (A)
        self.vbat = 0.0      # Battery voltage (V)
        self.ibat = 0.0      # Battery current (A)
        self.vsys = 0.0      # System voltage (V)
        self.tdie = 0.0      # Die temperature (°C)
        self.vac1 = 0.0      # Input 1 voltage (V)
        self.vac2 = 0.0      # Input 2 voltage (V)
        self.ts = 0.0        # Temperature sensor reading (%)
        self.dp = 0.0        # USB D+ voltage (V)
        self.dm = 0.0        # USB D- voltage (V)
        
        # Fault status
        self.faults = [False] * 13  # One for each fault type
        
        # Verify communication with the device
        try:
            device_id = self._read_register(Regs.PART_INFO, 8)
            # Extract part number from bits 5-3
            part_number = (device_id >> 3) & 0x07
            # Extract device revision from bits 2-0
            device_rev = device_id & 0x07
            
            if part_number != 3:  # 3 = BQ25798
                print("Warning: Unexpected part number:", part_number, "expected 3 (BQ25798)")
            
            print(f"Connected to BQ25798 (rev {device_rev}) at address 0x{i2c_address:02X}")
        except Exception as e:
            print(f"Failed to communicate with BQ25798: {e}")
            raise OSError(f"Communication with BQ25798 failed: {e}")
        
        # Initialize the device
        self._initialize_device()
        
        # Update status immediately
        self.update_all()

    #######################
    # I2C Helper Methods  #
    #######################
    
    def _read_register(self, reg, length=8, start_bit=None, end_bit=None):
        """Read from a register.
        
        Args:
            reg (int): Register address
            length (int): Register length in bits (8 or 16)
            start_bit (int, optional): Start bit position
            end_bit (int, optional): End bit position
            
        Returns:
            int: Register value
        """
        if length not in (8, 16):
            raise ValueError(f"Register length must be 8 or 16, got {length}")
            
        try:
            if length == 8:
                # Read a single byte
                result = self.i2c.readfrom_mem(self.i2c_address, reg, 1)
                value = result[0]
            else:  # length == 16
                # Read two bytes (little-endian)
                result = self.i2c.readfrom_mem(self.i2c_address, reg, 2)
                value = result[0] | (result[1] << 8)
                
            # Extract bits if requested
            if start_bit is not None:
                if end_bit is None:
                    end_bit = start_bit
                
                # Create mask for the specified bits
                mask = ((1 << (end_bit - start_bit + 1)) - 1) << start_bit
                value = (value & mask) >> start_bit
            
            return value
        except Exception as e:
            print(f"Failed to read register 0x{reg:02X}: {e}")
            raise OSError(f"I2C read failed: {e}")

    def _write_register(self, reg, value, length=8, start_bit=None, end_bit=None):
        """Write to a register.
        
        Args:
            reg (int): Register address
            value (int): Value to write
            length (int): Register length in bits (8 or 16)
            start_bit (int, optional): Start bit position
            end_bit (int, optional): End bit position
        """
        if length not in (8, 16):
            raise ValueError(f"Register length must be 8 or 16, got {length}")
            
        try:
            # If only setting specific bits, read-modify-write
            if start_bit is not None:
                if end_bit is None:
                    end_bit = start_bit
                
                # Read current value
                current_value = self._read_register(reg, length)
                
                # Create mask for the specified bits
                mask = ((1 << (end_bit - start_bit + 1)) - 1) << start_bit
                
                # Clear the bits to be modified and set the new value
                new_value = (current_value & ~mask) | ((value << start_bit) & mask)
                value = new_value
            
            # Write the value
            if length == 8:
                # Write a single byte
                self.i2c.writeto_mem(self.i2c_address, reg, bytes([value]))
            else:  # length == 16
                # Write two bytes (little-endian)
                self.i2c.writeto_mem(self.i2c_address, reg, bytes([value & 0xFF, (value >> 8) & 0xFF]))
        except Exception as e:
            print(f"Failed to write register 0x{reg:02X}: {e}")
            raise OSError(f"I2C write failed: {e}")

    def _initialize_device(self):
        """Initialize the BQ25798 with default settings for 4S LiFePO4 battery."""
        print("Initializing BQ25798...")
        
        # Ensure ship mode is disabled
        if self.is_ship_mode_enabled():
            self.disable_ship_mode()
            print("Disabled ship mode")
        
        # Set charge voltage for 4S LiFePO4
        self.set_charge_voltage(self.LIFEPO4_4S_MAX_VOLTAGE)
        
        # Set safety timers
        self.set_precharge_timeout(30)  # 30 minutes
        self.set_fast_charge_timeout(720)  # 12 hours
        
        # Configure the ADC
        self.set_adc_mode(ADC_CONTINUOUS)  # We'll keep ADC on to monitor input sources
        self.enable_battery_current_sense()
        
        # Configure for solar MPPT when using solar input
        self.configure_mppt(VOC_MPPT_PCT_87_50)  # 87.5% of VOC
        
        # Set default USB OTG parameters
        self.set_otg_voltage(5.0)  # 5V
        self.set_otg_current(1.0)  # 1A
        
        # Configure minimum system voltage to prevent battery over-discharge
        self.set_minimum_system_voltage(self.LIFEPO4_4S_MIN_VOLTAGE)
        
        # Set termination parameters
        self.set_termination_current(0.2)  # 200mA termination current
        self.enable_termination()
        
        # Set recharge threshold
        self.set_recharge_threshold(0.2)  # 200mV recharge threshold
        
        # Enable charging
        self.enable_charging()
        
        print("BQ25798 initialization complete")

    #######################
    # Status Updates      #
    #######################
    
    def update_all(self):
        """Update all status information and automatically configure based on input source."""
        current_time = time.ticks_ms()
        
        # Only update if the update interval has passed
        if time.ticks_diff(current_time, self._last_update_ms) >= self.update_interval:
            self._last_update_ms = current_time
            
            # Update ADC readings
            self.update_adc_readings()
            
            # Determine and configure for input source
            self._detect_and_configure_input_source()
            
            # Update status
            self.update_charge_state()
            self.update_fault_status()
            
            # Print status (only for debugging, can be removed in final version)
            print(f"Source: {self._input_source}, VBUS: {self.vbus:.1f}V, VBAT: {self.vbat:.1f}V, "
                  f"IBAT: {self.ibat:.2f}A, State: {self._charge_state}")

    def update_adc_readings(self):
        """Update all ADC readings."""
        # Enable ADC for one-shot mode
        one_shot_mode = (self._adc_mode == ADC_ONE_SHOT)
        if one_shot_mode:
            self.enable_adc()
            # Allow time for readings to stabilize
            time.sleep_ms(100)
        
        try:
            # Read ADC registers
            self.vbus = self._read_register(Regs.ADC_VBUS, 16) / 1000.0  # mV to V
            self.vbat = self._read_register(Regs.ADC_VBAT, 16) / 1000.0  # mV to V
            self.vsys = self._read_register(Regs.ADC_VSYS, 16) / 1000.0  # mV to V
            self.vac1 = self._read_register(Regs.ADC_VAC1, 16) / 1000.0  # mV to V
            self.vac2 = self._read_register(Regs.ADC_VAC2, 16) / 1000.0  # mV to V
            self.ts = self._read_register(Regs.ADC_TS, 16) * 0.0976563  # 0.0976563% per bit
            self.dp = self._read_register(Regs.ADC_DP, 16) / 1000.0     # mV to V
            self.dm = self._read_register(Regs.ADC_DM, 16) / 1000.0     # mV to V
            
            # Die temperature (2's complement)
            tdie_raw = self._read_register(Regs.ADC_TDIE, 16)
            if tdie_raw & 0x8000:  # Negative temperature
                self.tdie = -((~tdie_raw & 0xFFFF) + 1) * 0.5  # 0.5°C per bit
            else:
                self.tdie = tdie_raw * 0.5  # 0.5°C per bit
            
            # Current readings (2's complement)
            ibus_raw = self._read_register(Regs.ADC_IBUS, 16)
            if ibus_raw & 0x8000:  # Negative current
                self.ibus = -((~ibus_raw & 0xFFFF) + 1) / 1000.0  # mA to A
            else:
                self.ibus = ibus_raw / 1000.0  # mA to A
            
            ibat_raw = self._read_register(Regs.ADC_IBAT, 16)
            if ibat_raw & 0x8000:  # Negative current
                self.ibat = -((~ibat_raw & 0xFFFF) + 1) / 1000.0  # mA to A
            else:
                self.ibat = ibat_raw / 1000.0  # mA to A
        
        finally:
            # Disable ADC if we were in one-shot mode
            if one_shot_mode:
                self.disable_adc()

    def _detect_and_configure_input_source(self):
        """Detect the input source and configure the charger appropriately."""
        old_source = self._input_source
        
        # Check if a USB-C PD source is connected (this would need implementation specific to your hardware)
        # For now, we'll assume PD negotiation succeeded if VBUS is close to our PD voltage trigger
        if abs(self.vbus - self.USB_PD_VOLTAGE_TRIGGER) < 1.0:
            new_source = "USB_PD"
        # Check for solar vs DC input
        elif self.vbus > 0.5:  # If there's any meaningful input voltage
            if self.vbus >= self.SOLAR_VOLTAGE_THRESHOLD:
                new_source = "SOLAR"
            else:
                new_source = "DC"
        else:
            new_source = "NONE"
        
        # If source changed, reconfigure
        if new_source != old_source:
            self._input_source = new_source
            print(f"Input source changed: {old_source} -> {new_source}")
            
            if new_source == "SOLAR":
                # Configure for solar input with MPPT
                self.set_charge_current(self.SOLAR_CHARGE_CURRENT)
                self.set_input_current_limit(self.SOLAR_INPUT_CURRENT_LIMIT)
                self.enable_mppt()
            elif new_source == "DC":
                # Configure for DC input
                self.set_charge_current(self.DC_CHARGE_CURRENT)
                self.set_input_current_limit(self.DC_INPUT_CURRENT_LIMIT)
                self.disable_mppt()
            elif new_source == "USB_PD":
                # Configure for USB PD input
                self.set_charge_current(self.USB_PD_CHARGE_CURRENT)
                self.set_input_current_limit(self.USB_PD_INPUT_CURRENT_LIMIT)
                self.disable_mppt()
            else:  # NONE
                # No input source, no special configuration needed
                pass

    def update_charge_state(self):
        """Update charge state information."""
        # Read charge state from register
        charge_status = self._read_register(Regs.CHARGER_STATUS_1, 8, start_bit=5, end_bit=7)
        
        # Map register value to charge state
        state_map = {
            0: NOT_CHARGING,
            1: TRICKLE_CHARGE,
            2: PRECHARGE,
            3: FAST_CHARGE,
            4: TAPER_CHARGE,
            5: RESERVED,
            6: TOP_OFF,
            7: CHARGE_DONE
        }
        
        self._charge_state = state_map.get(charge_status, NOT_CHARGING)

    def update_fault_status(self):
        """Update fault status information."""
        # Read fault status registers
        fault_reg_0 = self._read_register(Regs.FAULT_STATUS_0, 8)
        fault_reg_1 = self._read_register(Regs.FAULT_STATUS_1, 8)
        
        # Update fault status
        self.faults[FAULT_IBAT_REGULATION] = bool(fault_reg_0 & (1 << 7))
        self.faults[FAULT_VBUS_OVP] = bool(fault_reg_0 & (1 << 6))
        self.faults[FAULT_VBAT_OVP] = bool(fault_reg_0 & (1 << 5))
        self.faults[FAULT_IBUS_OCP] = bool(fault_reg_0 & (1 << 4))
        self.faults[FAULT_IBAT_OCP] = bool(fault_reg_0 & (1 << 3))
        self.faults[FAULT_CONVERTER_OCP] = bool(fault_reg_0 & (1 << 2))
        self.faults[FAULT_VAC2_OVP] = bool(fault_reg_0 & (1 << 1))
        self.faults[FAULT_VAC1_OVP] = bool(fault_reg_0 & (1 << 0))
        
        self.faults[FAULT_VSYS_SHORT] = bool(fault_reg_1 & (1 << 7))
        self.faults[FAULT_VSYS_OVP] = bool(fault_reg_1 & (1 << 6))
        self.faults[FAULT_OTG_OVP] = bool(fault_reg_1 & (1 << 5))
        self.faults[FAULT_OTG_UVP] = bool(fault_reg_1 & (1 << 4))
        self.faults[FAULT_TDIE_SHUTDOWN] = bool(fault_reg_1 & (1 << 2))
        
        # If any faults, print them (for debugging)
        active_faults = self.get_active_faults()
        if active_faults:
            print("Active faults:", active_faults)

    def get_active_faults(self):
        """Get a list of active faults."""
        active = []
        fault_names = [
            "IBAT_REGULATION", "VBUS_OVP", "VBAT_OVP", "IBUS_OCP", 
            "IBAT_OCP", "CONVERTER_OCP", "VAC2_OVP", "VAC1_OVP",
            "VSYS_SHORT", "VSYS_OVP", "OTG_OVP", "OTG_UVP", "TDIE_SHUTDOWN"
        ]
        
        for i, fault in enumerate(self.faults):
            if fault:
                active.append(fault_names[i])
                
        return active

    #######################
    # ADC Configuration   #
    #######################
    
    def enable_adc(self):
        """Enable the ADC."""
        try:
            self._write_register(Regs.ADC_CONTROL, 1, start_bit=7)
            return True
        except Exception as e:
            print(f"Failed to enable ADC: {e}")
            return False

    def disable_adc(self):
        """Disable the ADC."""
        try:
            self._write_register(Regs.ADC_CONTROL, 0, start_bit=7)
            return True
        except Exception as e:
            print(f"Failed to disable ADC: {e}")
            return False

    def set_adc_mode(self, mode):
        """Set the ADC mode.
        
        Args:
            mode: ADC mode (ADC_ONE_SHOT or ADC_CONTINUOUS)
        """
        try:
            if mode == ADC_ONE_SHOT:
                # One-shot mode: bit 6 = 1
                self._write_register(Regs.ADC_CONTROL, 1, start_bit=6)
            else:  # ADC_CONTINUOUS
                # Continuous mode: bit 6 = 0
                self._write_register(Regs.ADC_CONTROL, 0, start_bit=6)
                
            self._adc_mode = mode
            print(f"ADC mode set to {'ONE_SHOT' if mode == ADC_ONE_SHOT else 'CONTINUOUS'}")
            return True
        except Exception as e:
            print(f"Failed to set ADC mode: {e}")
            return False

    def set_adc_sample_speed(self, speed):
        """Set the ADC sample speed."""
        try:
            # Write to bits 4-5 of ADC control register
            self._write_register(Regs.ADC_CONTROL, speed, start_bit=4, end_bit=5)
            return True
        except Exception as e:
            print(f"Failed to set ADC sample speed: {e}")
            return False

    def enable_battery_current_sense(self):
        """Enable battery current sensing (discharge current)."""
        try:
            # Set bit 5 of charge control register 5
            self._write_register(Regs.CHARGER_CTRL_5, 1, start_bit=5)
            return True
        except Exception as e:
            print(f"Failed to enable battery current sensing: {e}")
            return False

    def disable_battery_current_sense(self):
        """Disable battery current sensing (discharge current)."""
        try:
            # Clear bit 5 of charge control register 5
            self._write_register(Regs.CHARGER_CTRL_5, 0, start_bit=5)
            return True
        except Exception as e:
            print(f"Failed to disable battery current sensing: {e}")
            return False

    #######################
    # Charging Control    #
    #######################
    
    def set_charge_voltage(self, voltage):
        """Set the battery charge voltage.
        
        Args:
            voltage (float): Charge voltage in volts (3.0-18.8V)
        """
        if not 3.0 <= voltage <= 18.8:
            raise ValueError(f"Charge voltage must be between 3.0V and 18.8V, got {voltage}V")
            
        try:
            # Convert to register value: 10mV per bit
            reg_value = int(voltage * 100)
            self._write_register(Regs.CHARGE_VOLTAGE, reg_value, length=16)
            return True
        except Exception as e:
            print(f"Failed to set charge voltage: {e}")
            return False

    def get_charge_voltage(self):
        """Get the battery charge voltage setting in volts."""
        reg_value = self._read_register(Regs.CHARGE_VOLTAGE, length=16)
        return reg_value / 100.0  # 10mV per bit

    def set_charge_current(self, current):
        """Set the battery charge current.
        
        Args:
            current (float): Charge current in amperes (0.05-5.0A)
        """
        if not 0.05 <= current <= 5.0:
            raise ValueError(f"Charge current must be between 0.05A and 5.0A, got {current}A")
            
        try:
            # Convert to register value: 10mA per bit
            reg_value = int(current * 100)
            self._write_register(Regs.CHARGE_CURRENT, reg_value, length=16)
            return True
        except Exception as e:
            print(f"Failed to set charge current: {e}")
            return False

    def get_charge_current(self):
        """Get the battery charge current setting in amperes."""
        reg_value = self._read_register(Regs.CHARGE_CURRENT, length=16)
        return reg_value / 100.0  # 10mA per bit

    def enable_charging(self):
        """Enable battery charging."""
        try:
            # Set bit 5 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 1, start_bit=5)
            return True
        except Exception as e:
            print(f"Failed to enable charging: {e}")
            return False

    def disable_charging(self):
        """Disable battery charging."""
        try:
            # Clear bit 5 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 0, start_bit=5)
            return True
        except Exception as e:
            print(f"Failed to disable charging: {e}")
            return False

    def is_charging_enabled(self):
        """Check if battery charging is enabled."""
        return bool(self._read_register(Regs.CHARGER_CTRL_0, 8, start_bit=5))

    def set_input_current_limit(self, current):
        """Set the input current limit.
        
        Args:
            current (float): Input current limit in amperes (0.1-3.3A)
        """
        if not 0.1 <= current <= 3.3:
            raise ValueError(f"Input current must be between 0.1A and 3.3A, got {current}A")
            
        try:
            # Convert to register value: 10mA per bit
            reg_value = int(current * 100)
            self._write_register(Regs.INPUT_CURRENT_LIMIT, reg_value, length=16)
            return True
        except Exception as e:
            print(f"Failed to set input current limit: {e}")
            return False

    def get_input_current_limit(self):
        """Get the input current limit setting in amperes."""
        reg_value = self._read_register(Regs.INPUT_CURRENT_LIMIT, length=16)
        return reg_value / 100.0  # 10mA per bit

    def set_input_voltage_limit(self, voltage):
        """Set the input voltage limit (VINDPM).
        
        Args:
            voltage (float): Input voltage limit in volts (3.6-22V)
        """
        if not 3.6 <= voltage <= 22.0:
            raise ValueError(f"Input voltage must be between 3.6V and 22.0V, got {voltage}V")
            
        try:
            # Convert to register value: 100mV per bit
            reg_value = int(voltage * 10)
            self._write_register(Regs.INPUT_VOLTAGE_LIMIT, reg_value, length=8)
            return True
        except Exception as e:
            print(f"Failed to set input voltage limit: {e}")
            return False

    def get_input_voltage_limit(self):
        """Get the input voltage limit (VINDPM) setting in volts."""
        reg_value = self._read_register(Regs.INPUT_VOLTAGE_LIMIT, length=8)
        return reg_value / 10.0  # 100mV per bit

    def set_precharge_current(self, current):
        """Set the precharge current limit.
        
        Args:
            current (float): Precharge current in amperes (0.04-2.0A)
        """
        if not 0.04 <= current <= 2.0:
            raise ValueError(f"Precharge current must be between 0.04A and 2.0A, got {current}A")
            
        try:
            # Convert to register value: 40mA per bit
            reg_value = int(current * 1000 / 40)
            # Write to bits 0-5 of precharge control register
            self._write_register(Regs.PRECHARGE_CONTROL, reg_value, start_bit=0, end_bit=5)
            return True
        except Exception as e:
            print(f"Failed to set precharge current: {e}")
            return False

    def get_precharge_current(self):
        """Get the precharge current limit setting in amperes."""
        reg_value = self._read_register(Regs.PRECHARGE_CONTROL, 8, start_bit=0, end_bit=5)
        return reg_value * 0.04  # 40mA per bit

    def set_termination_current(self, current):
        """Set the charge termination current.
        
        Args:
            current (float): Termination current in amperes (0.04-1.0A)
        """
        if not 0.04 <= current <= 1.0:
            raise ValueError(f"Termination current must be between 0.04A and 1.0A, got {current}A")
            
        try:
            # Convert to register value: 40mA per bit
            reg_value = int(current * 1000 / 40)
            # Write to bits 0-4 of termination control register
            self._write_register(Regs.TERMINATION_CONTROL, reg_value, start_bit=0, end_bit=4)
            return True
        except Exception as e:
            print(f"Failed to set termination current: {e}")
            return False

    def get_termination_current(self):
        """Get the charge termination current setting in amperes."""
        reg_value = self._read_register(Regs.TERMINATION_CONTROL, 8, start_bit=0, end_bit=4)
        return reg_value * 0.04  # 40mA per bit

    def enable_termination(self):
        """Enable charge termination."""
        try:
            # Set bit 1 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 1, start_bit=1)
            return True
        except Exception as e:
            print(f"Failed to enable termination: {e}")
            return False

    def disable_termination(self):
        """Disable charge termination."""
        try:
            # Clear bit 1 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 0, start_bit=1)
            return True
        except Exception as e:
            print(f"Failed to disable termination: {e}")
            return False

    def is_termination_enabled(self):
        """Check if charge termination is enabled."""
        return bool(self._read_register(Regs.CHARGER_CTRL_0, 8, start_bit=1))

    def set_minimum_system_voltage(self, voltage):
        """Set the minimum system voltage.
        
        Args:
            voltage (float): Minimum system voltage in volts (2.5-16.0V)
        """
        if not 2.5 <= voltage <= 16.0:
            raise ValueError(f"Minimum system voltage must be between 2.5V and 16.0V, got {voltage}V")
            
        try:
            # Convert to register value: 250mV per bit, 2.5V offset
            reg_value = int((voltage - 2.5) / 0.25)
            # Write to bits 0-5 of minimum system voltage register
            self._write_register(Regs.MIN_SYS_VOLTAGE, reg_value, start_bit=0, end_bit=5)
            return True
        except Exception as e:
            print(f"Failed to set minimum system voltage: {e}")
            return False

    def get_minimum_system_voltage(self):
        """Get the minimum system voltage setting in volts."""
        reg_value = self._read_register(Regs.MIN_SYS_VOLTAGE, 8, start_bit=0, end_bit=5)
        return 2.5 + (reg_value * 0.25)  # 2.5V offset, 250mV per bit

    def set_recharge_threshold(self, threshold):
        """Set the battery recharge threshold.
        
        Args:
            threshold (float): Recharge threshold in volts (0.05-0.8V)
        """
        if not 0.05 <= threshold <= 0.8:
            raise ValueError(f"Recharge threshold must be between 0.05V and 0.8V, got {threshold}V")
            
        try:
            # Convert to register value: 50mV per bit
            reg_value = int(threshold / 0.05) - 1  # 0-based indexing
            # Write to bits 0-3 of recharge control register
            self._write_register(Regs.RECHARGE_CONTROL, reg_value, start_bit=0, end_bit=3)
            return True
        except Exception as e:
            print(f"Failed to set recharge threshold: {e}")
            return False

    def get_recharge_threshold(self):
        """Get the battery recharge threshold setting in volts."""
        reg_value = self._read_register(Regs.RECHARGE_CONTROL, 8, start_bit=0, end_bit=3)
        return (reg_value + 1) * 0.05  # 50mV per bit, 1-based value

    #######################
    # MPPT Configuration  #
    #######################
    
    def configure_mppt(self, mppt_percentage):
        """Configure Maximum Power Point Tracking for solar input.
        
        Args:
            mppt_percentage: MPPT percentage of open circuit voltage
        """
        try:
            # Write to bits 0-2 of MPPT control register
            self._write_register(Regs.MPPT_CONTROL, mppt_percentage, start_bit=0, end_bit=2)
            return True
        except Exception as e:
            print(f"Failed to configure MPPT: {e}")
            return False

    def enable_mppt(self):
        """Enable Maximum Power Point Tracking for solar input."""
        try:
            # Set bit 7 of MPPT control register
            self._write_register(Regs.MPPT_CONTROL, 1, start_bit=7)
            print("MPPT enabled")
            return True
        except Exception as e:
            print(f"Failed to enable MPPT: {e}")
            return False

    def disable_mppt(self):
        """Disable Maximum Power Point Tracking."""
        try:
            # Clear bit 7 of MPPT control register
            self._write_register(Regs.MPPT_CONTROL, 0, start_bit=7)
            print("MPPT disabled")
            return True
        except Exception as e:
            print(f"Failed to disable MPPT: {e}")
            return False

    def is_mppt_enabled(self):
        """Check if MPPT is enabled."""
        return bool(self._read_register(Regs.MPPT_CONTROL, 8, start_bit=7))
        
    #######################
    # Ship Mode           #
    #######################
    
    def enable_ship_mode(self):
        """Enable ship mode to disconnect the battery."""
        try:
            # Set bit 5 of charger control register 2
            self._write_register(Regs.CHARGER_CTRL_2, 1, start_bit=5)
            print("Ship mode enabled")
            return True
        except Exception as e:
            print(f"Failed to enable ship mode: {e}")
            return False

    def disable_ship_mode(self):
        """Disable ship mode to reconnect the battery."""
        try:
            # Clear bit 5 of charger control register 2
            self._write_register(Regs.CHARGER_CTRL_2, 0, start_bit=5)
            print("Ship mode disabled")
            return True
        except Exception as e:
            print(f"Failed to disable ship mode: {e}")
            return False

    def is_ship_mode_enabled(self):
        """Check if ship mode is enabled."""
        return bool(self._read_register(Regs.CHARGER_CTRL_2, 8, start_bit=5))
    
    #######################
    # USB OTG Functions   #
    #######################
    
    def set_otg_voltage(self, voltage):
        """Set the USB-C sourcing (OTG) voltage.
        
        Args:
            voltage (float): OTG output voltage in volts (2.8-22.0V)
        """
        if not 2.8 <= voltage <= 22.0:
            raise ValueError(f"OTG voltage must be between 2.8V and 22.0V, got {voltage}V")
            
        try:
            # Convert to register value: 10mV per bit, 2.8V offset
            reg_value = int((voltage - 2.8) * 100)
            self._write_register(Regs.OTG_VOLTAGE, reg_value, length=16)
            return True
        except Exception as e:
            print(f"Failed to set OTG voltage: {e}")
            return False

    def get_otg_voltage(self):
        """Get the USB-C sourcing (OTG) voltage setting in volts."""
        reg_value = self._read_register(Regs.OTG_VOLTAGE, length=16)
        return 2.8 + (reg_value / 100.0)  # 2.8V offset, 10mV per bit

    def set_otg_current(self, current):
        """Set the USB-C sourcing (OTG) current limit.
        
        Args:
            current (float): OTG output current in amperes (0.16-3.36A)
        """
        if not 0.16 <= current <= 3.36:
            raise ValueError(f"OTG current must be between 0.16A and 3.36A, got {current}A")
            
        try:
            # Convert to register value: 40mA per bit
            reg_value = int(current * 1000 / 40)
            # Write to bits 0-6 of OTG current register
            self._write_register(Regs.OTG_CURRENT, reg_value, start_bit=0, end_bit=6)
            return True
        except Exception as e:
            print(f"Failed to set OTG current: {e}")
            return False

    def get_otg_current(self):
        """Get the USB-C sourcing (OTG) current limit setting in amperes."""
        reg_value = self._read_register(Regs.OTG_CURRENT, 8, start_bit=0, end_bit=6)
        return reg_value * 0.04  # 40mA per bit
        
    def enable_otg_mode(self):
        """Enable USB OTG mode (USB power delivery)."""
        try:
            # Set bit 4 of charger control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 1, start_bit=4)
            print("OTG mode enabled")
            return True
        except Exception as e:
            print(f"Failed to enable OTG mode: {e}")
            return False

    def disable_otg_mode(self):
        """Disable USB OTG mode."""
        try:
            # Clear bit 4 of charger control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 0, start_bit=4)
            print("OTG mode disabled")
            return True
        except Exception as e:
            print(f"Failed to disable OTG mode: {e}")
            return False

    def is_otg_mode_enabled(self):
        """Check if USB OTG mode is enabled."""
        return bool(self._read_register(Regs.CHARGER_CTRL_0, 8, start_bit=4))
    
    #######################
    # Timer Configuration #
    #######################
    
    def set_precharge_timeout(self, minutes):
        """Set the precharge timeout.
        
        Args:
            minutes (int): Timeout in minutes (30 or 120)
        """
        if minutes != 30 and minutes != 120:
            raise ValueError(f"Precharge timeout must be 30 or 120 minutes, got {minutes}")
            
        try:
            # 0 = 120 minutes (2 hours), 1 = 30 minutes (0.5 hours)
            reg_value = 1 if minutes == 30 else 0
            # Write to bit 7 of OTG current register
            self._write_register(Regs.OTG_CURRENT, reg_value, start_bit=7)
            return True
        except Exception as e:
            print(f"Failed to set precharge timeout: {e}")
            return False

    def get_precharge_timeout(self):
        """Get the precharge timeout setting in minutes."""
        reg_value = self._read_register(Regs.OTG_CURRENT, 8, start_bit=7)
        return 30 if reg_value == 1 else 120

    def set_fast_charge_timeout(self, minutes):
        """Set the fast charge timeout.
        
        Args:
            minutes (int): Timeout in minutes (300, 480, 720, or 1440)
        """
        allowed_values = {300: 0, 480: 1, 720: 2, 1440: 3}  # minutes: register value
        if minutes not in allowed_values:
            raise ValueError(f"Fast charge timeout must be one of {list(allowed_values.keys())}, got {minutes}")
            
        try:
            # Write to bits 1-2 of timer control register
            self._write_register(Regs.TIMER_CONTROL, allowed_values[minutes], start_bit=1, end_bit=2)
            return True
        except Exception as e:
            print(f"Failed to set fast charge timeout: {e}")
            return False

    def get_fast_charge_timeout(self):
        """Get the fast charge timeout setting in minutes."""
        reg_value = self._read_register(Regs.TIMER_CONTROL, 8, start_bit=1, end_bit=2)
        timeout_map = {0: 300, 1: 480, 2: 720, 3: 1440}
        return timeout_map.get(reg_value, 300)
    
    #######################
    # Battery Information #
    #######################
    
    def get_charge_state(self):
        """Get the current charge state as an integer constant."""
        return self._charge_state
        
    def get_charge_state_name(self):
        """Get the current charge state as a string."""
        state_map = {
            NOT_CHARGING: "Not charging",
            TRICKLE_CHARGE: "Trickle charging",
            PRECHARGE: "Pre-charging",
            FAST_CHARGE: "Fast charging (CC)",
            TAPER_CHARGE: "Taper charging (CV)",
            RESERVED: "Reserved",
            TOP_OFF: "Top-off charging",
            CHARGE_DONE: "Charge complete"
        }
        return state_map.get(self._charge_state, "Unknown")
        
    def is_battery_charging(self):
        """Check if the battery is currently charging."""
        charging_states = [TRICKLE_CHARGE, PRECHARGE, FAST_CHARGE, TAPER_CHARGE, TOP_OFF]
        return self._charge_state in charging_states

    def estimate_battery_percentage(self):
        """Estimate battery state of charge (percentage).
        
        Note: This is an approximation based on the battery voltage and a reference curve.
        For more accurate SoC estimation, additional techniques like coulomb counting
        should be used.
        
        Returns:
            float: Estimated battery percentage (0-100)
        """
        if self.vbat <= 0:
            self.update_adc_readings()
            
        if self.vbat <= 0:
            print("Cannot estimate battery percentage: battery voltage reading is zero")
            return 0
            
        # Get voltage per cell
        voltage_per_cell = self.vbat / self.battery_cells
        
        # Get reference curve for LiFePO4
        curve = self.LIFEPO4_SOC_CURVE
        
        # Find the closest reference points
        if voltage_per_cell <= curve[0][0]:
            return curve[0][1]  # Below minimum voltage
        
        if voltage_per_cell >= curve[-1][0]:
            return curve[-1][1]  # Above maximum voltage
        
        # Linear interpolation between reference points
        for i in range(len(curve) - 1):
            v1, p1 = curve[i]
            v2, p2 = curve[i + 1]
            
            if v1 <= voltage_per_cell <= v2:
                # Linear interpolation
                return p1 + (p2 - p1) * (voltage_per_cell - v1) / (v2 - v1)
        
        # Shouldn't reach here
        return 0

    def get_charging_metrics(self):
        """Get comprehensive charging metrics.
        
        Returns:
            dict: Charging metrics dictionary
        """
        self.update_all()
        
        # Calculate power values
        input_power = self.vbus * self.ibus
        battery_power = self.vbat * self.ibat
        
        # Estimate efficiency when charging
        efficiency = 0.0
        if input_power > 0 and battery_power > 0:
            efficiency = (battery_power / input_power) * 100.0
            
        # Estimate time to full
        time_to_full = float('inf')
        if self.is_battery_charging() and self.ibat > 0:
            # Very rough estimate based on battery capacity and current charge rate
            soc = self.estimate_battery_percentage()
            remaining_capacity = self.battery_capacity_ah * (100 - soc) / 100
            time_to_full = remaining_capacity / self.ibat  # hours
        
        return {
            'state': self.get_charge_state_name(),
            'source': self._input_source,
            'voltage': self.vbat,
            'current': self.ibat,
            'soc': self.estimate_battery_percentage(),
            'input_power': input_power,
            'battery_power': battery_power,
            'efficiency': efficiency,
            'time_to_full': time_to_full,
            'temperature': self.tdie,
            'faults': self.get_active_faults()
        }


# Example usage
if __name__ == "__main__":
    import time
    
    print("Initializing BQ25798 Battery Charger")
    
    # Create an instance of the BQ25798 driver
    # Default I2C pins for Raspberry Pi Pico are GPIO 0 (SDA) and GPIO 1 (SCL)
    charger = BQ25798(sda_pin=0, scl_pin=1)
    
    # Main loop
    try:
        while True:
            # Update status and automatically configure for input source
            charger.update_all()
            
            # Get charging metrics
            metrics = charger.get_charging_metrics()
            
            # Print current status
            print(f"Source: {metrics['source']}, State: {metrics['state']}")
            print(f"Battery: {metrics['voltage']:.2f}V, {metrics['current']:.2f}A, {metrics['soc']:.1f}%")
            print(f"Input: {charger.vbus:.2f}V, {charger.ibus:.2f}A")
            print(f"Temperature: {metrics['temperature']:.1f}°C")
            
            if metrics['faults']:
                print(f"Faults: {metrics['faults']}")
                
            print("-" * 40)
            
            # Wait before next update
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("Exiting...")
        
    finally:
        # Always try to make sure charging is properly configured before exit
        if charger._input_source != "NONE":
            charger.enable_charging()
