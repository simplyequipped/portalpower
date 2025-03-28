"""
Integrated Battery Management System with USB-C PD

Combines BQ25798 battery charger and TUSB422 USB-C PD controller 
for a complete power management solution using the Raspberry Pi Pico.

Features:
- 4S LiFePO4 battery charging with automatic source detection (solar, DC, USB-C PD)
- USB-C PD sink mode (receiving power) for battery charging
- USB-C PD source mode (providing power) for OTG functionality
- Automatic switching between charging and OTG modes
- Maximum Power Point Tracking (MPPT) for solar input
"""

import time
from machine import I2C, Pin
import _thread

# Import our driver modules - make sure these are in the same directory
from bq25798_micropython import BQ25798
from tusb422_driver import TUSB422

# Configure debug logging
DEBUG = True

def log(message):
    """Print debug messages if debugging is enabled."""
    if DEBUG:
        print(message)

class PowerManager:
    """Integrated power management system with BQ25798 and TUSB422."""
    
    # Power state constants
    STATE_NO_POWER = const(0)
    STATE_SOLAR_CHARGING = const(1)
    STATE_DC_CHARGING = const(2)
    STATE_USB_PD_CHARGING = const(3)
    STATE_OTG_ENABLED = const(4)
    
    # OTG mode selection options
    OTG_OFF = const(0)
    OTG_AUTO = const(1)
    OTG_MANUAL_ON = const(2)
    
    def __init__(self, sda_pin=0, scl_pin=1, pd_int_pin=2, otg_button_pin=None):
        """Initialize the power management system.
        
        Args:
            sda_pin: I2C SDA pin number
            scl_pin: I2C SCL pin number
            pd_int_pin: TUSB422 interrupt pin
            otg_button_pin: Optional GPIO pin for OTG mode button
        """
        # Configure I2C bus (shared by both ICs)
        self.i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=100000)
        
        # Current system state
        self.current_state = self.STATE_NO_POWER
        self.otg_mode = self.OTG_AUTO
        self.last_status_check = 0
        
        # Configure OTG button if provided
        self.otg_button = None
        if otg_button_pin is not None:
            self.otg_button = Pin(otg_button_pin, Pin.IN, Pin.PULL_UP)
        
        log("Initializing Battery Charger (BQ25798)...")
        # Initialize BQ25798 with 4S LiFePO4 configuration
        self.charger = BQ25798(
            i2c=self.i2c, 
            update_interval=5000  # 5 second update interval
        )
        
        log("Initializing USB-C PD Controller (TUSB422)...")
        # Initialize TUSB422 USB-C PD controller
        self.pd_controller = TUSB422(
            i2c=self.i2c,
            int_pin=pd_int_pin
        )
        
        # Start in sink mode by default (for charging)
        self.pd_controller.set_sink_mode()
        
        # Initial configuration of both ICs
        self._configure_for_charging()
        
        # Start monitoring thread for state transitions
        _thread.start_new_thread(self._monitor_thread, ())
        
        log("Power Manager initialization complete")
    
    def _configure_for_charging(self):
        """Configure the system for battery charging."""
        # Configure charger for default settings
        self.charger.set_charge_voltage(self.charger.LIFEPO4_4S_MAX_VOLTAGE)
        self.charger.set_minimum_system_voltage(self.charger.LIFEPO4_4S_MIN_VOLTAGE)
        self.charger.enable_charging()
        
        # Configure PD controller for sink mode
        if not self.pd_controller.is_sink_mode:
            self.pd_controller.set_sink_mode()
            self.pd_controller.advertise_sink_capabilities()
    
    def _configure_for_otg(self):
        """Configure the system for USB OTG mode (providing power)."""
        # Disable charging
        self.charger.disable_charging()
        
        # Configure PD controller for source mode
        if not self.pd_controller.is_source_mode:
            self.pd_controller.set_source_mode()
            
            # Configure OTG voltage and current
            self.charger.set_otg_voltage(5.0)  # 5V output
            self.charger.set_otg_current(3.0)  # 3A max
            
            # Enable OTG mode on BQ25798
            self.charger.enable_otg_mode()
            
            # Advertise source capabilities
            self.pd_controller.advertise_source_capabilities()
    
    def _monitor_thread(self):
        """Background thread to monitor system state and handle transitions."""
        while True:
            try:
                # Check charger status
                self.charger.update_all()
                
                # Check USB PD controller status
                self.pd_controller.poll()
                
                # Determine the current state based on inputs
                self._update_system_state()
                
                # Implement state machine logic
                self._handle_state_transitions()
                
                # Sleep for a short duration to prevent CPU hogging
                time.sleep_ms(100)
                
            except Exception as e:
                log(f"Error in monitor thread: {e}")
                time.sleep_ms(1000)  # Longer delay on error
    
    def _update_system_state(self):
        """Update the current system state based on inputs."""
        old_state = self.current_state
        
        # Check if OTG mode is manually enabled via button
        if self.otg_button is not None and self.otg_button.value() == 0:
            # Button is pressed (active low), enable OTG mode
            self.otg_mode = self.OTG_MANUAL_ON
        elif self.otg_mode == self.OTG_MANUAL_ON and self.otg_button is not None and self.otg_button.value() == 1:
            # Button is released, return to auto mode
            self.otg_mode = self.OTG_AUTO
        
        # Check USB-C connection status
        usb_c_connected, _ = self.pd_controller.get_connection_status()
        
        # If manual OTG mode is enabled, override everything else
        if self.otg_mode == self.OTG_MANUAL_ON and self.charger.vbat > self.charger.LIFEPO4_4S_MIN_VOLTAGE + 1.0:
            self.current_state = self.STATE_OTG_ENABLED
        
        # If in auto mode, determine state based on inputs
        elif self.otg_mode == self.OTG_AUTO:
            if usb_c_connected and self.pd_controller.is_vbus_present():
                # USB-C PD is connected and providing power
                self.current_state = self.STATE_USB_PD_CHARGING
            elif self.charger.vbus > 0.5:  # Any meaningful input voltage
                if self.charger.vbus >= self.charger.SOLAR_VOLTAGE_THRESHOLD:
                    # Solar input detected
                    self.current_state = self.STATE_SOLAR_CHARGING
                else:
                    # DC input detected
                    self.current_state = self.STATE_DC_CHARGING
            elif usb_c_connected and not self.pd_controller.is_vbus_present() and \
                 self.charger.vbat > self.charger.LIFEPO4_4S_MIN_VOLTAGE + 1.0:
                # Device is connected but not providing power, and battery has enough charge
                # Switch to OTG mode to provide power
                self.current_state = self.STATE_OTG_ENABLED
            else:
                # No power source detected
                self.current_state = self.STATE_NO_POWER
        
        # Log state changes
        if old_state != self.current_state:
            state_names = {
                self.STATE_NO_POWER: "No Power",
                self.STATE_SOLAR_CHARGING: "Solar Charging",
                self.STATE_DC_CHARGING: "DC Charging",
                self.STATE_USB_PD_CHARGING: "USB-PD Charging",
                self.STATE_OTG_ENABLED: "OTG Mode Enabled"
            }
            log(f"State changed: {state_names.get(old_state, 'Unknown')} -> {state_names.get(self.current_state, 'Unknown')}")
    
    def _handle_state_transitions(self):
        """Handle transitions between system states."""
        # Current time for interval checking
        current_time = time.ticks_ms()
        
        # Only execute full status check periodically
        if time.ticks_diff(current_time, self.last_status_check) < 1000:  # 1 second
            return
            
        self.last_status_check = current_time
        
        # Handle each state
        if self.current_state == self.STATE_NO_POWER:
            # Make sure we're in sink mode to accept any power that becomes available
            if not self.pd_controller.is_sink_mode:
                self._configure_for_charging()
                
            # Ensure OTG mode is disabled
            self.charger.disable_otg_mode()
            
        elif self.current_state == self.STATE_SOLAR_CHARGING:
            # Configure for solar charging with MPPT
            self._configure_for_charging()
            self.charger.set_charge_current(self.charger.SOLAR_CHARGE_CURRENT)
            self.charger.set_input_current_limit(self.charger.SOLAR_INPUT_CURRENT_LIMIT)
            self.charger.enable_mppt()
            
        elif self.current_state == self.STATE_DC_CHARGING:
            # Configure for DC charging
            self._configure_for_charging()
            self.charger.set_charge_current(self.charger.DC_CHARGE_CURRENT)
            self.charger.set_input_current_limit(self.charger.DC_INPUT_CURRENT_LIMIT)
            self.charger.disable_mppt()
            
        elif self.current_state == self.STATE_USB_PD_CHARGING:
            # Configure for USB-PD charging
            self._configure_for_charging()
            self.charger.set_charge_current(self.charger.USB_PD_CHARGE_CURRENT)
            self.charger.set_input_current_limit(self.charger.USB_PD_INPUT_CURRENT_LIMIT)
            self.charger.disable_mppt()
            
            # Make sure we're advertising sink capabilities
            self.pd_controller.advertise_sink_capabilities()
            
            # Request 15V PD (if not already negotiated)
            if self.pd_controller.negotiated_voltage != 15.0:
                self.pd_controller.request_power(15.0, 3.0)
            
        elif self.current_state == self.STATE_OTG_ENABLED:
            # Configure for OTG mode
            self._configure_for_otg()
    
    def get_metrics(self):
        """Get comprehensive system metrics.
        
        Returns:
            dict: System metrics
        """
        # Get charging metrics from BQ25798
        charger_metrics = self.charger.get_charging_metrics()
        
        # Get PD controller status
        pd_connected, cc_pin = self.pd_controller.get_connection_status()
        pd_role = self.pd_controller.get_current_role()
        
        # Determine current operating mode
        if self.current_state == self.STATE_OTG_ENABLED:
            operating_mode = "OTG (Power Source)"
        elif self.current_state in (self.STATE_SOLAR_CHARGING, self.STATE_DC_CHARGING, self.STATE_USB_PD_CHARGING):
            operating_mode = "Charging"
        else:
            operating_mode = "Standby"
        
        # Combine metrics
        metrics = {
            'state': self.current_state,
            'operating_mode': operating_mode,
            'usb_pd_connected': pd_connected,
            'usb_pd_cc_pin': cc_pin,
            'usb_pd_role': pd_role,
            'battery_voltage': self.charger.vbat,
            'battery_current': self.charger.ibat,
            'battery_soc': charger_metrics['soc'],
            'input_voltage': self.charger.vbus,
            'input_current': self.charger.ibus,
            'system_voltage': self.charger.vsys,
            'temperature': self.charger.tdie,
            'faults': charger_metrics['faults'],
            'charge_state': charger_metrics['state'],
            'mppt_enabled': self.charger.is_mppt_enabled()
        }
        
        return metrics
    
    def set_otg_mode(self, mode):
        """Set the OTG mode.
        
        Args:
            mode: OTG mode (OTG_OFF, OTG_AUTO, OTG_MANUAL_ON)
        """
        if mode in (self.OTG_OFF, self.OTG_AUTO, self.OTG_MANUAL_ON):
            self.otg_mode = mode
            
            # Force state update immediately
            self._update_system_state()
            self._handle_state_transitions()
            
            log(f"OTG mode set to {mode}")
            return True
        else:
            log(f"Invalid OTG mode: {mode}")
            return False


# Example usage
if __name__ == "__main__":
    import time
    
    print("Initializing Power Management System")
    
    # Create an instance of the power manager
    # GPIO 15 is used as an optional OTG manual control button
    power_manager = PowerManager(sda_pin=0, scl_pin=1, pd_int_pin=2, otg_button_pin=15)
    
    # Main loop
    try:
        while True:
            # Get current metrics
            metrics = power_manager.get_metrics()
            
            # Print current status
            state_names = {
                PowerManager.STATE_NO_POWER: "No Power",
                PowerManager.STATE_SOLAR_CHARGING: "Solar Charging",
                PowerManager.STATE_DC_CHARGING: "DC Charging",
                PowerManager.STATE_USB_PD_CHARGING: "USB-PD Charging",
                PowerManager.STATE_OTG_ENABLED: "OTG Mode"
            }
            
            print(f"State: {state_names.get(metrics['state'], 'Unknown')}")
            print(f"Mode: {metrics['operating_mode']}")
            print(f"Battery: {metrics['battery_voltage']:.2f}V, {metrics['battery_current']:.2f}A, {metrics['battery_soc']:.1f}%")
            print(f"Input: {metrics['input_voltage']:.2f}V, {metrics['input_current']:.2f}A")
            print(f"USB-C: Connected={metrics['usb_pd_connected']}, Role={metrics['usb_pd_role']}")
            print(f"Temperature: {metrics['temperature']:.1f}°C")
            
            if metrics['faults']:
                print(f"Faults: {metrics['faults']}")
                
            print("-" * 40)
            
            # Wait before next update
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("Exiting...")
