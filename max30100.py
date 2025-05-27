# max30100.py
""""
  MicroPython Library for the Maxim MAX30100 pulse oximetry system.
  Adapted for generic MicroPython platforms (e.g., RP2040/RP2350).

  Original ESP8266 version by J5 (j5liu@yahoo.com), MAY 2018
  Based on original Python library by mfitzip: https://github.com/mfitzp/MAX30100
"""

import machine

# Register Addresses
INT_STATUS   = 0x00  # Which interrupts are tripped
INT_ENABLE   = 0x01  # Which interrupts are active
FIFO_WR_PTR  = 0x02  # Where data is being written
OVRFLOW_CTR  = 0x03  # Number of lost samples
FIFO_RD_PTR  = 0x04  # Where to read from
FIFO_DATA    = 0x05  # Output data buffer
MODE_CONFIG  = 0x06  # Control register
SPO2_CONFIG  = 0x07  # Oximetry settings
LED_CONFIG   = 0x09  # Pulse width and power of LEDs
TEMP_INTG    = 0x16  # Temperature value, whole number
TEMP_FRAC    = 0x17  # Temperature value, fraction
REV_ID       = 0xFE  # Part revision
PART_ID      = 0xFF  # Part ID, normally 0x11

I2C_ADDRESS  = 0x57  # I2C address of the MAX30100 device

# Configuration options mapped to register values
PULSE_WIDTH = { # LED Pulse Width (us) and ADC Resolution
    200: 0,  # 13-bit
    400: 1,  # 14-bit
    800: 2,  # 15-bit
   1600: 3,  # 16-bit
}

SAMPLE_RATE = { # Samples per second
    50: 0,
   100: 1,
   167: 2, # Note: Datasheet says 167, but often 200 is the next step after 100 for some implementations.
          # Let's assume 167 is a specific value if the user requests it.
   200: 3,
   400: 4,
   600: 5,
   800: 6,
  1000: 7,
}

LED_CURRENT = { # LED Current (mA)
       0: 0,
     4.4: 1,
     7.6: 2,
    11.0: 3,
    14.2: 4,
    17.4: 5,
    20.8: 6,
    24.0: 7,
    27.1: 8,
    30.6: 9,
    33.8: 10,
    37.0: 11,
    40.2: 12,
    43.6: 13,
    46.8: 14,
    50.0: 15
}

ADC_RANGE = { # SpO2 ADC Range (nA)
    2048: 0,
    4096: 1,
    8192: 2,
   16384: 3,
}

# Modes
MODE_HR_ONLY = 0x02
MODE_SPO2_EN = 0x03 # SpO2 enabled (activates Red and IR LEDs)

# Interrupt types for enable_interrupt (maps to which bit to set high in INT_ENABLE)
# These are the bit numbers (0-7), the function will shift them.
# Example: INT_A_FULL = 7, so (1 << 7)
INT_A_FULL_EN   = 0b10000000 # FIFO Almost Full Enable
INT_TEMP_RDY_EN = 0b01000000 # Temperature Ready Enable
INT_HR_RDY_EN   = 0b00100000 # Heart Rate Data Ready Enable
INT_SPO2_RDY_EN = 0b00010000 # SpO2 Data Ready Enable
# PWR_RDY is only in INT_STATUS, not enableable

def _get_valid(d, value, value_type="value"):
    """Helper to get register value from human-readable value, or raise KeyError."""
    try:
        return d[value]
    except KeyError:
        raise KeyError(
            f"{value_type} {value} not valid. Use one of: {', '.join(map(str, d.keys()))}"
        )

def _twos_complement(val, bits):
    """Compute the 2's complement of int value val."""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)
    return val


class MAX30100(object):

    def __init__(self,
                 i2c=None,
                 mode=MODE_SPO2_EN,
                 sample_rate=100,   # Samples per second
                 led_current_red=11.0, # mA
                 led_current_ir=11.0,  # mA
                 pulse_width=1600,   # us, for 16-bit ADC resolution
                 adc_range=4096,     # nA
                 max_buffer_len=100  # Local buffer for readings
                 ):

        if i2c is None:
            raise ValueError("I2C object must be provided.")
        self.i2c = i2c

        # Perform a soft reset to ensure a known state
        self.reset()
        # Wait for reset to complete. Datasheet doesn't specify time,
        # but a small delay is good practice.
        # The RESET bit clears itself. We can poll it or just wait.
        import utime
        utime.sleep_ms(10) # Brief delay after reset

        # Basic configuration
        self.set_mode(mode)
        self.set_led_currents(led_current_red, led_current_ir)
        self.configure_spo2_and_adc(sample_rate, pulse_width, adc_range)

        # Clear FIFO pointers and overflow counter before starting
        self.clear_fifo()

        # Reflectance data (latest update stored locally)
        self.buffer_red = []
        self.buffer_ir = []
        self.max_buffer_len = max_buffer_len
        
        # Prime the temperature sensor for first read if needed
        # self.refresh_temperature() # Mode setting might do this

    @property
    def red(self):
        """Latest Red LED value from the local buffer."""
        return self.buffer_red[-1] if self.buffer_red else None

    @property
    def ir(self):
        """Latest IR LED value from the local buffer."""
        return self.buffer_ir[-1] if self.buffer_ir else None

    def _read_byte_data(self, reg_addr):
        """Reads a single byte from the specified register."""
        return self.i2c.readfrom_mem(I2C_ADDRESS, reg_addr, 1)[0]

    def _read_bytes_data(self, reg_addr, num_bytes):
        """Reads multiple bytes from the specified register."""
        return self.i2c.readfrom_mem(I2C_ADDRESS, reg_addr, num_bytes)

    def _write_byte_data(self, reg_addr, value):
        """Writes a single byte to the specified register."""
        self.i2c.writeto_mem(I2C_ADDRESS, reg_addr, bytes([value]))

    def set_led_currents(self, red_current_ma=11.0, ir_current_ma=11.0):
        """Sets the LED currents for Red and IR LEDs."""
        red_reg_val = _get_valid(LED_CURRENT, red_current_ma, "Red LED current")
        ir_reg_val = _get_valid(LED_CURRENT, ir_current_ma, "IR LED current")
        # Red LED current is in the upper nibble, IR in the lower.
        self._write_byte_data(LED_CONFIG, (red_reg_val << 4) | ir_reg_val)

    def set_mode(self, mode):
        """
        Sets the operating mode of the sensor.
        MODE_HR_ONLY (0x02) or MODE_SPO2_EN (0x03).
        Also used to trigger temperature readings.
        """
        if mode not in [MODE_HR_ONLY, MODE_SPO2_EN]:
            raise ValueError("Invalid mode specified.")
        
        current_mode_config = self._read_byte_data(MODE_CONFIG)
        # Clear mode bits (0-2) and TEMP_EN (bit 3), preserve SHDN (bit 7)
        new_mode_config = (current_mode_config & 0b11110000) | mode
        self._write_byte_data(MODE_CONFIG, new_mode_config)

    def configure_spo2_and_adc(self, sample_rate_sps=100, pulse_width_us=1600, adc_range_na=4096):
        """
        Configures SpO2 settings: ADC range, sample rate, and LED pulse width (ADC resolution).
        """
        adc_rge_val = _get_valid(ADC_RANGE, adc_range_na, "ADC range")
        sr_val = _get_valid(SAMPLE_RATE, sample_rate_sps, "Sample rate")
        pw_val = _get_valid(PULSE_WIDTH, pulse_width_us, "Pulse width")

        # SPO2_CONFIG register layout:
        # Bit 7: SPO2_HI_RES_EN (0 for 1.6ms pulse width, 1 for 1.6ms with 16-bit ADC)
        #        This bit seems redundant if LED_PW sets resolution.
        #        Datasheet: "When this bit is set to one, the SpO2 ADC resolution is 16-bit with 1.6ms LED pulse width."
        #        It's usually left as 0, and LED_PW controls resolution.
        #        Let's assume we control resolution via pulse_width_us / pw_val.
        # Bits 6:5: SPO2_ADC_RGE[1:0]
        # Bits 4:2: SPO2_SR[2:0]
        # Bits 1:0: LED_PW[1:0]

        spo2_config_val = (adc_rge_val << 5) | (sr_val << 2) | pw_val
        # If pulse_width_us is 1600 (16-bit), some might set SPO2_HI_RES_EN.
        # For now, let's keep it simple and not set bit 7 unless explicitly needed.
        # if pulse_width_us == 1600:
        #     spo2_config_val |= (1 << 7) # Enable HI_RES for 16-bit

        self._write_byte_data(SPO2_CONFIG, spo2_config_val)

    def enable_interrupt(self, interrupt_mask):
        """
        Enables one or more interrupts based on the mask.
        Example: enable_interrupt(INT_SPO2_RDY_EN | INT_A_FULL_EN)
        To clear pending interrupts, read INT_STATUS after enabling.
        """
        self._write_byte_data(INT_ENABLE, interrupt_mask)
        self.get_interrupt_status() # Read to clear any pending interrupt flags

    def get_interrupt_status(self):
        """Reads and returns the interrupt status register. Reading clears the register."""
        return self._read_byte_data(INT_STATUS)

    def get_samples_available(self):
        """Returns the number of new samples available in the FIFO."""
        write_ptr = self._read_byte_data(FIFO_WR_PTR)
        read_ptr = self._read_byte_data(FIFO_RD_PTR)
        # Formula from datasheet: (write_ptr - read_ptr + 16) % 16
        return (write_ptr - read_ptr + 16) % 16

    def read_fifo_one_sample(self):
        """Reads one sample (IR and Red data) from the FIFO."""
        fifo_data_bytes = self._read_bytes_data(FIFO_DATA, 4)
        # Data is IR MSB, IR LSB, Red MSB, Red LSB
        ir_value = (fifo_data_bytes[0] << 8) | fifo_data_bytes[1]
        red_value = (fifo_data_bytes[2] << 8) | fifo_data_bytes[3]
        return ir_value, red_value

    def read_sensor_fifo(self):
        """
        Reads all available samples from the FIFO and updates local buffers.
        This is the primary method to get new data.
        """
        num_samples = self.get_samples_available()
        
        for _ in range(num_samples):
            try:
                ir_val, red_val = self.read_fifo_one_sample()
                self.buffer_ir.append(ir_val)
                self.buffer_red.append(red_val)
            except Exception as e:
                # print(f"Error reading one sample from FIFO: {e}") # Optional debug
                break # Stop if there's an error

        # Keep local buffers to max_buffer_len
        self.buffer_red = self.buffer_red[-self.max_buffer_len:]
        self.buffer_ir = self.buffer_ir[-self.max_buffer_len:]

    def clear_fifo(self):
        """Clears the FIFO by setting read and write pointers to 0."""
        self._write_byte_data(FIFO_WR_PTR, 0)
        self._write_byte_data(FIFO_RD_PTR, 0)
        self._write_byte_data(OVRFLOW_CTR, 0) # Also clear overflow counter

    def shutdown(self):
        """Puts the MAX30100 into power-down mode."""
        current_mode_config = self._read_byte_data(MODE_CONFIG)
        self._write_byte_data(MODE_CONFIG, current_mode_config | 0x80) # Set SHDN bit (bit 7)

    def wakeup(self):
        """Wakes the MAX30100 from power-down mode."""
        current_mode_config = self._read_byte_data(MODE_CONFIG)
        self._write_byte_data(MODE_CONFIG, current_mode_config & ~0x80) # Clear SHDN bit

    def reset(self):
        """Performs a software reset of the MAX30100."""
        current_mode_config = self._read_byte_data(MODE_CONFIG)
        self._write_byte_data(MODE_CONFIG, current_mode_config | 0x40) # Set RESET bit (bit 6)
        # The RESET bit clears itself. Wait for it to clear.
        # Polling is better than a fixed delay.
        import utime
        timeout = 100 # ms
        start_time = utime.ticks_ms()
        while (self._read_byte_data(MODE_CONFIG) & 0x40):
            if utime.ticks_diff(utime.ticks_ms(), start_time) > timeout:
                # print("Warning: Reset bit did not clear automatically.")
                break
            utime.sleep_ms(1)


    def refresh_temperature(self):
        """Initiates a new temperature reading."""
        current_mode_config = self._read_byte_data(MODE_CONFIG)
        # Set TEMP_EN bit (bit 3)
        self._write_byte_data(MODE_CONFIG, current_mode_config | (1 << 3))
        # TEMP_EN bit clears itself when the reading is done.

    def get_temperature(self):
        """
        Returns the die temperature in degrees Celsius.
        Call refresh_temperature() first and wait for TEMP_RDY interrupt or poll.
        """
        # Ensure temperature reading is triggered if not recently done.
        # A simple way is to always trigger it, then wait for it to complete.
        self.refresh_temperature()
        
        # Wait for TEMP_EN bit to clear (or TEMP_RDY interrupt if configured)
        import utime
        timeout = 100 # ms, adjust as needed
        start_time = utime.ticks_ms()
        while (self._read_byte_data(MODE_CONFIG) & (1 << 3)): # Check if TEMP_EN is still set
            if utime.ticks_diff(utime.ticks_ms(), start_time) > timeout:
                # print("Warning: Temperature reading timed out (TEMP_EN did not clear).")
                return None # Or raise an error
            utime.sleep_ms(1)

        temp_intg = self._read_byte_data(TEMP_INTG)
        temp_frac = self._read_byte_data(TEMP_FRAC)
        
        # Temperature is stored in two's complement if it can be negative,
        # but MAX30100 operating range is -40 to +85.
        # The TEMP_INTG register is signed.
        temperature = _twos_complement(temp_intg, 8) + (temp_frac * 0.0625)
        return temperature

    def get_revision_id(self):
        """Returns the revision ID of the MAX30100."""
        return self._read_byte_data(REV_ID)

    def get_part_id(self):
        """Returns the part ID of the MAX30100 (should be 0x11)."""
        return self._read_byte_data(PART_ID)

    def get_all_registers(self):
        """Returns a dictionary of all relevant register values for debugging."""
        return {
            "INT_STATUS": self._read_byte_data(INT_STATUS),
            "INT_ENABLE": self._read_byte_data(INT_ENABLE),
            "FIFO_WR_PTR": self._read_byte_data(FIFO_WR_PTR),
            "OVRFLOW_CTR": self._read_byte_data(OVRFLOW_CTR),
            "FIFO_RD_PTR": self._read_byte_data(FIFO_RD_PTR),
            # FIFO_DATA is read in chunks, not a single value here
            "MODE_CONFIG": self._read_byte_data(MODE_CONFIG),
            "SPO2_CONFIG": self._read_byte_data(SPO2_CONFIG),
            "LED_CONFIG": self._read_byte_data(LED_CONFIG),
            "TEMP_INTG": self._read_byte_data(TEMP_INTG),
            "TEMP_FRAC": self._read_byte_data(TEMP_FRAC),
            "REV_ID": self._read_byte_data(REV_ID),
            "PART_ID": self._read_byte_data(PART_ID),
        }