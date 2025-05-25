# main.py
import machine
import utime
import math
import max30100 # Ensure max30100.py is in the same directory

# --- Board Specific Configuration ---
I2C_BUS_ID = 0
SCL_PIN = 5 # GPIO5
SDA_PIN = 4 # GPIO4
# --------------------------------------------------------------------

# --- SpO2 Parameters ---
# Tune these based on observed DC levels when finger is on vs. off
FINGER_MIN_IR_VALUE = 5000  # Example: Adjust based on your readings with good LED currents
FINGER_MIN_RED_VALUE = 5000 # Example: Adjust based on your readings

SPO2_CALCULATION_WINDOW_SIZE = 500
MIN_SAMPLES_FOR_SPO2 = 100

POLL_INTERVAL_MS = 20
SAMPLE_RATE_SPS = 100

SMOOTHING_WINDOW_SIZE = 50
raw_ir_smoothing_buffer = []
raw_red_smoothing_buffer = []
# -------------------------------------

MODE_SPO2 = max30100.MODE_SPO2_EN


class SpO2Calculator:
    def __init__(self, buffer_size=SPO2_CALCULATION_WINDOW_SIZE):
        self.buffer_size = buffer_size
        self.red_buffer = []
        self.ir_buffer = []
        self.spo2_value = 0.0
        
        self.last_dc_red = 0
        self.last_dc_ir = 0
        self.last_ac_red_rms = 0
        self.last_ac_ir_rms = 0
        self.last_r_value = 0.0
        # self.finger_present = False # Not used in this class directly

    def add_reading(self, red_sample, ir_sample):
        self.red_buffer.append(red_sample)
        self.ir_buffer.append(ir_sample)

        while len(self.red_buffer) > self.buffer_size:
            self.red_buffer.pop(0)
        while len(self.ir_buffer) > self.buffer_size:
            self.ir_buffer.pop(0)

    def calculate_spo2(self):
        if len(self.red_buffer) < MIN_SAMPLES_FOR_SPO2:
            return 0.0

        dc_red = sum(self.red_buffer) / len(self.red_buffer)
        dc_ir = sum(self.ir_buffer) / len(self.ir_buffer)
        
        self.last_dc_red = dc_red
        self.last_dc_ir = dc_ir

        # Optional: Secondary DC check, though main loop finger detection is primary
        # Consider adjusting or removing if FINGER_MIN_RED_VALUE is well-tuned in main
        # if dc_red < FINGER_MIN_RED_VALUE / 2 or dc_ir < FINGER_MIN_IR_VALUE / 2 :
        #     return 0.0 # Semnal prea slab

        ac_red_sq_sum = sum([(val - dc_red) ** 2 for val in self.red_buffer])
        ac_red_rms = math.sqrt(ac_red_sq_sum / len(self.red_buffer))

        ac_ir_sq_sum = sum([(val - dc_ir) ** 2 for val in self.ir_buffer])
        ac_ir_rms = math.sqrt(ac_ir_sq_sum / len(self.ir_buffer))
        
        self.last_ac_red_rms = ac_red_rms
        self.last_ac_ir_rms = ac_ir_rms

        MIN_AC_AMPLITUDE = 30.0 # Or 40.0, based on your observations of AC for good readings
        if ac_red_rms < MIN_AC_AMPLITUDE or ac_ir_rms < MIN_AC_AMPLITUDE:
            # print(f"AC too weak: ACR={ac_red_rms:.1f}, ACI={ac_ir_rms:.1f}")
            return self.spo2_value # Return last valid value

        if dc_red == 0 or dc_ir == 0:
             return self.spo2_value
        ratio_red = ac_red_rms / dc_red
        ratio_ir = ac_ir_rms / dc_ir

        if ratio_ir == 0:
            return self.spo2_value

        R = ratio_red / ratio_ir
        self.last_r_value = R

        a = -45.060
        b = 30.354
        c = 94.845
        spo2_calc = a * R * R + b * R + c

        if spo2_calc > 100.0:
            spo2_calc = 100.0
        elif spo2_calc < 70.0: # This floor is still useful
            spo2_calc = 70.0

        self.spo2_value = spo2_calc
        return self.spo2_value

    def reset(self):
        # print("Resetting SpO2 calculator...") # Optional
        self.red_buffer.clear()
        self.ir_buffer.clear()
        self.spo2_value = 0.0
        self.last_r_value = 0.0
        self.last_dc_red = 0
        self.last_dc_ir = 0
        self.last_ac_red_rms = 0
        self.last_ac_ir_rms = 0
        # self.finger_present = False # Not used
        
        
# Add this class definition alongside your SpO2Calculator class

class BPMCalculator:
    def __init__(self, sample_rate_sps=100, window_seconds=6, bpm_avg_size=4): # window_seconds to look for peaks
        self.sample_rate = sample_rate_sps
        self.ir_data_buffer = [] # Buffer to store IR data for peak detection
        self.buffer_max_size = int(sample_rate_sps * window_seconds) # e.g., 100sps * 6s = 600 samples
        
        self.last_peak_time_ms = 0
        self.beat_intervals_ms = [] # Store recent beat-to-beat intervals in ms
        self.bpm_avg_size = bpm_avg_size # Number of recent beats to average for BPM
        
        self.current_bpm = 0.0
        
        # Peak detection parameters (these may need tuning)
        # self.peak_threshold_factor = 0.6 # Not used in this simplified version yet
        self.min_samples_between_peaks = int(sample_rate_sps * 0.3) # e.g., 300ms physiological limit (200 BPM max)
        self.samples_since_last_peak = 0
        self.last_ir_value_for_slope = 0 # To detect rising/falling slope for simple peak detection (not fully used in current simple peak)

    def add_ir_reading(self, ir_sample):
        self.ir_data_buffer.append(ir_sample) # <<< FIXED >>>
        if len(self.ir_data_buffer) > self.buffer_max_size: # <<< FIXED (both self.)>>>
            self.ir_data_buffer.pop(0) # <<< FIXED >>>

        self.samples_since_last_peak += 1 # <<< FIXED >>>

        if len(self.ir_data_buffer) < 3: # <<< FIXED >>>
            self.last_ir_value_for_slope = ir_sample # <<< FIXED >>>
            return

        is_potential_peak = False
        if len(self.ir_data_buffer) >= 3 and \
           self.ir_data_buffer[-3] < self.ir_data_buffer[-2] and \
           self.ir_data_buffer[-2] > self.ir_data_buffer[-1]: # <<< FIXED (all self.ir_data_buffer) >>>
            
            if self.samples_since_last_peak >= self.min_samples_between_peaks: # <<< FIXED (both self.) >>>
                is_potential_peak = True

        if is_potential_peak:
            current_time_ms = utime.ticks_ms()
            if self.last_peak_time_ms > 0: # <<< FIXED >>>
                interval_ms = utime.ticks_diff(current_time_ms, self.last_peak_time_ms) # <<< FIXED >>>
                
                if 300 < interval_ms < 2000: # Physiological filter
                    self.beat_intervals_ms.append(interval_ms) # <<< FIXED >>>
                    if len(self.beat_intervals_ms) > self.bpm_avg_size: # <<< FIXED (both self.) >>>
                        self.beat_intervals_ms.pop(0) # <<< FIXED >>>
                    
                    if len(self.beat_intervals_ms) > 0: # <<< FIXED >>>
                        avg_interval_ms = sum(self.beat_intervals_ms) / len(self.beat_intervals_ms) # <<< FIXED >>>
                        self.current_bpm = 60000.0 / avg_interval_ms # <<< FIXED >>>
                    else:
                        self.current_bpm = 0.0 # <<< FIXED >>>
            
            self.last_peak_time_ms = current_time_ms # <<< FIXED >>>
            self.samples_since_last_peak = 0 # <<< FIXED >>>
        
        self.last_ir_value_for_slope = ir_sample # <<< FIXED >>>


    def get_bpm(self):
        if 30 < self.current_bpm < 220: # <<< FIXED >>>
            return self.current_bpm # <<< FIXED >>>
        return 0.0

    def reset(self):
        # print("Resetting BPM calculator...") # Optional
        self.ir_data_buffer.clear() # <<< FIXED >>>
        self.last_peak_time_ms = 0 # <<< FIXED >>>
        self.beat_intervals_ms.clear() # <<< FIXED >>>
        self.current_bpm = 0.0 # <<< FIXED >>>
        self.samples_since_last_peak = 0 # <<< FIXED >>>
        self.last_ir_value_for_slope = 0 # <<< FIXED >>>
        
        

def main():
    print("Inițializare senzor MAX30100 pentru SpO2 & BPM...")

    led_current_setting_red = 14.2
    led_current_setting_ir = 40.2

    try:
        i2c = machine.I2C(I2C_BUS_ID, scl=machine.Pin(SCL_PIN), sda=machine.Pin(SDA_PIN), freq=400000)
        devices = i2c.scan()
        # ... (rest of sensor initialization as before) ...
        sensor = max30100.MAX30100(
            i2c=i2c,
            mode=MODE_SPO2, # SpO2 mode also provides IR data needed for BPM
            sample_rate=SAMPLE_RATE_SPS,
            led_current_red = led_current_setting_red,
            led_current_ir = led_current_setting_ir,
            pulse_width=1600,
            adc_range=4096,
            max_buffer_len=SPO2_CALCULATION_WINDOW_SIZE + SMOOTHING_WINDOW_SIZE + 10
        )
        print(f"Senzor MAX30100 (Part ID: {hex(sensor.get_part_id())}) inițializat. Așezați degetul ferm și acoperiți senzorul.")

    except Exception as e:
        # ... (exception handling as before) ...
        return

    spo2_calc = SpO2Calculator()
    bpm_calc = BPMCalculator(sample_rate_sps=SAMPLE_RATE_SPS) # <<< NEW: Instantiate BPM calculator

    last_print_time = utime.ticks_ms()
    finger_on = False
    print_interval_ms = 1000

    while True:
        try:
            sensor.read_sensor_fifo()
            latest_ir_raw = sensor.ir
            latest_red_raw = sensor.red

            current_ir_smoothed = latest_ir_raw if latest_ir_raw is not None else 0
            current_red_smoothed = latest_red_raw if latest_red_raw is not None else 0

            if latest_ir_raw is not None and latest_red_raw is not None:
                # Smoothing (as before)
                raw_ir_smoothing_buffer.append(latest_ir_raw)
                raw_red_smoothing_buffer.append(latest_red_raw)

                if len(raw_ir_smoothing_buffer) > SMOOTHING_WINDOW_SIZE:
                    raw_ir_smoothing_buffer.pop(0)
                if len(raw_red_smoothing_buffer) > SMOOTHING_WINDOW_SIZE:
                    raw_red_smoothing_buffer.pop(0)
                
                if len(raw_ir_smoothing_buffer) == SMOOTHING_WINDOW_SIZE:
                    current_ir_smoothed = sum(raw_ir_smoothing_buffer) / SMOOTHING_WINDOW_SIZE
                    current_red_smoothed = sum(raw_red_smoothing_buffer) / SMOOTHING_WINDOW_SIZE
                
                # Finger detection (as before)
                if current_ir_smoothed > FINGER_MIN_IR_VALUE and \
                   current_red_smoothed > FINGER_MIN_RED_VALUE:
                    if not finger_on:
                        print("Deget detectat. Se stabilizează citirile...")
                        finger_on = True
                        spo2_calc.reset()
                        bpm_calc.reset() # <<< NEW: Reset BPM calculator
                        raw_ir_smoothing_buffer.clear()
                        raw_red_smoothing_buffer.clear()
                    
                    # Add readings to calculators
                    spo2_calc.add_reading(current_red_smoothed, current_ir_smoothed)
                    bpm_calc.add_ir_reading(current_ir_smoothed) # <<< NEW: Add IR to BPM calculator
                    
                    calculated_spo2_this_cycle = 0.0
                    current_bpm_value = 0.0 # <<< NEW
                    
                    if len(spo2_calc.red_buffer) >= MIN_SAMPLES_FOR_SPO2:
                        calculated_spo2_this_cycle = spo2_calc.calculate_spo2()
                    
                    current_bpm_value = bpm_calc.get_bpm() # <<< NEW: Get BPM

                    # Conditional Printing (as before, now add BPM)
                    if utime.ticks_diff(utime.ticks_ms(), last_print_time) >= print_interval_ms:
                        spo2_display_str = f"{calculated_spo2_this_cycle:.1f}%" if calculated_spo2_this_cycle > 0 else "Calc..."
                        if calculated_spo2_this_cycle < 90.0 and calculated_spo2_this_cycle > 0 : # If SpO2 is calculated but low
                            spo2_display_str = f"{calculated_spo2_this_cycle:.1f}% (R:{spo2_calc.last_r_value:.3f})"
                        elif not (calculated_spo2_this_cycle >= 90.0 and (0.3 < spo2_calc.last_r_value < 0.9)):
                            # If not printing full SpO2, at least show BPM if available
                            if current_bpm_value > 0:
                                print(f"Status: BPM: {current_bpm_value:.1f} | IR:{current_ir_smoothed:<5.0f} RED:{current_red_smoothed:<5.0f} SpO2:{spo2_display_str}")
                            else:
                                print(f"Status: Collecting... | IR:{current_ir_smoothed:<5.0f} RED:{current_red_smoothed:<5.0f} SpO2:{spo2_display_str}")
                            last_print_time = utime.ticks_ms()
                            # Continue to next iteration of the loop if we printed a status message
                            # and don't meet the >90% SpO2 condition, to avoid printing the full line.
                            # However, the original request was to *only* print full if SpO2 > 90.
                            # Let's stick to that.

                        # Print full line if SpO2 is good, or just status if BPM is the only thing ready.
                        if calculated_spo2_this_cycle >= 90.0 and (0.3 < spo2_calc.last_r_value < 0.9):
                            print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | "
                                  f"BPM: {current_bpm_value:.1f} | SpO2: {calculated_spo2_this_cycle:.1f}% | R: {spo2_calc.last_r_value:.3f} "
                                  f"(ACr:{spo2_calc.last_ac_red_rms:.1f}, DCr:{spo2_calc.last_dc_red:.0f}, "
                                  f"ACir:{spo2_calc.last_ac_ir_rms:.1f}, DCir:{spo2_calc.last_dc_ir:.0f})")
                        elif current_bpm_value > 0: # SpO2 not >90 or R bad, but BPM is available
                             print(f"Status: BPM: {current_bpm_value:.1f} | IR:{current_ir_smoothed:<5.0f} RED:{current_red_smoothed:<5.0f} SpO2:{spo2_display_str}")
                        # else: # Neither good SpO2 nor BPM ready, but finger is on.
                        #     print(f"Status: Processing... | IR:{current_ir_smoothed:<5.0f} RED:{current_red_smoothed:<5.0f}")

                        last_print_time = utime.ticks_ms()
                
                elif finger_on: # Finger removed
                    print("Deget îndepărtat sau semnal slab.")
                    finger_on = False
                    spo2_calc.reset()
                    bpm_calc.reset() # <<< NEW: Reset BPM calculator
                    raw_ir_smoothing_buffer.clear()
                    raw_red_smoothing_buffer.clear()
                    if utime.ticks_diff(utime.ticks_ms(), last_print_time) >= print_interval_ms:
                        print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | BPM: --- | SpO2: ---")
                        last_print_time = utime.ticks_ms()
                
                else: # Finger not on
                    if utime.ticks_diff(utime.ticks_ms(), last_print_time) >= print_interval_ms:
                        print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | BPM: --- | SpO2: Așteaptă degetul...")
                        last_print_time = utime.ticks_ms()

            utime.sleep_ms(POLL_INTERVAL_MS)

        except OSError as e:
            # ... (OSError handling as before, ensure bpm_calc.reset() is also called) ...
            print(f"Eroare I2C: {e}. Se reîncearcă...")
            utime.sleep_ms(1000)
            try:
                sensor.reset(); utime.sleep_ms(100)
                sensor.set_mode(MODE_SPO2)
                sensor.configure_spo2_and_adc(sample_rate=SAMPLE_RATE_SPS, pulse_width=1600, adc_range=4096)
                sensor.set_led_currents(led_current_red=led_current_setting_red, led_current_ir=led_current_setting_ir)
                sensor.clear_fifo()
                spo2_calc.reset()
                bpm_calc.reset() # <<< NEW
                raw_ir_smoothing_buffer.clear(); raw_red_smoothing_buffer.clear()
                finger_on = False
                print("Senzor resetat și reconfigurat după eroare I2C.")
            except Exception as reinit_e:
                print(f"Eroare la resetarea senzorului după eroare I2C: {reinit_e}")
                utime.sleep_ms(5000)
        except Exception as e:
            # ... (general exception handling as before) ...
            import sys
            sys.print_exception(e)
            print(f"Eroare în bucla principală.")
            utime.sleep_ms(1000)


if __name__ == "__main__":
    main()
