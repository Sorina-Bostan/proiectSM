import machine
import utime
import math
import json
import max30100

measurement_active_in_pulse_py = False 

I2C_BUS_ID_PULSE = 0
SCL_PIN_PULSE = 5
SDA_PIN_PULSE = 4
FINGER_MIN_IR_VALUE_PULSE = 5000
FINGER_MIN_RED_VALUE_PULSE = 5000
SPO2_CALCULATION_WINDOW_SIZE_PULSE = 100
MIN_SAMPLES_FOR_SPO2_PULSE = 30
POLL_INTERVAL_MS_PULSE = 50
SAMPLE_RATE_SPS_PULSE = 100
SMOOTHING_WINDOW_SIZE_PULSE = 10
MODE_SPO2_PULSE = max30100.MODE_SPO2_EN

LED_CURRENT_RED_PULSE = 14.2
LED_CURRENT_IR_PULSE = 40.2
MIN_AC_AMP_SPO2_PULSE = 20.0

DATA_FILE_PATH = "/sensordata.json"
FILE_WRITE_INTERVAL_S = 1

def median(lst):
    n = len(lst)
    s = sorted(lst)
    if n == 0:
        return 0
    if n % 2 == 1:
        return s[n // 2]
    else:
        return (s[n // 2 - 1] + s[n // 2]) / 2

class BPMCalculator:
    def __init__(self, sample_rate_sps=SAMPLE_RATE_SPS_PULSE, window_seconds=8, bpm_avg_size=6):
        self.sample_rate = sample_rate_sps
        self.ir_data_buffer = []
        self.buffer_max_size = int(sample_rate_sps * window_seconds)
        self.last_peak_time_ms = 0
        self.beat_intervals_ms = []
        self.bpm_avg_size = bpm_avg_size
        self.current_bpm = 0.0
        self.min_samples_between_peaks = int(sample_rate_sps * 0.3)
        self.samples_since_last_peak = 0
        self.last_peak_detected_time_ms = utime.ticks_ms()
        self.peak_timeout_ms = 4000

    def add_ir_reading(self, ir_sample):
        if self.ir_data_buffer:
            prev = self.ir_data_buffer[-1]
            if abs(ir_sample - prev) > 0.2 * prev:
                return
        self.ir_data_buffer.append(ir_sample)
        if len(self.ir_data_buffer) > self.buffer_max_size:
            self.ir_data_buffer.pop(0)
        self.samples_since_last_peak += 1
        if len(self.ir_data_buffer) < 5:
            return

        idx = -3
        prev2 = self.ir_data_buffer[idx]
        prev1 = self.ir_data_buffer[idx+1]
        curr = self.ir_data_buffer[idx+2]
        next1 = self.ir_data_buffer[idx+3]
        next2 = self.ir_data_buffer[idx+4]

        window = self.ir_data_buffer[-self.buffer_max_size:] if len(self.ir_data_buffer) >= self.buffer_max_size else self.ir_data_buffer
        min_ir = min(window)
        max_ir = max(window)
        median_ir = median(window)
        threshold = median_ir + 0.4 * (max_ir - median_ir)

        is_peak = (
            prev1 < curr > next1 and
            curr > threshold and
            self.samples_since_last_peak >= self.min_samples_between_peaks
        )

        if is_peak:
            current_time_ms = utime.ticks_ms()
            self.last_peak_detected_time_ms = current_time_ms
            if self.last_peak_time_ms > 0:
                interval_ms = utime.ticks_diff(current_time_ms, self.last_peak_time_ms)
                if 300 < interval_ms < 2000:
                    self.beat_intervals_ms.append(interval_ms)
                    if len(self.beat_intervals_ms) > self.bpm_avg_size:
                        self.beat_intervals_ms.pop(0)
                    if len(self.beat_intervals_ms) > 0:
                        avg_interval_ms = sum(self.beat_intervals_ms) / len(self.beat_intervals_ms)
                        self.current_bpm = 60000.0 / avg_interval_ms
                        print(f"pulse.py: BPM peak detected, interval={interval_ms} ms, BPM={self.current_bpm:.1f}")
            self.last_peak_time_ms = current_time_ms
            self.samples_since_last_peak = 0

    def get_bpm(self):
        if utime.ticks_diff(utime.ticks_ms(), self.last_peak_detected_time_ms) > self.peak_timeout_ms:
            self.current_bpm = 0.0
            self.beat_intervals_ms.clear()
        if 30 < self.current_bpm < 220:
            return self.current_bpm
        return 0.0

    def reset(self):
        self.ir_data_buffer.clear()
        self.last_peak_time_ms = 0
        self.beat_intervals_ms.clear()
        self.current_bpm = 0.0
        self.samples_since_last_peak = 0

class SpO2Calculator:
    def __init__(self, buffer_size=SPO2_CALCULATION_WINDOW_SIZE_PULSE):
        self.buffer_size = buffer_size
        self.red_buffer = []
        self.ir_buffer = []
        self.spo2_value = 0.0
        self.last_dc_red = 0
        self.last_dc_ir = 0
        self.last_ac_red_rms = 0
        self.last_ac_ir_rms = 0
        self.last_r_value = 0.0

    def add_reading(self, red_sample, ir_sample):
        if self.red_buffer and abs(red_sample - self.red_buffer[-1]) > 0.2 * self.red_buffer[-1]:
            return
        if self.ir_buffer and abs(ir_sample - self.ir_buffer[-1]) > 0.2 * self.ir_buffer[-1]:
            return
        self.red_buffer.append(red_sample)
        self.ir_buffer.append(ir_sample)
        while len(self.red_buffer) > self.buffer_size:
            self.red_buffer.pop(0)
        while len(self.ir_buffer) > self.buffer_size:
            self.ir_buffer.pop(0)

    def calculate_spo2(self):
        if len(self.red_buffer) < MIN_SAMPLES_FOR_SPO2_PULSE:
            return self.spo2_value
        red_buf = list(self.red_buffer)
        ir_buf = list(self.ir_buffer)
        if len(red_buf) > 5:
            red_buf = sorted(red_buf)[1:-1]
        if len(ir_buf) > 5:
            ir_buf = sorted(ir_buf)[1:-1]
        dc_red = sum(red_buf) / len(red_buf)
        dc_ir = sum(ir_buf) / len(ir_buf)
        self.last_dc_red = dc_red
        self.last_dc_ir = dc_ir
        if dc_red == 0 or dc_ir == 0:
            return self.spo2_value
        ac_red_sq_sum = sum([(v - dc_red) ** 2 for v in red_buf])
        ac_red_rms = math.sqrt(ac_red_sq_sum / len(red_buf))
        ac_ir_sq_sum = sum([(v - dc_ir) ** 2 for v in ir_buf])
        ac_ir_rms = math.sqrt(ac_ir_sq_sum / len(ir_buf))
        self.last_ac_red_rms = ac_red_rms
        self.last_ac_ir_rms = ac_ir_rms
        if ac_red_rms < MIN_AC_AMP_SPO2_PULSE or ac_ir_rms < MIN_AC_AMP_SPO2_PULSE:
            return self.spo2_value
        ratio_red = ac_red_rms / dc_red
        ratio_ir = ac_ir_rms / dc_ir
        if ratio_ir == 0:
            return self.spo2_value
        R = ratio_red / ratio_ir
        if R < 0.3: R = 0.3
        if R > 1.2: R = 1.2
        self.last_r_value = R
        a = -45.060
        b = 30.354
        c = 94.845
        spo2_calc = a * R * R + b * R + c
        if spo2_calc > 100.0: spo2_calc = 100.0
        elif spo2_calc < 70.0: spo2_calc = 70.0
        self.spo2_value = spo2_calc
        return self.spo2_value

    def reset(self):
        self.red_buffer.clear()
        self.ir_buffer.clear()
        self.spo2_value = 0.0
        self.last_r_value = 0.0
        self.last_dc_red = 0
        self.last_dc_ir = 0
        self.last_ac_red_rms = 0
        self.last_ac_ir_rms = 0

class PulseOximeter:
    def __init__(self):
        print("PulseOximeter: __init__ called")
        self.initialized = False
        self.sensor_status = "Initializing..."
        self.is_finger = False
        self.current_spo2 = 0.0
        self.current_bpm_val = 0.0
        self.signal_quality = 0.0
        self.last_file_write_time_ms = utime.ticks_ms()
        self._init_sensor()

    def _init_sensor(self):
        print("PulseOximeter: _init_sensor called")
        try:
            self.i2c = machine.I2C(I2C_BUS_ID_PULSE, scl=machine.Pin(SCL_PIN_PULSE), sda=machine.Pin(SDA_PIN_PULSE), freq=400000)
            self.sensor = max30100.MAX30100(
                i2c=self.i2c, mode=MODE_SPO2_PULSE, sample_rate=SAMPLE_RATE_SPS_PULSE,
                led_current_red=LED_CURRENT_RED_PULSE, led_current_ir=LED_CURRENT_IR_PULSE,
                pulse_width=1600, adc_range=4096,
                max_buffer_len=SPO2_CALCULATION_WINDOW_SIZE_PULSE + SMOOTHING_WINDOW_SIZE_PULSE + 20
            )
            self.spo2_calc = SpO2Calculator()
            self.bpm_calc = BPMCalculator()
            self.raw_ir_smoothing_buffer = []
            self.raw_red_smoothing_buffer = []
            self.initialized = True
            self.sensor_status = "Sensor OK. Place finger."
            print("Sensor initialized OK", self.sensor_status)
            print("I2C scan from pulse:", self.i2c.scan())
        except Exception as e:
            self.sensor_status = "Sensor Init ERROR"
            print("pulse.py: Sensor init error:", e)
            self.initialized = False

    def step(self, active):
        print(f"PulseOximeter: step called, active={active}, initialized={self.initialized}")
        if not self.initialized:
            self._init_sensor()
            return 0, 0, False

        if not active:
            self.sensor_status = "Stopped. Click Start."
            self.is_finger = False
            self.current_spo2 = 0.0
            self.current_bpm_val = 0.0
            return 0, 0, False

        try:
            self.sensor.read_sensor_fifo()
            print(f"PulseOximeter: sensor read completed, ir={self.sensor.ir}, red={self.sensor.red}")
            latest_ir = self.sensor.ir
            latest_red = self.sensor.red
            print(f"latest_ir={latest_ir}, latest_red={latest_red}")

            if latest_ir is not None and latest_red is not None:
                self.raw_ir_smoothing_buffer.append(latest_ir)
                self.raw_red_smoothing_buffer.append(latest_red)
                if len(self.raw_ir_smoothing_buffer) > SMOOTHING_WINDOW_SIZE_PULSE:
                    self.raw_ir_smoothing_buffer.pop(0)
                if len(self.raw_red_smoothing_buffer) > SMOOTHING_WINDOW_SIZE_PULSE:
                    self.raw_red_smoothing_buffer.pop(0)

                if len(self.raw_ir_smoothing_buffer) == SMOOTHING_WINDOW_SIZE_PULSE:
                    current_ir_smoothed = sum(self.raw_ir_smoothing_buffer) / SMOOTHING_WINDOW_SIZE_PULSE
                    current_red_smoothed = sum(self.raw_red_smoothing_buffer) / SMOOTHING_WINDOW_SIZE_PULSE
                else:
                    current_ir_smoothed = latest_ir
                    current_red_smoothed = latest_red

                print(f"latest_ir={latest_ir}, latest_red={latest_red}")
                print(f"current_ir_smoothed={current_ir_smoothed}, current_red_smoothed={current_red_smoothed}")
                
                print("is_finger:", self.is_finger)
                
                was_finger = self.is_finger

                if current_ir_smoothed > FINGER_MIN_IR_VALUE_PULSE and current_red_smoothed > FINGER_MIN_RED_VALUE_PULSE:
                    print("Finger detected with sufficient signal quality.")
                    if not was_finger:
                        self.sensor_status = "Finger detected. Stabilizing..."
                        print("Transition: no finger -> finger detected. Resetting buffers.")
                        self.spo2_calc.reset()
                        self.bpm_calc.reset()
                        self.raw_ir_smoothing_buffer.clear()
                        self.raw_red_smoothing_buffer.clear()
                    self.is_finger = True

                    self.spo2_calc.add_reading(current_red_smoothed, current_ir_smoothed)
                    self.bpm_calc.add_ir_reading(current_ir_smoothed)

                    if len(self.spo2_calc.red_buffer) >= MIN_SAMPLES_FOR_SPO2_PULSE:
                        calc_spo2 = self.spo2_calc.calculate_spo2()
                        if 70.0 <= calc_spo2 <= 100.0 and (0.3 < self.spo2_calc.last_r_value < 1.2):
                            self.current_spo2 = calc_spo2
                        elif calc_spo2 <= 70.0:
                            self.spo2_calc.reset()
                            self.current_spo2 = 0.0

                    calc_bpm = self.bpm_calc.get_bpm()
                    if calc_bpm > 0:
                        self.current_bpm_val = calc_bpm
                elif self.is_finger:
                    print("Finger removed or signal lost.")
                    self.sensor_status = "Finger removed or signal lost."
                    self.is_finger = False
                    self.spo2_calc.reset()
                    self.bpm_calc.reset()
                    self.raw_ir_smoothing_buffer.clear()
                    self.raw_red_smoothing_buffer.clear()
                    self.current_spo2 = 0.0
                    self.current_bpm_val = 0.0
                else:
                    print("No finger detected or signal too weak.")
                    self.sensor_status = "Place finger on sensor."
                    self.current_spo2 = 0.0
                    self.current_bpm_val = 0.0
            print(f"PulseOximeter: returning bpm={self.current_bpm_val}, spo2={self.current_spo2}, finger_on={self.is_finger}")
            return int(self.current_bpm_val), int(self.current_spo2), self.is_finger

        except Exception as e:
            print("pulse.py: Sensor step error:", e)
            return 0, 0, False

if __name__ == "__main__":
    import time
    print("Running pulse.py as main for testing...")
    ox = PulseOximeter()
    ox_active = True
    try:
        while True:
            bpm, spo2, finger_on = ox.step(ox_active)
            print(f"[TEST LOOP] BPM={bpm}, SpO2={spo2}, Finger on={finger_on}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Test loop stopped by user.")