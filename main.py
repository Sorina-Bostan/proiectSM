# main.py
import machine
import utime
import math
import max30100 # Asigură-te că fișierul max30100.py (modificat anterior) este în același director

# --- Configurare specifică plăcii ---
I2C_BUS_ID = 0
SCL_PIN = 5 # GPIO5
SDA_PIN = 4 # GPIO4
# --------------------------------------------------------------------

# --- Parametri pentru SpO2 ---
# Thresholds for detecting a finger based on raw sensor readings
FINGER_MIN_IR_VALUE = 3000
FINGER_MIN_RED_VALUE = 4000

# MODIFICARE: Mărire fereastră de calcul - Corespunde cu timpul necesar pentru a capta mai multe bătăi de inimă
SPO2_CALCULATION_WINDOW_SIZE = 500 # Număr de mostre (ex: la 100sps, asta e 5 secunde de date)
MIN_SAMPLES_FOR_SPO2 = 100         # Număr minim de mostre în buffere înainte de a încerca calculul (1s la 100sps)

POLL_INTERVAL_MS = 20 # Interval de citire a senzorului
SAMPLE_RATE_SPS = 100 # Rata de eșantionare a senzorului (100 samples per second)

# MODIFICARE: Parametri pentru media mobilă de netezire a semnalului brut
# Netezirea semnalului brut înainte de procesarea AC/DC poate reduce zgomotul aleator.
SMOOTHING_WINDOW_SIZE = 50 # Câte eșantioane să mediezi pentru netezire (0.5s la 100sps)
raw_ir_smoothing_buffer = []
raw_red_smoothing_buffer = []
# -------------------------------------

# Constante de mod din biblioteca max30100.py
MODE_SPO2 = max30100.MODE_SPO2_EN


class SpO2Calculator:
    def __init__(self, buffer_size=SPO2_CALCULATION_WINDOW_SIZE):
        self.buffer_size = buffer_size
        self.red_buffer = []
        self.ir_buffer = []
        self.spo2_value = 0.0 # Valoarea SpO2 calculată
        
        # Variabile pentru debug și afișare detaliată
        self.last_dc_red = 0
        self.last_dc_ir = 0
        self.last_ac_red_rms = 0
        self.last_ac_ir_rms = 0
        self.last_r_value = 0.0
        self.finger_present = False

    def add_reading(self, red_sample, ir_sample):
        # Adaugă eșantioane noi și menține dimensiunea bufferului
        self.red_buffer.append(red_sample)
        self.ir_buffer.append(ir_sample)

        while len(self.red_buffer) > self.buffer_size:
            self.red_buffer.pop(0)
        while len(self.ir_buffer) > self.buffer_size:
            self.ir_buffer.pop(0)

    def calculate_spo2(self):
        # Asigură-te că avem suficiente date în buffer pentru un calcul stabil
        if len(self.red_buffer) < MIN_SAMPLES_FOR_SPO2:
            return 0.0 # Sau o valoare invalidă pentru a indica că nu e gata

        # 1. Calculul Componentelor DC (Media semnalului)
        # Reprezintă lumina absorbită de alte componente decât sângele arterial pulsatil.
        dc_red = sum(self.red_buffer) / len(self.red_buffer)
        dc_ir = sum(self.ir_buffer) / len(self.ir_buffer)
        
        self.last_dc_red = dc_red
        self.last_dc_ir = dc_ir

        # Verifică dacă semnalul DC este suficient de puternic pentru a indica prezența degetului
        # Această verificare este complementară cu cea din main.
        if dc_red < FINGER_MIN_RED_VALUE / 2 or dc_ir < FINGER_MIN_IR_VALUE / 2 :
            return 0.0 # Semnal prea slab

        # 2. Calculul Componentelor AC (Amplitudinea pulsațiilor)
        # Reprezintă lumina absorbită de sângele arterial pulsatil.
        # RMS (Root Mean Square) este o modalitate robustă de a estima amplitudinea AC.
        ac_red_sq_sum = 0
        for val in self.red_buffer:
            ac_red_sq_sum += (val - dc_red) ** 2
        ac_red_rms = math.sqrt(ac_red_sq_sum / len(self.red_buffer))

        ac_ir_sq_sum = 0
        for val in self.ir_buffer:
            ac_ir_sq_sum += (val - dc_ir) ** 2
        ac_ir_rms = math.sqrt(ac_ir_sq_sum / len(self.ir_buffer))
        
        self.last_ac_red_rms = ac_red_rms
        self.last_ac_ir_rms = ac_ir_rms

        # MODIFICARE: Prag de Amplitudine Minima AC - Esențial pentru a asigura un semnal PPG valid
        # Dacă pulsația este prea slabă, calculul R devine instabil și imprecis.
        # Valoarea exactă necesită calibrare și testare. Poate fi ajustată.
        MIN_AC_AMPLITUDE = 15.0 # Ajustează acest prag!
        if ac_red_rms < MIN_AC_AMPLITUDE or ac_ir_rms < MIN_AC_AMPLITUDE:
            # print(f"AC too weak: ACR={ac_red_rms:.1f}, ACI={ac_ir_rms:.1f}")
            # Returnează ultima valoare validă sau 0.0/None pentru a indica o citire slabă.
            return self.spo2_value # Mentine ultima valoare valida pentru stabilitate vizuala

        # 3. Calculul Raportului de Perfuzație (AC/DC)
        # PI = AC / DC, reflectă cantitatea de sânge pulsatil în comparație cu totalul.
        ratio_red = ac_red_rms / dc_red
        ratio_ir = ac_ir_rms / dc_ir

        # Evită împărțirea la zero
        if ratio_ir == 0:
            return self.spo2_value # Mentine ultima valoare valida

        # 4. Calculul Raportului R (Ratio of Ratios)
        # R = (ACred/DCred) / (ACired / DCired) - Aceasta este valoarea cheie pentru SpO2.
        # Conform documentației Maxim.
        R = ratio_red / ratio_ir
        self.last_r_value = R # Stochează valoarea R pentru debug

        # 5. Calculul SpO2 folosind Formula Cuadratică (din documentația Maxim)
        # SpO2 = aR^2 + bR + c
        # Coeficienții a, b, c sunt coeficienți de calibrare.
        # Acestea sunt valori comune găsite în implementări pentru MAX30100/MAX30102.
        # O calibrare specifică pentru senzorul și amplasarea ta ar oferi cea mai bună acuratețe.
        a = -45.060
        b = 30.354
        c = 94.845
        spo2_calc = a * R * R + b * R + c

        # Limitează valoarea SpO2 la un interval rezonabil
        if spo2_calc > 100.0:
            spo2_calc = 100.0
        elif spo2_calc < 70.0: # Rar sub 70% la o persoană conștientă fără probleme grave
            spo2_calc = 70.0

        self.spo2_value = spo2_calc
        return self.spo2_value

    def reset(self):
        print("Resetting SpO2 calculator...")
        self.red_buffer.clear()
        self.ir_buffer.clear()
        self.spo2_value = 0.0
        self.last_r_value = 0.0
        self.finger_present = False


def main():
    print("Inițializare senzor MAX30100 pentru SpO2...")
    try:
        i2c = machine.I2C(I2C_BUS_ID, scl=machine.Pin(SCL_PIN), sda=machine.Pin(SDA_PIN), freq=400000)
        
        devices = i2c.scan()
        expected_addr = max30100.I2C_ADDRESS
        if not expected_addr in devices:
            print(f"Senzorul MAX30100 nu a fost găsit la adresa 0x{expected_addr:02X}!")
            print(f"Dispozitive găsite: {[hex(d) for d in devices]}")
            return

        print("Configuring sensor for SpO2...")
        # Configurațiile senzorului sunt critice pentru calitatea semnalului.
        # Puls width și ADC range afectează rezoluția și timpul de integrare.
        # LED currents afectează intensitatea luminii. Ajustează pentru a obține valori DC bune (nu saturate, nu prea mici).
        sensor = max30100.MAX30100(
            i2c=i2c,
            mode=MODE_SPO2,
            sample_rate=SAMPLE_RATE_SPS,
            led_current_red=11.0, # Ajustează între 0.0 și 50.0 mA
            led_current_ir=11.0,  # Ajustează între 0.0 și 50.0 mA
            pulse_width=1600,     # 16-bit ADC, 1600us pulse width (recomandat pentru rezoluție înaltă)
            adc_range=4096,       # Full range 4096 (16-bit) - max value for ADC.
            max_buffer_len=SPO2_CALCULATION_WINDOW_SIZE + 20 # Asigură buffer suficient în obiectul sensor
        )
        
        part_id = sensor.get_part_id()
        if part_id != 0x11: # 0x11 este Part ID pentru MAX30100
            print(f"WARNING: Part ID {hex(part_id)} diferit de cel așteptat (0x11). Asigură-te că este MAX30100.")

        print(f"Senzor MAX30100 (Part ID: {hex(part_id)}) inițializat. Așezați degetul ferm.")

    except Exception as e:
        import sys
        sys.print_exception(e)
        print(f"Eroare la inițializarea senzorului.")
        return

    spo2_calc = SpO2Calculator()
    last_print_time = utime.ticks_ms()
    finger_on = False
    print_interval_ms = 1000 # Afișează SpO2 la fiecare secundă

    while True:
        try:
            # Citește datele brute din FIFO-ul senzorului
            sensor.read_sensor_fifo()
            latest_ir_raw = sensor.ir
            latest_red_raw = sensor.red

            current_ir_smoothed = latest_ir_raw # Valori de fallback dacă nu avem suficiente pentru netezire
            current_red_smoothed = latest_red_raw

            if latest_ir_raw is not None and latest_red_raw is not None:
                # Aplică o medie mobilă pentru a netezi semnalul brut
                raw_ir_smoothing_buffer.append(latest_ir_raw)
                raw_red_smoothing_buffer.append(latest_red_raw)

                if len(raw_ir_smoothing_buffer) > SMOOTHING_WINDOW_SIZE:
                    raw_ir_smoothing_buffer.pop(0)
                    raw_red_smoothing_buffer.pop(0)

                # Netezirea se aplică doar dacă avem suficiente date în bufferul de netezire
                if len(raw_ir_smoothing_buffer) == SMOOTHING_WINDOW_SIZE:
                    current_ir_smoothed = sum(raw_ir_smoothing_buffer) / SMOOTHING_WINDOW_SIZE
                    current_red_smoothed = sum(raw_red_smoothing_buffer) / SMOOTHING_WINDOW_SIZE
                
                # Logică de detecție a degetului bazată pe valorile DC (netezite)
                # O prezență stabilă a degetului este indicată de valori DC ridicate.
                if current_ir_smoothed > FINGER_MIN_IR_VALUE and \
                   current_red_smoothed > FINGER_MIN_RED_VALUE:
                    if not finger_on:
                        print("Deget detectat. Se stabilizează citirile...")
                        finger_on = True
                        spo2_calc.reset() # Resetează bufferul de calcul SpO2 la detectarea degetului
                        # Golește și bufferele de netezire pentru a începe cu date "curate"
                        raw_ir_smoothing_buffer.clear()
                        raw_red_smoothing_buffer.clear()
                    
                    # Adaugă valorile (netezite) la bufferul de calcul SpO2
                    spo2_calc.add_reading(current_red_smoothed, current_ir_smoothed)
                    
                    if utime.ticks_diff(utime.ticks_ms(), last_print_time) >= print_interval_ms:
                        if len(spo2_calc.red_buffer) >= MIN_SAMPLES_FOR_SPO2:
                            calculated_spo2 = spo2_calc.calculate_spo2()
                            # Afișează informații detaliate pentru debug și monitorizare
                            print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | "
                                  f"SpO2: {calculated_spo2:.1f}% | R: {spo2_calc.last_r_value:.3f} "
                                  f"(AC_R:{spo2_calc.last_ac_red_rms:.1f}, DC_R:{spo2_calc.last_dc_red:.0f}, "
                                  f"AC_I:{spo2_calc.last_ac_ir_rms:.1f}, DC_I:{spo2_calc.last_dc_ir:.0f})")
                        else:
                            print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | "
                                  f"SpO2: Calculare... ({len(spo2_calc.red_buffer)}/{MIN_SAMPLES_FOR_SPO2})")
                        last_print_time = utime.ticks_ms()

                elif finger_on:
                    # Degetul a fost îndepărtat sau semnalul a devenit prea slab
                    print("Deget îndepărtat sau semnal slab.")
                    finger_on = False
                    spo2_calc.reset() # Resetează complet calculatorul
                    raw_ir_smoothing_buffer.clear() # Golește bufferele de netezire
                    raw_red_smoothing_buffer.clear()
                    print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | SpO2: ---")
                    last_print_time = utime.ticks_ms()
                
                # Cazul inițial când degetul nu este încă detectat, dar semnalul nu e zero
                else:
                    if utime.ticks_diff(utime.ticks_ms(), last_print_time) >= print_interval_ms:
                        print(f"IR: {current_ir_smoothed:<5.0f} | RED: {current_red_smoothed:<5.0f} | SpO2: Așteaptă degetul...")
                        last_print_time = utime.ticks_ms()

            utime.sleep_ms(POLL_INTERVAL_MS) # Așteaptă înainte de următoarea citire

        except OSError as e:
            # Eroare I2C - Poate fi o deconectare temporară sau problemă hardware
            print(f"Eroare I2C: {e}. Se reîncearcă...")
            utime.sleep_ms(1000)
            try:
                # Încearcă să resetezi și să reconfigurezi senzorul complet
                sensor.reset()
                utime.sleep_ms(100)
                sensor.set_mode(MODE_SPO2)
                sensor.configure_spo2_and_adc(
                    sample_rate=SAMPLE_RATE_SPS,
                    pulse_width=1600, 
                    adc_range=4096
                )
                sensor.set_led_currents(led_current_red=11.0, led_current_ir=11.0)
                sensor.clear_fifo() # Golește FIFO după resetare
                print("Senzor resetat și reconfigurat după eroare I2C.")
            except Exception as reinit_e:
                print(f"Eroare la resetarea senzorului după eroare I2C: {reinit_e}")
                # În caz de eroare persistentă, poate fi necesară o repornire completă
                utime.sleep_ms(5000) # Așteaptă mai mult înainte de a reîncerca

        except Exception as e:
            # Orice altă eroare neașteptată
            import sys
            sys.print_exception(e)
            print(f"Eroare în bucla principală.")
            utime.sleep_ms(1000)

if __name__ == "__main__":
    main()
