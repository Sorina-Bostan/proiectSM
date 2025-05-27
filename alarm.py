import machine
import time
from puls_data import puls 

BUZZER_PIN = 22
LED_PIN = 26

buzzer = machine.PWM(machine.Pin(BUZZER_PIN))
led = machine.Pin(LED_PIN, machine.Pin.OUT)

PULS_MIN = 60
PULS_MAX = 100

def alarma_on():
    led.value(1)
    buzzer.freq(1000)
    buzzer.duty_u16(30000)
    print("Puls anormal! Alarma activată.")

def alarma_off():
    led.value(0)
    buzzer.duty_u16(0)
    print("Puls normal.")

alarma_off()

if puls < PULS_MIN or puls > PULS_MAX:
    alarma_on()
    time.sleep(5)
    alarma_off()
else:
    alarma_off()

buzzer.deinit()

